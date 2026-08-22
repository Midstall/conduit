//! Linux PCI backend: read-only enumeration of `/sys/bus/pci/devices/*`.
//!
//! Each entry there is a directory named `DDDD:BB:DD.F` (segment:bus:device.
//! function). The `vendor`/`device`/`class`/`revision` sysfs attribute files
//! give config-space identity. The `resource` file's lines are "start end flags"
//! per BAR (BAR0..5 then the expansion ROM), and a populated BAR has start != 0,
//! size = end - start + 1. All of these are world-readable, so this runs as a
//! normal user with no root. It exposes BAR addresses only and never reads BAR
//! register contents (that is the consumer's bare-metal concern, and would need
//! root/mmap).
//!
//! Pure `std.os.linux` syscalls (open/read/close/getdents64) with
//! `std.posix.errno`, no libc, no allocator. Paths handed to syscalls are
//! NUL-terminated.

const std = @import("std");
const backend = @import("../../backend.zig");
const resource = @import("../../resource.zig");
const pci = @import("../pci.zig");

const linux = std.os.linux;

const sysfs_root = "/sys/bus/pci/devices";

/// Max length of a PCI device directory name ("DDDD:BB:DD.F" = 12 chars).
const addr_name_max = 16;

pub const LinuxPciBackend = struct {
    /// fd of the open `/sys/bus/pci/devices` directory (getdents64 cursor), or -1.
    dir_fd: i32 = -1,
    /// getdents64 buffer + the byte window we have not yet consumed.
    dents: [4096]u8 = undefined,
    dents_len: usize = 0,
    dents_off: usize = 0,
    /// The directory name of the node most recently returned by `next`, so
    /// `resources` can re-open its `resource` file.
    cur_name: [addr_name_max]u8 = undefined,
    cur_name_len: usize = 0,

    pub fn init() LinuxPciBackend {
        return .{};
    }

    pub fn any(self: *LinuxPciBackend) backend.Backend {
        return backend.fromImpl(self);
    }

    pub fn reset(self: *LinuxPciBackend) void {
        if (self.dir_fd >= 0) {
            _ = linux.close(self.dir_fd);
            self.dir_fd = -1;
        }
        self.dents_len = 0;
        self.dents_off = 0;
        self.cur_name_len = 0;
    }

    pub fn next(self: *LinuxPciBackend) backend.Error!?backend.Node {
        if (self.dir_fd < 0) {
            const fd = linux.open(sysfs_root, .{ .ACCMODE = .RDONLY, .DIRECTORY = true }, 0);
            if (signed(fd) < 0) return null; // no PCI sysfs (e.g. a container): empty.
            self.dir_fd = @intCast(fd);
        }

        while (true) {
            const name = (try self.nextDirEnt()) orelse return null;
            // PCI device dirs are "DDDD:BB:DD.F", so skip "." / ".." and anything else.
            if (!looksLikePciAddr(name)) continue;

            const info = self.readIdentity(name) orelse continue;

            // Remember the dir name so `resources` can read its `resource` file.
            self.cur_name_len = @min(name.len, self.cur_name.len);
            @memcpy(self.cur_name[0..self.cur_name_len], name[0..self.cur_name_len]);

            return pci.synthIds(info);
        }
    }

    pub fn resources(self: *LinuxPciBackend, node: backend.Node, out: *resource.List) backend.Error!void {
        _ = node;
        out.* = .{};
        if (self.cur_name_len == 0) return;

        var path_buf: [128]u8 = undefined;
        const path = std.fmt.bufPrintZ(&path_buf, "{s}/{s}/resource", .{ sysfs_root, self.cur_name[0..self.cur_name_len] }) catch return;

        var file_buf: [4096]u8 = undefined;
        const n = readFile(path, &file_buf) orelse return;
        const text = file_buf[0..n];

        // Each line: "0xSTART 0xEND 0xFLAGS". BARs in order: BAR0..5 then ROM.
        var lines = std.mem.splitScalar(u8, text, '\n');
        var bar: usize = 0;
        while (lines.next()) |line| : (bar += 1) {
            if (bar >= 6) break; // only BAR0..5. Ignore the expansion ROM line.
            if (line.len == 0) continue;
            const r = parseResourceLine(line) orelse continue;
            if (r.start == 0) continue; // unpopulated BAR.
            const size = r.end - r.start + 1;
            // PCI resource flag bit 8 (0x100) marks an IO-space BAR, else memory.
            if (r.flags & 0x100 != 0) {
                try out.append(.{ .reg_io = .{ .port = @truncate(r.start), .size = @truncate(@min(size, 0xFFFF)) } });
            } else {
                try out.append(.{ .mmio = .{ .base = r.start, .size = size } });
            }
        }
    }

    /// Pull the next directory entry name from the getdents64 stream.
    fn nextDirEnt(self: *LinuxPciBackend) backend.Error!?[]const u8 {
        while (true) {
            if (self.dents_off >= self.dents_len) {
                const rc = linux.getdents64(self.dir_fd, &self.dents, self.dents.len);
                const s = signed(rc);
                if (s < 0) return error.BadFormat;
                if (s == 0) return null; // end of directory.
                self.dents_len = @intCast(s);
                self.dents_off = 0;
            }
            const d: *align(1) linux.dirent64 = @ptrCast(&self.dents[self.dents_off]);
            const reclen: usize = d.reclen;
            if (reclen == 0 or self.dents_off + reclen > self.dents_len) {
                // Defensive: a malformed record, so stop rather than loop.
                self.dents_off = self.dents_len;
                continue;
            }
            self.dents_off += reclen;
            const name_ptr: [*:0]const u8 = @ptrCast(&d.name);
            const name = std.mem.span(name_ptr);
            if (name.len == 0) continue;
            return name;
        }
    }

    /// Read vendor/device/class/revision from a device's sysfs attribute files.
    fn readIdentity(self: *LinuxPciBackend, name: []const u8) ?backend.PciInfo {
        _ = self;
        const addr = parsePciAddr(name) orelse return null;

        var info = backend.PciInfo{
            .segment = addr.segment,
            .bus = addr.bus,
            .device = addr.device,
            .function = addr.function,
        };

        info.vendor_id = @truncate(readHexAttr(name, "vendor") orelse return null);
        info.device_id = @truncate(readHexAttr(name, "device") orelse return null);
        // class is a 24-bit value 0xCCSSPP (class, subclass, prog-if).
        const class24 = readHexAttr(name, "class") orelse 0;
        info.class_code = @truncate(class24 >> 16);
        info.subclass = @truncate(class24 >> 8);
        info.prog_if = @truncate(class24);
        info.revision = @truncate(readHexAttr(name, "revision") orelse 0);
        return info;
    }
};

const Addr = struct { segment: u16, bus: u8, device: u8, function: u8 };

const ResLine = struct { start: u64, end: u64, flags: u64 };

fn signed(rc: usize) isize {
    return @bitCast(rc);
}

/// True if `name` has the "DDDD:BB:DD.F" shape (segment:bus:device.function).
fn looksLikePciAddr(name: []const u8) bool {
    return parsePciAddr(name) != null;
}

/// Parse "DDDD:BB:DD.F" -> segment/bus/device/function (all hex).
fn parsePciAddr(name: []const u8) ?Addr {
    // 0000:00:00.0 = 12 chars exactly.
    if (name.len != 12) return null;
    if (name[4] != ':' or name[7] != ':' or name[10] != '.') return null;
    const seg = parseHex(u16, name[0..4]) orelse return null;
    const bus = parseHex(u8, name[5..7]) orelse return null;
    const dev = parseHex(u8, name[8..10]) orelse return null;
    const func = parseHex(u8, name[11..12]) orelse return null;
    return .{ .segment = seg, .bus = bus, .device = dev, .function = func };
}

fn parseHex(comptime T: type, s: []const u8) ?T {
    return std.fmt.parseInt(T, s, 16) catch null;
}

/// Read a sysfs hex attribute file ("0x....\n") for device `name`.
fn readHexAttr(name: []const u8, attr: []const u8) ?u64 {
    var path_buf: [128]u8 = undefined;
    const path = std.fmt.bufPrintZ(&path_buf, "{s}/{s}/{s}", .{ sysfs_root, name, attr }) catch return null;
    var buf: [32]u8 = undefined;
    const n = readFile(path, &buf) orelse return null;
    var text = buf[0..n];
    // Trim trailing newline / whitespace.
    while (text.len > 0 and (text[text.len - 1] == '\n' or text[text.len - 1] == ' ' or text[text.len - 1] == '\r')) {
        text = text[0 .. text.len - 1];
    }
    if (std.mem.startsWith(u8, text, "0x") or std.mem.startsWith(u8, text, "0X")) text = text[2..];
    return std.fmt.parseInt(u64, text, 16) catch null;
}

/// Parse one "0xSTART 0xEND 0xFLAGS" resource line.
fn parseResourceLine(line: []const u8) ?ResLine {
    var it = std.mem.tokenizeScalar(u8, line, ' ');
    const start = parseHexField(it.next() orelse return null) orelse return null;
    const end = parseHexField(it.next() orelse return null) orelse return null;
    const flags = parseHexField(it.next() orelse return null) orelse 0;
    return .{ .start = start, .end = end, .flags = flags };
}

fn parseHexField(s: []const u8) ?u64 {
    var t = s;
    if (std.mem.startsWith(u8, t, "0x") or std.mem.startsWith(u8, t, "0X")) t = t[2..];
    return std.fmt.parseInt(u64, t, 16) catch null;
}

/// Read a whole (small) file into `buf`, returning the byte count, or null on
/// any error. Read-only, NUL-terminated path, raw syscalls.
fn readFile(path: [:0]const u8, buf: []u8) ?usize {
    const fd_rc = linux.open(path.ptr, .{ .ACCMODE = .RDONLY }, 0);
    if (signed(fd_rc) < 0) return null;
    const fd: i32 = @intCast(fd_rc);
    defer _ = linux.close(fd);

    var total: usize = 0;
    while (total < buf.len) {
        const rc = linux.read(fd, buf[total..].ptr, buf.len - total);
        const s = signed(rc);
        if (s < 0) return null;
        if (s == 0) break;
        total += @intCast(s);
    }
    return total;
}

test "parse a PCI address" {
    const a = parsePciAddr("0001:01:00.0").?;
    try std.testing.expectEqual(@as(u16, 1), a.segment);
    try std.testing.expectEqual(@as(u8, 1), a.bus);
    try std.testing.expectEqual(@as(u8, 0), a.device);
    try std.testing.expectEqual(@as(u8, 0), a.function);
    try std.testing.expect(parsePciAddr("..") == null);
    try std.testing.expect(parsePciAddr("0000:00:00") == null);
}

test "parse a resource line" {
    const r = parseResourceLine("0x0000000060000000 0x0000000063ffffff 0x0000000000040200").?;
    try std.testing.expectEqual(@as(u64, 0x60000000), r.start);
    try std.testing.expectEqual(@as(u64, 0x63ffffff), r.end);
    try std.testing.expectEqual(@as(u64, 0x0400_0000), r.end - r.start + 1); // 64 MiB
    // An unpopulated BAR line.
    const z = parseResourceLine("0x0000000000000000 0x0000000000000000 0x0000000000000000").?;
    try std.testing.expectEqual(@as(u64, 0), z.start);
}
