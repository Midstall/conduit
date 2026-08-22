//! The serial / UART device-class contract. A vtable over any byte-stream UART.
//! Drivers (driver/ns16550a.zig, driver/pl011.zig) implement the small method
//! set and wrap themselves with `Serial.from`.

const std = @import("std");
const Serial = @This();

ctx: ?*anyopaque,
write_fn: *const fn (ctx: ?*anyopaque, bytes: []const u8) void,
read_fn: *const fn (ctx: ?*anyopaque, buf: []u8) usize,
tx_ready_fn: *const fn (ctx: ?*anyopaque) bool,
rx_ready_fn: *const fn (ctx: ?*anyopaque) bool,

/// Write all bytes (blocking until each is accepted).
pub fn write(self: Serial, bytes: []const u8) void {
    self.write_fn(self.ctx, bytes);
}

/// Read currently-available bytes into `buf`. Return the count (may be 0).
pub fn read(self: Serial, buf: []u8) usize {
    return self.read_fn(self.ctx, buf);
}

/// True if the transmitter can accept a byte now.
pub fn txReady(self: Serial) bool {
    return self.tx_ready_fn(self.ctx);
}

/// True if a received byte is waiting.
pub fn rxReady(self: Serial) bool {
    return self.rx_ready_fn(self.ctx);
}

/// Wrap a driver pointer (with `write`/`read`/`txReady`/`rxReady` methods) into
/// a `Serial`.
pub fn from(impl: anytype) Serial {
    const P = @TypeOf(impl);
    const gen = struct {
        fn write(ctx: ?*anyopaque, bytes: []const u8) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.write(bytes);
        }
        fn read(ctx: ?*anyopaque, buf: []u8) usize {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.read(buf);
        }
        fn txReady(ctx: ?*anyopaque) bool {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.txReady();
        }
        fn rxReady(ctx: ?*anyopaque) bool {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.rxReady();
        }
    };
    return .{
        .ctx = impl,
        .write_fn = gen.write,
        .read_fn = gen.read,
        .tx_ready_fn = gen.txReady,
        .rx_ready_fn = gen.rxReady,
    };
}

/// A `std.Io.Writer` over a serial port. It sends every write straight to the
/// UART through `Serial.write`, so `std.fmt` formatting and any std API that
/// takes a writer reach the console. Store it at a stable address and pass
/// `&w.interface` to consumers. Bytes go out on each write. Pass an empty
/// `buffer` for an unbuffered writer, so no bytes wait for a flush.
pub const Writer = struct {
    serial: Serial,
    interface: std.Io.Writer,

    const vtable: std.Io.Writer.VTable = .{ .drain = drain };

    fn drain(w: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
        const self: *Writer = @alignCast(@fieldParentPtr("interface", w));
        // Send the buffered bytes, then each vector, then the last one `splat`
        // times. Serial.write blocks until the UART accepts each byte, so it
        // never fails.
        if (w.end != 0) self.serial.write(w.buffer[0..w.end]);
        w.end = 0;
        // An unbuffered writer can reach drain with no vectors. Guard the empty
        // case, or `data.len - 1` below wraps and reads a bogus slice.
        if (data.len == 0) return 0;
        const head = data[0 .. data.len - 1];
        for (head) |bytes| self.serial.write(bytes);
        const last = data[head.len];
        var i: usize = 0;
        while (i < splat) : (i += 1) self.serial.write(last);
        // Report the bytes taken from `data` (the buffer does not count).
        var written: usize = last.len * splat;
        for (head) |bytes| written += bytes.len;
        return written;
    }
};

/// Wrap this serial port as a `std.Io.Writer`. `buffer` is formatting scratch.
/// An empty buffer makes the writer unbuffered, so it sends every byte at once.
pub fn writer(self: Serial, buffer: []u8) Writer {
    return .{ .serial = self, .interface = .{ .vtable = &Writer.vtable, .buffer = buffer } };
}

/// A `std.Io.Reader` over a serial port. It reads only the bytes the UART holds
/// now and never blocks: a read returns 0 when nothing waits, which the
/// `std.Io.Reader` contract allows (0 does not mean end of stream). Store it at
/// a stable address and pass `&r.interface` to consumers. An empty `buffer`
/// makes it unbuffered, so a read reflects the UART state at that instant.
pub const Reader = struct {
    serial: Serial,
    interface: std.Io.Reader,

    const vtable: std.Io.Reader.VTable = .{ .stream = stream, .readVec = readVec };

    fn stream(r: *std.Io.Reader, w: *std.Io.Writer, limit: std.Io.Limit) std.Io.Reader.StreamError!usize {
        const self: *Reader = @alignCast(@fieldParentPtr("interface", r));
        var tmp: [64]u8 = undefined;
        const n = self.serial.read(limit.slice(&tmp));
        if (n == 0) return 0;
        try w.writeAll(tmp[0..n]);
        return n;
    }

    fn readVec(r: *std.Io.Reader, data: [][]u8) std.Io.Reader.Error!usize {
        const self: *Reader = @alignCast(@fieldParentPtr("interface", r));
        if (data.len == 0 or data[0].len == 0) return 0;
        return self.serial.read(data[0]);
    }
};

/// Wrap this serial port as a `std.Io.Reader`. Pass an empty `buffer` for an
/// unbuffered reader.
pub fn reader(self: Serial, buffer: []u8) Reader {
    return .{
        .serial = self,
        .interface = .{ .vtable = &Reader.vtable, .buffer = buffer, .seek = 0, .end = 0 },
    };
}

const CaptureSerial = struct {
    out: *std.ArrayList(u8),
    gpa: std.mem.Allocator,

    fn write(self: *CaptureSerial, bytes: []const u8) void {
        self.out.appendSlice(self.gpa, bytes) catch @panic("out of memory");
    }
    fn read(_: *CaptureSerial, _: []u8) usize {
        return 0;
    }
    fn txReady(_: *CaptureSerial) bool {
        return true;
    }
    fn rxReady(_: *CaptureSerial) bool {
        return false;
    }
};

test "unbuffered writer drain tolerates an empty vector" {
    // A zero-length buffer makes std call drain with no vectors (data.len == 0).
    // Before the fix, `data.len - 1` wrapped and the drain read a bogus slice off
    // the stack. This regression test drives that exact call.
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(std.testing.allocator);
    var cap = CaptureSerial{ .out = &out, .gpa = std.testing.allocator };
    var empty: [0]u8 = .{};
    var w = writer(Serial.from(&cap), &empty);
    const n = try Writer.drain(&w.interface, &.{}, 1);
    try std.testing.expectEqual(@as(usize, 0), n);
    try std.testing.expectEqual(@as(usize, 0), out.items.len);
}

test "unbuffered writer prints formatted output through drain" {
    var out: std.ArrayList(u8) = .empty;
    defer out.deinit(std.testing.allocator);
    var cap = CaptureSerial{ .out = &out, .gpa = std.testing.allocator };
    var empty: [0]u8 = .{};
    var w = writer(Serial.from(&cap), &empty);
    // Padding drives the splat and vector path that the empty-buffer bug hit.
    try w.interface.print("[{s}] {X:0>8}\n", .{ "acpi", @as(u32, 0x1234) });
    try std.testing.expectEqualStrings("[acpi] 00001234\n", out.items);
}
