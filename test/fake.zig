//! Fake MMIO backings for driver round-trip tests.

const std = @import("std");
const Mmio = @import("conduit").Mmio;

/// A flat little-endian register file of `N` bytes. `force_off`/`force_val` pin
/// one register's read value and discard writes to it. This models the status
/// bits a poll loop waits on. Use it for small-offset drivers.
pub fn Flat(comptime N: usize) type {
    return struct {
        const Self = @This();
        pub const no_force = ~@as(usize, 0);

        buf: [N]u8 = [_]u8{0} ** N,
        force_off: usize = no_force,
        force_val: u64 = 0,

        fn read(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
            const self: *Self = @ptrCast(@alignCast(ctx));
            if (off == self.force_off) return self.force_val;
            return switch (width) {
                .byte => self.buf[off],
                .half => std.mem.readInt(u16, self.buf[off..][0..2], .little),
                .word => std.mem.readInt(u32, self.buf[off..][0..4], .little),
                .dword => std.mem.readInt(u64, self.buf[off..][0..8], .little),
            };
        }

        fn write(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
            const self: *Self = @ptrCast(@alignCast(ctx));
            if (off == self.force_off) return;
            switch (width) {
                .byte => self.buf[off] = @truncate(val),
                .half => std.mem.writeInt(u16, self.buf[off..][0..2], @truncate(val), .little),
                .word => std.mem.writeInt(u32, self.buf[off..][0..4], @truncate(val), .little),
                .dword => std.mem.writeInt(u64, self.buf[off..][0..8], val, .little),
            }
        }

        pub fn mmio(self: *Self) Mmio {
            return .{ .ctx = self, .base = 0, .read_fn = read, .write_fn = write };
        }
    };
}

/// A sparse register file. It records writes. It serves reads from the last write
/// to that offset, or from a preset read override. Use it for drivers with huge
/// offsets where a flat buffer is impractical, like the PLIC claim register at
/// 0x200004 or GIC distributor banks.
pub const Sparse = struct {
    const Entry = struct { off: usize, val: u64 };

    writes: [128]Entry = undefined,
    nw: usize = 0,
    reads: [16]Entry = undefined,
    nr: usize = 0,

    fn read(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const self: *Sparse = @ptrCast(@alignCast(ctx));
        var i = self.nr;
        while (i > 0) {
            i -= 1;
            if (self.reads[i].off == off) return self.reads[i].val;
        }
        return self.lastWrite(off) orelse 0;
    }

    fn writeFn(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const self: *Sparse = @ptrCast(@alignCast(ctx));
        if (self.nw < self.writes.len) {
            self.writes[self.nw] = .{ .off = off, .val = val };
            self.nw += 1;
        }
    }

    /// The most recent value written to `off`, if any.
    pub fn lastWrite(self: *const Sparse, off: usize) ?u64 {
        var i = self.nw;
        while (i > 0) {
            i -= 1;
            if (self.writes[i].off == off) return self.writes[i].val;
        }
        return null;
    }

    /// Pin a read value at `off`, for example a GIC IAR or PLIC claim register.
    pub fn setRead(self: *Sparse, off: usize, val: u64) void {
        self.reads[self.nr] = .{ .off = off, .val = val };
        self.nr += 1;
    }

    pub fn mmio(self: *Sparse) Mmio {
        return .{ .ctx = self, .base = 0, .read_fn = read, .write_fn = writeFn };
    }
};
