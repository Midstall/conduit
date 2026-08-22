//! Driver round-trip tests. Each test binds a driver over a fake in-memory
//! register file. It asserts the correct register traffic. It exercises the
//! `Mmio` seam and the `Serial` contract without hardware.

const std = @import("std");
const Mmio = @import("conduit").Mmio;
const ns16550a = @import("conduit").driver.ns16550a;
const pl011 = @import("conduit").driver.pl011;

/// A byte-addressed fake register file. It forces `ready_off`/`ready_bit` high so
/// polling writes never spin.
const FakeRegs = struct {
    buf: [256]u8 = [_]u8{0} ** 256,
    ready_off: usize,
    ready_bit: u8,

    fn read(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        const self: *FakeRegs = @ptrCast(@alignCast(ctx));
        if (off == self.ready_off) return self.ready_bit; // tx always ready
        return self.load(off, width);
    }

    fn write(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        const self: *FakeRegs = @ptrCast(@alignCast(ctx));
        self.store(off, width, val);
    }

    fn load(self: *FakeRegs, off: usize, width: Mmio.Width) u64 {
        return switch (width) {
            .byte => self.buf[off],
            .half => std.mem.readInt(u16, self.buf[off..][0..2], .little),
            .word => std.mem.readInt(u32, self.buf[off..][0..4], .little),
            .dword => std.mem.readInt(u64, self.buf[off..][0..8], .little),
        };
    }

    fn store(self: *FakeRegs, off: usize, width: Mmio.Width, val: u64) void {
        switch (width) {
            .byte => self.buf[off] = @truncate(val),
            .half => std.mem.writeInt(u16, self.buf[off..][0..2], @truncate(val), .little),
            .word => std.mem.writeInt(u32, self.buf[off..][0..4], @truncate(val), .little),
            .dword => std.mem.writeInt(u64, self.buf[off..][0..8], val, .little),
        }
    }

    fn mmio(self: *FakeRegs) Mmio {
        return .{ .ctx = self, .base = 0, .read_fn = read, .write_fn = write };
    }
};

test "ns16550a: init programs 8N1 and write lands in THR" {
    var regs = FakeRegs{ .ready_off = 5, .ready_bit = 0x20 }; // LSR THRE
    var uart = ns16550a.bind(regs.mmio(), .{});

    try std.testing.expectEqual(@as(u8, 0x03), regs.buf[3]); // LCR = 8N1 from init

    const s = uart.serial();
    s.write("hi");
    try std.testing.expectEqual(@as(u8, 'i'), regs.buf[0]); // last byte at THR
}

test "pl011: init enables UART and write lands in DR" {
    var regs = FakeRegs{ .ready_off = 0x18, .ready_bit = 0x00 }; // FR: TXFF clear => ready
    var uart = pl011.bind(regs.mmio());

    try std.testing.expectEqual(@as(u32, 0x301), std.mem.readInt(u32, regs.buf[0x30..][0..4], .little)); // CR

    const s = uart.serial();
    s.write("ok");
    try std.testing.expectEqual(@as(u8, 'k'), regs.buf[0]); // last byte at DR
}
