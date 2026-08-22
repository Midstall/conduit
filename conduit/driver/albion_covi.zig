//! Albion COVI driver: the SEP/RMM side of the hardware interrupt-mediation core.
//! The RMM programs each realm's interrupt allow-list here. The hardware gate
//! enforces the list inline. The host can inject only a permitted interrupt. The
//! gate puts the RMM's software rule into silicon, so the host cannot bypass it.
//! This is the SEP/Root-side window only. It is not in the AP firewall whitelist.
//!
//! Registers (8-byte strided, match lib/src/covi/covi.dart):
//!   0x00 ARG_REALM (RW)  0x08 ARG_IRQ (RW)  0x10 CMD (W)  0x18 RESULT (RO)
//!   0x20 ALLOW_READ (RO: selected realm's bitmap)  0x28 FAULT (RO)
//!   0x30 ENABLE (RW bit0)

const Mmio = @import("../mmio.zig");

pub const Covi = struct {
    mmio: Mmio,

    const ARG_REALM = 0x00;
    const ARG_IRQ = 0x08;
    const CMD = 0x10;
    const RESULT = 0x18;
    const ALLOW_READ = 0x20;
    const FAULT = 0x28;
    const ENABLE = 0x30;

    const OP_ALLOW = 1;
    const OP_DENY = 2;
    const OP_CLEAR = 3;

    pub const Result = enum(u4) {
        ok = 0,
        range = 2,
        bad_op = 4,
        _,
    };

    fn submit(self: Covi, op: u32, realm: u8, irq: u32) Result {
        self.mmio.write(u32, ARG_REALM, realm);
        self.mmio.write(u32, ARG_IRQ, irq);
        self.mmio.write(u32, CMD, op);
        return @enumFromInt(@as(u4, @truncate(self.mmio.read(u32, RESULT))));
    }

    /// Permit `irq` to be injected into `realm`.
    pub fn allow(self: Covi, realm: u8, irq: u32) Result {
        return self.submit(OP_ALLOW, realm, irq);
    }
    /// Revoke `irq` for `realm`.
    pub fn deny(self: Covi, realm: u8, irq: u32) Result {
        return self.submit(OP_DENY, realm, irq);
    }
    /// Clear all of a realm's interrupt permissions (at realm teardown).
    pub fn clearRealm(self: Covi, realm: u8) Result {
        return self.submit(OP_CLEAR, realm, 0);
    }
    /// Read a realm's current allow bitmap.
    pub fn allowMask(self: Covi, realm: u8) u32 {
        self.mmio.write(u32, ARG_REALM, realm);
        return self.mmio.read(u32, ALLOW_READ);
    }
    /// Turn enforcement on (0 = transparent bypass at reset).
    pub fn setEnable(self: Covi, on: bool) void {
        self.mmio.write(u32, ENABLE, if (on) 1 else 0);
    }
    /// Read the latched fault word ({irq, realm, valid} of the last denied inject).
    pub fn fault(self: Covi) u32 {
        return self.mmio.read(u32, FAULT);
    }
};

const std = @import("std");

// A model of the covi hardware allow-list, so the driver round-trips against it.
const MockCovi = struct {
    arg_realm: u8 = 0,
    arg_irq: u32 = 0,
    allow: [16]u32 = [_]u32{0} ** 16,
    result: u32 = 0,

    fn rd(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const m: *MockCovi = @ptrCast(@alignCast(ctx.?));
        return switch (off) {
            0x18 => m.result,
            0x20 => m.allow[m.arg_realm & 0xf],
            else => 0,
        };
    }
    fn wr(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const m: *MockCovi = @ptrCast(@alignCast(ctx.?));
        const bit = @as(u32, 1) << @as(u5, @truncate(m.arg_irq));
        switch (off) {
            0x00 => m.arg_realm = @truncate(val),
            0x08 => m.arg_irq = @truncate(val),
            0x10 => switch (val & 0xf) {
                1 => {
                    if (m.arg_irq >= 32) {
                        m.result = 2;
                    } else {
                        m.allow[m.arg_realm & 0xf] |= bit;
                        m.result = 0;
                    }
                },
                2 => {
                    m.allow[m.arg_realm & 0xf] &= ~bit;
                    m.result = 0;
                },
                3 => {
                    m.allow[m.arg_realm & 0xf] = 0;
                    m.result = 0;
                },
                else => m.result = 4,
            },
            else => {},
        }
    }
    fn mmio(self: *MockCovi) Mmio {
        return .{ .ctx = self, .base = 0, .read_fn = rd, .write_fn = wr };
    }
};

test "COVI driver programs and reads back a realm's interrupt allow-list" {
    var mock = MockCovi{};
    const covi = Covi{ .mmio = mock.mmio() };

    try std.testing.expectEqual(Covi.Result.ok, covi.allow(3, 5));
    try std.testing.expectEqual(Covi.Result.ok, covi.allow(3, 9));
    try std.testing.expectEqual(@as(u32, (1 << 5) | (1 << 9)), covi.allowMask(3));

    try std.testing.expectEqual(Covi.Result.ok, covi.deny(3, 5));
    try std.testing.expectEqual(@as(u32, 1 << 9), covi.allowMask(3));

    try std.testing.expectEqual(Covi.Result.ok, covi.clearRealm(3));
    try std.testing.expectEqual(@as(u32, 0), covi.allowMask(3));

    // a different realm is independent.
    try std.testing.expectEqual(Covi.Result.ok, covi.allow(4, 1));
    try std.testing.expectEqual(@as(u32, 1 << 1), covi.allowMask(4));
}
