//! Albion COVI injection-aperture driver: the AP-facing fast path for interrupt
//! injection (the only AP-reachable injection surface, firewall-whitelisted). A
//! write presents `{realm, irq}` to the COVI gate. A permitted injection becomes
//! pending for the realm, a denied one is dropped (and the COVI faults). The AP's
//! realm-entry path reads the pending set and acks delivered interrupts. The
//! per-realm allow-list itself lives on the SEP-only COVI control plane.
//!
//! Registers (8-byte strided, match lib/src/covi/covi_inject.dart):
//!   0x00 INJECT (W: data = (realm<<8)|irq)  0x08 SEL (RW)  0x10 PENDING (RO)
//!   0x18 ACK (W: clear bit `data` of SEL's pending)

const Mmio = @import("../mmio.zig");

pub const CoviInject = struct {
    mmio: Mmio,

    const INJECT = 0x00;
    const SEL = 0x08;
    const PENDING = 0x10;
    const ACK = 0x18;

    /// Request injection of `irq` into `realm` (gated inline by the COVI gate).
    pub fn inject(self: CoviInject, realm: u8, irq: u32) void {
        self.mmio.write(u32, INJECT, (@as(u32, realm) << 8) | (irq & 0xff));
    }

    /// Read a realm's pending (permitted-and-undelivered) injection bitmap.
    pub fn readPending(self: CoviInject, realm: u8) u32 {
        self.mmio.write(u32, SEL, realm);
        return self.mmio.read(u32, PENDING);
    }

    /// Acknowledge (clear) a delivered interrupt from a realm's pending set.
    pub fn ackPending(self: CoviInject, realm: u8, irq: u32) void {
        self.mmio.write(u32, SEL, realm);
        self.mmio.write(u32, ACK, irq);
    }
};

const std = @import("std");

// Models the aperture's per-realm pending bitmaps. The allow-list gating lives in
// the COVI gate and its ROHD test, so here we only exercise the inject/read/ack
// MMIO.
const MockAperture = struct {
    sel: u8 = 0,
    pending: [16]u32 = [_]u32{0} ** 16,

    fn rd(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const m: *MockAperture = @ptrCast(@alignCast(ctx.?));
        return switch (off) {
            0x10 => m.pending[m.sel & 0xf],
            else => 0,
        };
    }
    fn wr(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const m: *MockAperture = @ptrCast(@alignCast(ctx.?));
        switch (off) {
            0x00 => { // INJECT: (realm<<8)|irq, permitted here (gating is the gate's job).
                const realm: u8 = @truncate((val >> 8) & 0xf);
                const irq: u5 = @truncate(val & 0x1f);
                m.pending[realm] |= @as(u32, 1) << irq;
            },
            0x08 => m.sel = @truncate(val),
            0x18 => m.pending[m.sel & 0xf] &= ~(@as(u32, 1) << @as(u5, @truncate(val))),
            else => {},
        }
    }
    fn mmio(self: *MockAperture) Mmio {
        return .{ .ctx = self, .base = 0, .read_fn = rd, .write_fn = wr };
    }
};

test "COVI inject aperture: inject pends, read, ack clears" {
    var mock = MockAperture{};
    const ap = CoviInject{ .mmio = mock.mmio() };

    ap.inject(3, 5);
    ap.inject(3, 9);
    try std.testing.expectEqual(@as(u32, (1 << 5) | (1 << 9)), ap.readPending(3));
    ap.ackPending(3, 5);
    try std.testing.expectEqual(@as(u32, 1 << 9), ap.readPending(3));
    // a different realm is independent.
    try std.testing.expectEqual(@as(u32, 0), ap.readPending(4));
}
