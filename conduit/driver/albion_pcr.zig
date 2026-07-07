//! Albion PCR-bank driver, over an `Mmio`.
//!
//! The hardware TPM PCR bank (`AlbionTpm`, default window 0x4000_3000): a bank of
//! 256-bit Platform Configuration Registers with a hardware SHA-256 extend
//! (PCR = SHA-256(PCR_old || data)). The fTPM firmware drives this for the real
//! PCR_Extend/PCR_Read primitives. 32-bit word accesses, digests are big-endian
//! (word 0 = the most-significant 32 bits).
//!   SELECT   0x00 (RW): active PCR index
//!   EXTEND   0x04 (W bit0 = start, R bit0 = busy)
//!   STATUS   0x08 (RO): bit0 = busy, bit1 = external TPM present
//!   DATA[i]  0x10 + 4*i (RW, i in 0..7): 256-bit measurement to fold in
//!   PCR[i]   0x40 + 4*i (RO, i in 0..7): the selected PCR's 256-bit value

const Mmio = @import("../mmio.zig");

pub const Pcr = struct {
    mmio: Mmio,

    const SELECT = 0x00;
    const EXTEND = 0x04;
    const STATUS = 0x08;
    const DATA = 0x10;
    const PCR = 0x40;

    /// Select the active PCR.
    pub fn select(self: Pcr, idx: u32) void {
        self.mmio.write(u32, SELECT, idx);
    }

    /// True while an extend is in progress.
    pub fn busy(self: Pcr) bool {
        return (self.mmio.read(u32, STATUS) & 0x1) != 0;
    }

    /// Extend PCR `idx` with a 32-byte digest: PCR = SHA-256(PCR_old || digest).
    /// Blocks until the hardware extend completes.
    pub fn extend(self: Pcr, idx: u32, digest: [32]u8) void {
        self.select(idx);
        for (0..8) |i| {
            const w = (@as(u32, digest[4 * i]) << 24) |
                (@as(u32, digest[4 * i + 1]) << 16) |
                (@as(u32, digest[4 * i + 2]) << 8) |
                @as(u32, digest[4 * i + 3]);
            self.mmio.write(u32, DATA + 4 * i, w);
        }
        self.mmio.write(u32, EXTEND, 1);
        while (self.busy()) {}
    }

    /// Read PCR `idx`'s current 32-byte value (big-endian) into `out`.
    pub fn read(self: Pcr, idx: u32, out: *[32]u8) void {
        self.select(idx);
        for (0..8) |i| {
            const w = self.mmio.read(u32, PCR + 4 * i);
            out[4 * i] = @truncate(w >> 24);
            out[4 * i + 1] = @truncate(w >> 16);
            out[4 * i + 2] = @truncate(w >> 8);
            out[4 * i + 3] = @truncate(w);
        }
    }
};

pub fn bind(mmio: Mmio) Pcr {
    return .{ .mmio = mmio };
}
