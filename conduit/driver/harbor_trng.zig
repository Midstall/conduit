//! Harbor TRNG driver, over an `Mmio`.
//!
//! The reusable `HarborTrng` block (a DRBG with SP 800-90B health tests). The
//! fTPM firmware pulls randomness from it for TPM2_GetRandom and key generation.
//! On the Albion 64-bit fabric the registers are 8-BYTE STRIDED (each 32-bit
//! register on the low data lanes, so a 0x04 offset would land on the high lanes),
//! matching HarborTrng's `busDataWidth >= 64` layout.
//!   RAND   0x00 (RO): next DRBG word (reading advances the generator)
//!   STATUS 0x08 (RO): bit0 = ready, bit1 = health-test failed

const Mmio = @import("../mmio.zig");

pub const Trng = struct {
    mmio: Mmio,

    const RAND = 0x00;
    const STATUS = 0x08;

    /// True once the DRBG is seeded and producing output.
    pub fn ready(self: Trng) bool {
        return (self.mmio.read(u32, STATUS) & 0x1) != 0;
    }

    /// True while the continuous health tests have not flagged a fault.
    pub fn healthy(self: Trng) bool {
        return (self.mmio.read(u32, STATUS) & 0x2) == 0;
    }

    /// Next 32-bit random word.
    pub fn next(self: Trng) u32 {
        return self.mmio.read(u32, RAND);
    }

    /// Fill `out` with random bytes (little-endian word extraction).
    pub fn fill(self: Trng, out: []u8) void {
        var i: usize = 0;
        while (i + 4 <= out.len) : (i += 4) {
            const w = self.next();
            out[i] = @truncate(w);
            out[i + 1] = @truncate(w >> 8);
            out[i + 2] = @truncate(w >> 16);
            out[i + 3] = @truncate(w >> 24);
        }
        if (i < out.len) {
            var w = self.next();
            while (i < out.len) : (i += 1) {
                out[i] = @truncate(w);
                w >>= 8;
            }
        }
    }
};

pub fn bind(mmio: Mmio) Trng {
    return .{ .mmio = mmio };
}
