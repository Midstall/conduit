//! Albion seal-version driver: reads and bumps the per-realm anti-rollback
//! seal-version counters over an `Mmio`. The RMM bumps a realm's version on each
//! (re)seal and stamps the new version into the sealed blob. On unseal it reads
//! the current version and the seal engine rejects any stale blob.
//!
//! Register map (8-byte strided, matches lib/src/seal/seal_version.dart):
//!   0x00 SEL (RW)  0x08 VER_LO (R)  0x10 VER_HI (R)  0x18 BUMP (W bit0)

const Mmio = @import("../mmio.zig");

pub const SealVersion = struct {
    mmio: Mmio,

    const SEL = 0x00;
    const VER_LO = 0x08;
    const VER_HI = 0x10;
    const BUMP = 0x18;

    /// The slot's current monotonic version.
    pub fn current(self: SealVersion, slot: u32) u64 {
        self.mmio.write(u32, SEL, slot);
        const lo = self.mmio.read(u32, VER_LO);
        const hi = self.mmio.read(u32, VER_HI);
        return (@as(u64, hi) << 32) | lo;
    }

    /// Increment the slot's version (on a (re)seal) and return the new value.
    pub fn bump(self: SealVersion, slot: u32) u64 {
        self.mmio.write(u32, SEL, slot);
        self.mmio.write(u32, BUMP, 1);
        return self.current(slot);
    }
};
