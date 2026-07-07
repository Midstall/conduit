//! Albion integrity-descriptor driver, over an `Mmio`.
//!
//! The hardware sticky-fail attestation register (`AlbionIntegrity`, default
//! window 0x4000_D000): one bit per platform component, latched VERIFIED out of
//! reset. The SEP firmware marks a component FAILED (write-1-to-fail), and the bit
//! then stays cleared until reset (monotonic, malware cannot forge a clean
//! attestation). The AP reads the live vector for attestation but cannot write.
//! Registers are 8-byte-strided (low-lane convention for the SEP's 64-bit bus).
//!   STATUS     0x00 (RO): bit i = component i verified (1) / failed (0).
//!   FAIL       0x08 (W):  write-1-to-fail, each set bit permanently clears that
//!                         component (writing 0 is a no-op).
//!   COMPONENTS 0x10 (RO): number of defined components.
//!
//! Component bit assignment (mirrors `AlbionIntegrity`): 0 = secure-boot,
//! 1 = SEP software (signature enforcement active), 2 = fTPM.

const Mmio = @import("../mmio.zig");

pub const Integrity = struct {
    mmio: Mmio,

    const STATUS = 0x00;
    const FAIL = 0x08;
    const COMPONENTS = 0x10;

    /// Component bit: secure-boot (the boot ROM verified the signed payload).
    pub const component_secure_boot: u5 = 0;
    /// Component bit: SEP software (Ferrite signature enforcement is active).
    pub const component_sep_software: u5 = 1;
    /// Component bit: the fTPM.
    pub const component_ftpm: u5 = 2;

    /// Read the live integrity vector (bit i = component i verified).
    pub fn vector(self: Integrity) u32 {
        return self.mmio.read(u32, STATUS);
    }

    /// Number of defined components.
    pub fn components(self: Integrity) u32 {
        return self.mmio.read(u32, COMPONENTS);
    }

    /// Mark every component whose bit is set in `mask` as FAILED (sticky).
    pub fn markFailed(self: Integrity, mask: u32) void {
        self.mmio.write(u32, FAIL, mask);
    }

    /// Mark a single component (by bit index) FAILED.
    pub fn markComponentFailed(self: Integrity, component: u5) void {
        self.markFailed(@as(u32, 1) << component);
    }
};

pub fn bind(mmio: Mmio) Integrity {
    return .{ .mmio = mmio };
}
