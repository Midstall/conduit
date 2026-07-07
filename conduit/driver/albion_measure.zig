//! Albion measurement-bank driver: drives the per-realm RIM/REM registers over an
//! `Mmio`. The RMM allocates a slot per realm, extends the RIM with the initial
//! granules, locks it at activation, and frees it at destroy. SP4 attestation
//! reads the measurements into the token.
//!
//! Register map (8-byte strided, matches lib/src/measure/measure.dart):
//!   0x00 SLOT_SEL (RW)  0x08 REG_SEL (RW: 0=RIM, 1..=REM)  0x10 EXTEND (W bit0)
//!   0x18 ALLOC (W: realm) 0x20 LOCK (W bit0) 0x28 FREE (W bit0)
//!   0x30 STATUS (R: bit0 busy, bit1 valid, bit2 locked, bit3 reject)
//!   0x38 REALM (R)  0x40+ DATA (RW, 8 words @ +8)  0x80+ MEAS (R, 8 words @ +8)

const Mmio = @import("../mmio.zig");

pub const Measure = struct {
    mmio: Mmio,

    const SLOT_SEL = 0x00;
    const REG_SEL = 0x08;
    const EXTEND = 0x10;
    const ALLOC = 0x18;
    const LOCK = 0x20;
    const FREE = 0x28;
    const STATUS = 0x30;
    const REALM = 0x38;
    const DATA_BASE = 0x40;
    const MEAS_BASE = 0x80;

    /// Register index of the RIM (the initial measurement).
    pub const reg_rim: u32 = 0;

    fn selectSlot(self: Measure, slot: u32) void {
        self.mmio.write(u32, SLOT_SEL, slot);
    }
    fn selectReg(self: Measure, reg: u32) void {
        self.mmio.write(u32, REG_SEL, reg);
    }

    /// Allocate the slot to a realm (clears + unlocks its registers).
    pub fn allocSlot(self: Measure, slot: u32, realm: u8) void {
        self.selectSlot(slot);
        self.mmio.write(u32, ALLOC, realm);
    }

    /// Lock the slot's RIM (at activation).
    pub fn lockSlot(self: Measure, slot: u32) void {
        self.selectSlot(slot);
        self.mmio.write(u32, LOCK, 1);
    }

    /// Free the slot (at destroy).
    pub fn freeSlot(self: Measure, slot: u32) void {
        self.selectSlot(slot);
        self.mmio.write(u32, FREE, 1);
    }

    /// Extend a register with a 32-bit value (placed in the low word of the
    /// 256-bit fold input), then wait for the fold to complete.
    pub fn extend(self: Measure, slot: u32, reg: u32, value: u32) void {
        self.selectSlot(slot);
        self.selectReg(reg);
        var i: u32 = 0;
        while (i < 8) : (i += 1) {
            self.mmio.write(u32, DATA_BASE + 8 * i, if (i == 7) value else 0);
        }
        self.mmio.write(u32, EXTEND, 1);
        while (self.mmio.read(u32, STATUS) & 0x1 != 0) {}
    }

    /// Read one 32-bit word (big-endian, word 0 = MSBs) of a measurement.
    pub fn readWord(self: Measure, slot: u32, reg: u32, word: u32) u32 {
        self.selectSlot(slot);
        self.selectReg(reg);
        return self.mmio.read(u32, MEAS_BASE + 8 * word);
    }
};
