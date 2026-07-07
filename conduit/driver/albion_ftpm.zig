//! Albion fTPM-port driver, over an `Mmio`.
//!
//! The SEP-facing side of the TPM TIS seam (the hardware block `AlbionFtpmPort`,
//! default window 0x4000_B000). The SEP firmware, woken by the TIS doorbell
//! (MSIP), drains the AP's TPM2 command and writes the response through these
//! registers. All accesses are 32-bit words (the SoC fabric is 64-bit, but the
//! registers live on the low lanes, so word accesses avoid the byte-lane hazard).
//!   STATUS      0x00 (RO): bit0 = command pending, bits[31:16] = command length
//!   CMD_ADDR    0x04 (RW): command byte index to read
//!   CMD_DATA    0x08 (RO): command byte at CMD_ADDR
//!   RESP_ADDR   0x0C (RW): response byte index to write
//!   RESP_DATA   0x10 (W):  response byte (writing pulses the FIFO write)
//!   RESP_LEN    0x14 (RW): response length in bytes
//!   RESP_COMMIT 0x18 (W):  bit0 marks the response complete

const Mmio = @import("../mmio.zig");

pub const FtpmPort = struct {
    mmio: Mmio,

    const STATUS = 0x00;
    const CMD_ADDR = 0x04;
    const CMD_DATA = 0x08;
    const RESP_ADDR = 0x0c;
    const RESP_DATA = 0x10;
    const RESP_LEN = 0x14;
    const RESP_COMMIT = 0x18;

    /// True when a complete command from the AP is waiting to be drained.
    pub fn commandPending(self: FtpmPort) bool {
        return (self.mmio.read(u32, STATUS) & 0x1) != 0;
    }

    /// Length in bytes of the pending command.
    pub fn commandLen(self: FtpmPort) u16 {
        return @truncate(self.mmio.read(u32, STATUS) >> 16);
    }

    /// Read command byte at index `i`.
    pub fn readCommandByte(self: FtpmPort, i: u16) u8 {
        self.mmio.write(u32, CMD_ADDR, i);
        return @truncate(self.mmio.read(u32, CMD_DATA));
    }

    /// Write response byte `b` at index `i`.
    pub fn writeResponseByte(self: FtpmPort, i: u16, b: u8) void {
        self.mmio.write(u32, RESP_ADDR, i);
        self.mmio.write(u32, RESP_DATA, b);
    }

    /// Publish the response: set its length and signal completion to the TIS.
    pub fn commitResponse(self: FtpmPort, len: u16) void {
        self.mmio.write(u32, RESP_LEN, len);
        self.mmio.write(u32, RESP_COMMIT, 1);
    }
};

pub fn bind(mmio: Mmio) FtpmPort {
    return .{ .mmio = mmio };
}
