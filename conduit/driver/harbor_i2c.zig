//! Harbor I2C master driver, over an `Mmio`. Polled byte transfers. Each
//! register sits in its own 8-byte slot, because the controller sits on a
//! byte-addressed fabric that decodes the low bits of the byte address. 4-byte
//! spacing aliases every register onto its neighbour.
//!   0x00 CTRL, 0x08 STATUS, 0x10 DATA, 0x18 ADDR, 0x20 PRESCALE, 0x28 CMD.
//!
//! The slave address rides the DATA register as the first byte of the frame
//! (address << 1 | read), which is what the controller's shift engine sends, so
//! the ADDR register is only for slave mode.

const Mmio = @import("../mmio.zig");
const I2c = @import("../device/i2c.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .i2c,
    .dt_compatible = &.{ "midstall,harbor-i2c", "harbor,i2c" },
    .driver = "harbor_i2c",
};

pub const Options = struct {
    /// freq / (5 * bus_freq) - 1, per Harbor. 0 leaves the reset default.
    prescale: u32 = 0,
};

pub const HarborI2c = struct {
    mmio: Mmio,

    const CTRL = 0x00;
    const STATUS = 0x08;
    const DATA = 0x10;
    const ADDR = 0x18; // own address, slave mode only
    const PRESCALE = 0x20;
    const CMD = 0x28;

    const CTRL_ENABLE: u32 = 1 << 0;
    const ST_ACK: u32 = 1 << 1;
    const ST_CMD_DONE: u32 = 1 << 5;
    const CMD_START: u32 = 1 << 0;
    const CMD_STOP: u32 = 1 << 1;
    const CMD_WRITE: u32 = 1 << 2;
    const CMD_READ: u32 = 1 << 3;

    // One command finishes within a bounded number of SCL periods. This cap is
    // far above that at any prescale, so a healthy transfer never reaches it. A
    // controller that never reports CMD_DONE (unclocked or wedged) hits the cap
    // and the transfer gives up, rather than hanging the machine on the poll.
    const DONE_POLL_MAX: u32 = 1_000_000;

    fn issue(self: HarborI2c, command: u32) bool {
        self.mmio.write(u32, CMD, command);
        var i: u32 = 0;
        while (i < DONE_POLL_MAX) : (i += 1) {
            if (self.mmio.read(u32, STATUS) & ST_CMD_DONE != 0) {
                self.mmio.write(u32, STATUS, ST_CMD_DONE); // W1C
                return true;
            }
        }
        return false;
    }

    fn acked(self: HarborI2c) bool {
        return self.mmio.read(u32, STATUS) & ST_ACK != 0;
    }

    fn writeBytes(self: HarborI2c, addr: u7, buf: []const u8) bool {
        self.mmio.write(u32, DATA, (@as(u32, addr) << 1) | 0);
        if (!self.issue(CMD_START | CMD_WRITE)) return false;
        if (!self.acked()) {
            _ = self.issue(CMD_STOP);
            return false;
        }
        for (buf) |b| {
            self.mmio.write(u32, DATA, b);
            if (!self.issue(CMD_WRITE)) return false;
            if (!self.acked()) {
                _ = self.issue(CMD_STOP);
                return false;
            }
        }
        return self.issue(CMD_STOP);
    }

    fn readBytes(self: HarborI2c, addr: u7, buf: []u8) bool {
        self.mmio.write(u32, DATA, (@as(u32, addr) << 1) | 1);
        if (!self.issue(CMD_START | CMD_WRITE)) return false;
        if (!self.acked()) {
            _ = self.issue(CMD_STOP);
            return false;
        }
        for (buf) |*b| {
            if (!self.issue(CMD_READ)) return false;
            b.* = @truncate(self.mmio.read(u32, DATA));
        }
        return self.issue(CMD_STOP);
    }

    /// Write `wr` then read `rd` for 7-bit `addr` (each phase a transaction).
    pub fn transfer(self: HarborI2c, addr: u16, wr: []const u8, rd: []u8) bool {
        const a: u7 = @truncate(addr);
        if (wr.len > 0 and !self.writeBytes(a, wr)) return false;
        if (rd.len > 0 and !self.readBytes(a, rd)) return false;
        return true;
    }

    pub fn i2c(self: *HarborI2c) I2c {
        return I2c.from(self);
    }
};

pub fn bind(mmio: Mmio, opts: Options) HarborI2c {
    const dev = HarborI2c{ .mmio = mmio };
    if (opts.prescale != 0) mmio.write(u32, HarborI2c.PRESCALE, opts.prescale);
    mmio.write(u32, HarborI2c.CTRL, HarborI2c.CTRL_ENABLE);
    return dev;
}
