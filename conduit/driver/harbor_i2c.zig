//! Harbor I2C master driver, over an `Mmio`. Ported from Weir's drivers/i2c.zig
//! (Harbor harbor_i2c.c register map). Polled byte transfers.
//!   0x00 CTRL, 0x04 STATUS, 0x08 DATA, 0x10 PRESCALE, 0x14 CMD.

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
    const STATUS = 0x04;
    const DATA = 0x08;
    const PRESCALE = 0x10;
    const CMD = 0x14;

    const CTRL_ENABLE: u32 = 1 << 0;
    const ST_ACK: u32 = 1 << 1;
    const ST_CMD_DONE: u32 = 1 << 5;
    const CMD_START: u32 = 1 << 0;
    const CMD_STOP: u32 = 1 << 1;
    const CMD_WRITE: u32 = 1 << 2;
    const CMD_READ: u32 = 1 << 3;

    fn issue(self: HarborI2c, command: u32) void {
        self.mmio.write(u32, CMD, command);
        while (self.mmio.read(u32, STATUS) & ST_CMD_DONE == 0) {}
        self.mmio.write(u32, STATUS, ST_CMD_DONE); // W1C
    }

    fn acked(self: HarborI2c) bool {
        return self.mmio.read(u32, STATUS) & ST_ACK != 0;
    }

    fn writeBytes(self: HarborI2c, addr: u7, buf: []const u8) bool {
        self.mmio.write(u32, DATA, (@as(u32, addr) << 1) | 0);
        self.issue(CMD_START | CMD_WRITE);
        if (!self.acked()) {
            self.issue(CMD_STOP);
            return false;
        }
        for (buf) |b| {
            self.mmio.write(u32, DATA, b);
            self.issue(CMD_WRITE);
            if (!self.acked()) {
                self.issue(CMD_STOP);
                return false;
            }
        }
        self.issue(CMD_STOP);
        return true;
    }

    fn readBytes(self: HarborI2c, addr: u7, buf: []u8) bool {
        self.mmio.write(u32, DATA, (@as(u32, addr) << 1) | 1);
        self.issue(CMD_START | CMD_WRITE);
        if (!self.acked()) {
            self.issue(CMD_STOP);
            return false;
        }
        for (buf) |*b| {
            self.issue(CMD_READ);
            b.* = @truncate(self.mmio.read(u32, DATA));
        }
        self.issue(CMD_STOP);
        return true;
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
