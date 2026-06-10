//! Harbor SPI master driver, over an `Mmio`. Ported from Weir's drivers/spi.zig
//! (Harbor harbor_spi.c register map). Full-duplex, one byte per transfer, polled.
//!   0x00 CTRL, 0x04 STATUS, 0x08 DATA, 0x0C DIVIDER, 0x10 CS.

const Mmio = @import("../mmio.zig");
const Spi = @import("../device/spi.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .spi,
    .dt_compatible = &.{ "midstall,harbor-spi", "harbor,spi" },
    .driver = "harbor_spi",
};

pub const Options = struct {
    divider: u32 = 0,
    /// SPI mode 0..3 (CPOL/CPHA).
    mode: u2 = 0,
};

pub const HarborSpi = struct {
    mmio: Mmio,

    const CTRL = 0x00;
    const STATUS = 0x04;
    const DATA = 0x08;
    const DIVIDER = 0x0c;
    const CS = 0x10;

    const CTRL_ENABLE: u32 = 1 << 0;
    const CTRL_CPOL: u32 = 1 << 1;
    const CTRL_CPHA: u32 = 1 << 2;
    const ST_BUSY: u32 = 1 << 0;

    /// Assert (true) or release (false) chip select.
    pub fn chipSelect(self: HarborSpi, active: bool) void {
        self.mmio.write(u32, CS, if (active) 1 else 0);
    }

    fn waitIdle(self: HarborSpi) void {
        while (self.mmio.read(u32, STATUS) & ST_BUSY != 0) {}
    }

    pub fn transferByte(self: HarborSpi, out: u8) u8 {
        self.mmio.write(u32, DATA, out);
        self.waitIdle();
        return @truncate(self.mmio.read(u32, DATA));
    }

    /// Full-duplex transfer. The exchange runs for max(tx, rx) bytes; `tx` is
    /// padded with zeros past its end, `rx` is filled only within its bounds.
    /// Chip select is asserted around the exchange.
    pub fn transfer(self: HarborSpi, tx: []const u8, rx: []u8) void {
        const len = @max(tx.len, rx.len);
        self.chipSelect(true);
        var i: usize = 0;
        while (i < len) : (i += 1) {
            const out: u8 = if (i < tx.len) tx[i] else 0;
            const in = self.transferByte(out);
            if (i < rx.len) rx[i] = in;
        }
        self.chipSelect(false);
    }

    pub fn spi(self: *HarborSpi) Spi {
        return Spi.from(self);
    }
};

pub fn bind(mmio: Mmio, opts: Options) HarborSpi {
    const dev = HarborSpi{ .mmio = mmio };
    if (opts.divider != 0) mmio.write(u32, HarborSpi.DIVIDER, opts.divider);
    var ctrl: u32 = HarborSpi.CTRL_ENABLE;
    if (opts.mode & 0b01 != 0) ctrl |= HarborSpi.CTRL_CPHA;
    if (opts.mode & 0b10 != 0) ctrl |= HarborSpi.CTRL_CPOL;
    mmio.write(u32, HarborSpi.CTRL, ctrl);
    return dev;
}
