//! Harbor GPIO controller driver, over an `Mmio`. Each register sits in its own
//! 8-byte slot, because the controller sits on a byte-addressed fabric that
//! decodes the low bits of the byte address. 4-byte spacing aliases every
//! register onto its neighbour.
//!   0x00 INPUT (RO), 0x08 OUTPUT (RW), 0x10 DIR (RW, 1 = output),
//!   0x18 IRQ_EN, 0x20 IRQ_STATUS (W1C), 0x28 IRQ_EDGE.

const Mmio = @import("../mmio.zig");
const Gpio = @import("../device/gpio.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .gpio,
    .dt_compatible = &.{ "midstall,harbor-gpio", "harbor,gpio" },
    .driver = "harbor_gpio",
};

pub const HarborGpio = struct {
    mmio: Mmio,

    const INPUT = 0x00;
    const OUTPUT = 0x08;
    const DIR = 0x10;
    const IRQ_EN = 0x18;
    const IRQ_STATUS = 0x20; // write-1-to-clear
    const IRQ_EDGE = 0x28; // 0 = level, 1 = rising edge

    pub fn direction(self: HarborGpio, pin: u32, dir: Gpio.Dir) void {
        const bit = @as(u32, 1) << @truncate(pin);
        const cur = self.mmio.read(u32, DIR);
        self.mmio.write(u32, DIR, switch (dir) {
            .output => cur | bit,
            .input => cur & ~bit,
        });
    }

    pub fn set(self: HarborGpio, pin: u32, value: bool) void {
        const bit = @as(u32, 1) << @truncate(pin);
        const cur = self.mmio.read(u32, OUTPUT);
        self.mmio.write(u32, OUTPUT, if (value) cur | bit else cur & ~bit);
    }

    pub fn get(self: HarborGpio, pin: u32) bool {
        return self.mmio.read(u32, INPUT) & (@as(u32, 1) << @truncate(pin)) != 0;
    }

    /// Enable (true) or mask (false) the interrupt for one pin. `edge` picks
    /// rising-edge triggering over level.
    pub fn interruptEnable(self: HarborGpio, pin: u32, enabled: bool, edge: bool) void {
        const bit = @as(u32, 1) << @truncate(pin);
        const mode = self.mmio.read(u32, IRQ_EDGE);
        self.mmio.write(u32, IRQ_EDGE, if (edge) mode | bit else mode & ~bit);
        const en = self.mmio.read(u32, IRQ_EN);
        self.mmio.write(u32, IRQ_EN, if (enabled) en | bit else en & ~bit);
    }

    /// Pins with a pending interrupt.
    pub fn interruptStatus(self: HarborGpio) u32 {
        return self.mmio.read(u32, IRQ_STATUS);
    }

    /// Acknowledge every pin in `mask`.
    pub fn interruptClear(self: HarborGpio, mask: u32) void {
        self.mmio.write(u32, IRQ_STATUS, mask);
    }

    pub fn gpio(self: *HarborGpio) Gpio {
        return Gpio.from(self);
    }
};

pub fn bind(mmio: Mmio) HarborGpio {
    return .{ .mmio = mmio };
}
