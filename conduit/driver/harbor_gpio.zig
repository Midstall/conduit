//! Harbor GPIO controller driver, over an `Mmio`. Ported from Weir's
//! drivers/gpio.zig (Harbor HDL / harbor_gpio.c register map):
//!   0x00 INPUT (RO), 0x04 OUTPUT (RW), 0x08 DIR (RW, 1 = output),
//!   0x0C IRQ_EN, 0x10 IRQ_STATUS (W1C), 0x14 IRQ_EDGE.

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
    const OUTPUT = 0x04;
    const DIR = 0x08;

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

    pub fn gpio(self: *HarborGpio) Gpio {
        return Gpio.from(self);
    }
};

pub fn bind(mmio: Mmio) HarborGpio {
    return .{ .mmio = mmio };
}
