//! ARM GICv2 driver, implementing the `Intc` contract over its two MMIO
//! windows: the Distributor (GICD) and the CPU Interface (GICC). Used on QEMU's
//! aarch64 `virt` machine with `-machine gic-version=2`.

const Mmio = @import("../mmio.zig");
const Intc = @import("../device/intc.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .intc,
    .dt_compatible = &.{ "arm,gic-400", "arm,cortex-a15-gic", "arm,gic-v2" },
    .driver = "gicv2",
};

pub const Gicv2 = struct {
    dist: Mmio, // GICD
    cpu: Mmio, // GICC

    // Distributor registers.
    const GICD_CTLR = 0x000;
    const GICD_ISENABLER = 0x100;
    const GICD_ICENABLER = 0x180;
    const GICD_IPRIORITYR = 0x400;
    const GICD_ITARGETSR = 0x800;

    // CPU-interface registers.
    const GICC_CTLR = 0x00;
    const GICC_PMR = 0x04;
    const GICC_IAR = 0x0c;
    const GICC_EOIR = 0x10;

    const SPURIOUS: u32 = 1023;

    pub fn init(self: Gicv2) void {
        self.dist.write(u32, GICD_CTLR, 1); // enable distributor
        self.cpu.write(u32, GICC_PMR, 0xff); // allow all priorities
        self.cpu.write(u32, GICC_CTLR, 1); // enable CPU interface
    }

    pub fn enable(self: Gicv2, irq: u32) void {
        // Lowest priority, route to CPU 0 (byte-addressed priority/target regs).
        self.dist.write(u8, GICD_IPRIORITYR + irq, 0xa0);
        self.dist.write(u8, GICD_ITARGETSR + irq, 0x01);
        self.dist.write(u32, GICD_ISENABLER + (irq / 32) * 4, @as(u32, 1) << @truncate(irq % 32));
    }

    pub fn disable(self: Gicv2, irq: u32) void {
        self.dist.write(u32, GICD_ICENABLER + (irq / 32) * 4, @as(u32, 1) << @truncate(irq % 32));
    }

    pub fn claim(self: Gicv2) ?u32 {
        const iar = self.cpu.read(u32, GICC_IAR) & 0x3ff;
        return if (iar == SPURIOUS) null else iar;
    }

    pub fn complete(self: Gicv2, irq: u32) void {
        self.cpu.write(u32, GICC_EOIR, irq);
    }

    pub fn intc(self: *Gicv2) Intc {
        return Intc.from(self);
    }
};

/// Bind a GICv2 over its distributor and CPU-interface windows (resource 0 and
/// 1 of the matched device) and initialize it.
pub fn bind(dist: Mmio, cpu: Mmio) Gicv2 {
    const g = Gicv2{ .dist = dist, .cpu = cpu };
    g.init();
    return g;
}
