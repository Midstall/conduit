//! RISC-V PLIC (Platform-Level Interrupt Controller) driver, over an `Mmio`.
//! Implements the `Intc` contract for one hart context.
//!
//! Register map (per the SiFive PLIC spec):
//!   0x000000 + 4*irq          source priority
//!   0x002000 + 0x80*ctx       enable bits (1 bit/source)
//!   0x200000 + 0x1000*ctx     priority threshold
//!   0x200004 + 0x1000*ctx     claim/complete

const Mmio = @import("../mmio.zig");
const Intc = @import("../device/intc.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .intc,
    .dt_compatible = &.{ "riscv,plic0", "sifive,plic-1.0.0" },
    .driver = "plic",
};

pub const Options = struct {
    /// Hart context this instance drives (M/S context index).
    context: u32 = 0,
};

pub const Plic = struct {
    mmio: Mmio,
    context: u32,

    fn priorityOff(irq: u32) usize {
        return @as(usize, irq) * 4;
    }
    fn enableOff(self: Plic, irq: u32) usize {
        return 0x2000 + @as(usize, self.context) * 0x80 + (@as(usize, irq) / 32) * 4;
    }
    fn claimOff(self: Plic) usize {
        return 0x200004 + @as(usize, self.context) * 0x1000;
    }
    fn thresholdOff(self: Plic) usize {
        return 0x200000 + @as(usize, self.context) * 0x1000;
    }

    pub fn enable(self: Plic, irq: u32) void {
        self.mmio.write(u32, priorityOff(irq), 1); // lowest non-zero priority
        const off = self.enableOff(irq);
        const bit = @as(u32, 1) << @truncate(irq % 32);
        self.mmio.write(u32, off, self.mmio.read(u32, off) | bit);
    }

    pub fn disable(self: Plic, irq: u32) void {
        const off = self.enableOff(irq);
        const bit = @as(u32, 1) << @truncate(irq % 32);
        self.mmio.write(u32, off, self.mmio.read(u32, off) & ~bit);
    }

    pub fn claim(self: Plic) ?u32 {
        const irq = self.mmio.read(u32, self.claimOff());
        return if (irq == 0) null else irq;
    }

    pub fn complete(self: Plic, irq: u32) void {
        self.mmio.write(u32, self.claimOff(), irq);
    }

    /// Set the context's priority threshold. The PLIC masks interrupts at or below it.
    pub fn setThreshold(self: Plic, threshold: u32) void {
        self.mmio.write(u32, self.thresholdOff(), threshold);
    }

    pub fn intc(self: *Plic) Intc {
        return Intc.from(self);
    }
};

pub fn bind(mmio: Mmio, opts: Options) Plic {
    const p = Plic{ .mmio = mmio, .context = opts.context };
    p.setThreshold(0); // unmask all priorities above 0
    return p;
}
