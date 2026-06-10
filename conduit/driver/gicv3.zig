//! ARM GICv3 driver, implementing the `Intc` contract. The Distributor (GICD)
//! and per-CPU Redistributor (GICR) are MMIO; the CPU interface (claim/EOI,
//! priority mask, group enable) is the `ICC_*` system-register file, reached
//! with MRS/MSR on aarch64. Those accesses are comptime-gated on the target
//! arch so conduit still builds and unit-tests on any host (the MMIO paths work
//! everywhere; claim/complete are inert off aarch64).

const builtin = @import("builtin");
const Mmio = @import("../mmio.zig");
const Intc = @import("../device/intc.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .intc,
    .dt_compatible = &.{ "arm,gic-v3", "arm,gic-v3-its" },
    .driver = "gicv3",
};

pub const Gicv3 = struct {
    dist: Mmio, // GICD
    redist: Mmio, // GICR (RD_base; the SGI frame is 0x10000 above it)

    const GICD_CTLR = 0x000;
    const GICD_ISENABLER = 0x100;
    const GICD_ICENABLER = 0x180;
    const GICD_IPRIORITYR = 0x400;

    const GICR_WAKER = 0x0014;
    const GICR_SGI_BASE = 0x10000;
    const GICR_ISENABLER0 = GICR_SGI_BASE + 0x100;
    const GICR_ICENABLER0 = GICR_SGI_BASE + 0x180;
    const GICR_IPRIORITYR = GICR_SGI_BASE + 0x400;

    const SPURIOUS: u64 = 1023;

    pub fn init(self: Gicv3) void {
        // Wake the redistributor: clear ProcessorSleep, wait for ChildrenAsleep.
        self.redist.write(u32, GICR_WAKER, self.redist.read(u32, GICR_WAKER) & ~@as(u32, 1 << 1));
        while (self.redist.read(u32, GICR_WAKER) & (1 << 2) != 0) {}

        // Affinity Routing + Group1 enable on the distributor.
        self.dist.write(u32, GICD_CTLR, (1 << 4) | (1 << 1) | (1 << 0));

        // CPU interface (system registers): enable SRE, unmask priorities, enable Grp1.
        writeSys("ICC_SRE_EL1", readSys("ICC_SRE_EL1") | 1);
        writeSys("ICC_PMR_EL1", 0xff);
        writeSys("ICC_IGRPEN1_EL1", 1);
    }

    pub fn enable(self: Gicv3, irq: u32) void {
        if (irq < 32) {
            self.redist.write(u8, GICR_IPRIORITYR + irq, 0xa0);
            self.redist.write(u32, GICR_ISENABLER0, @as(u32, 1) << @truncate(irq));
        } else {
            self.dist.write(u8, GICD_IPRIORITYR + irq, 0xa0);
            self.dist.write(u32, GICD_ISENABLER + (irq / 32) * 4, @as(u32, 1) << @truncate(irq % 32));
        }
    }

    pub fn disable(self: Gicv3, irq: u32) void {
        if (irq < 32) {
            self.redist.write(u32, GICR_ICENABLER0, @as(u32, 1) << @truncate(irq));
        } else {
            self.dist.write(u32, GICD_ICENABLER + (irq / 32) * 4, @as(u32, 1) << @truncate(irq % 32));
        }
    }

    pub fn claim(self: Gicv3) ?u32 {
        _ = self;
        const iar = readSys("ICC_IAR1_EL1") & 0xffffff;
        return if (iar == SPURIOUS) null else @truncate(iar);
    }

    pub fn complete(self: Gicv3, irq: u32) void {
        _ = self;
        writeSys("ICC_EOIR1_EL1", irq);
    }

    pub fn intc(self: *Gicv3) Intc {
        return Intc.from(self);
    }
};

fn readSys(comptime name: []const u8) u64 {
    return switch (builtin.cpu.arch) {
        .aarch64 => asm volatile ("mrs %[v], " ++ name
            : [v] "=r" (-> u64),
        ),
        else => 0,
    };
}

fn writeSys(comptime name: []const u8, v: u64) void {
    switch (builtin.cpu.arch) {
        .aarch64 => asm volatile ("msr " ++ name ++ ", %[v]"
            :
            : [v] "r" (v),
        ),
        else => {},
    }
}

pub fn bind(dist: Mmio, redist: Mmio) Gicv3 {
    const g = Gicv3{ .dist = dist, .redist = redist };
    g.init();
    return g;
}
