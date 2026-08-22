//! RISC-V CLINT (Core-Local Interruptor) driver, over an `Mmio`. Ported from
//! Weir's arch/riscv/clint.zig. CLINT provides the machine timer and inter-hart
//! software interrupts (IPIs). CLINT does not claim or complete interrupts.
//! The PLIC does that. So this is a standalone driver, not an `Intc`.
//!   MSIP   0x0000 (u32 per hart)
//!   MTIMECMP 0x4000 (u64 per hart)
//!   MTIME  0xbff8 (u64 global)

const Mmio = @import("../mmio.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .timer,
    .dt_compatible = &.{ "riscv,clint0", "sifive,clint0" },
    .driver = "clint",
};

pub const Clint = struct {
    mmio: Mmio,

    const MSIP = 0x0000;
    const MTIMECMP = 0x4000;
    const MTIME = 0xbff8;

    /// The global timer counter.
    pub fn time(self: Clint) u64 {
        return self.mmio.read(u64, MTIME);
    }

    /// Program `hart`'s timer compare. A timer interrupt fires once time() >= value.
    pub fn setTimecmp(self: Clint, hart: usize, value: u64) void {
        self.mmio.write(u64, MTIMECMP + hart * 8, value);
    }

    /// Raise a software interrupt (IPI) on `hart`.
    pub fn sendIpi(self: Clint, hart: usize) void {
        self.mmio.write(u32, MSIP + hart * 4, 1);
    }

    /// Acknowledge (clear) the software interrupt on `hart`.
    pub fn clearIpi(self: Clint, hart: usize) void {
        self.mmio.write(u32, MSIP + hart * 4, 0);
    }
};

pub fn bind(mmio: Mmio) Clint {
    return .{ .mmio = mmio };
}
