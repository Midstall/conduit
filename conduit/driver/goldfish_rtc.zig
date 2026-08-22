//! Goldfish RTC driver, over an `Mmio`, implementing the `Rtc` contract. This is
//! the RTC on QEMU's `virt` machines (notably riscv64), exposing a 64-bit
//! nanoseconds-since-epoch counter across two 32-bit registers.
//!   0x00 TIME_LOW (reading latches TIME_HIGH), 0x04 TIME_HIGH.

const Mmio = @import("../mmio.zig");
const Rtc = @import("../device/rtc.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .rtc,
    .dt_compatible = &.{"google,goldfish-rtc"},
    .driver = "goldfish_rtc",
};

pub const Goldfish = struct {
    mmio: Mmio,

    const TIME_LOW = 0x00;
    const TIME_HIGH = 0x04;
    const NS_PER_S = 1_000_000_000;

    pub fn now(self: Goldfish) Rtc.DateTime {
        const low = self.mmio.read(u32, TIME_LOW); // read LOW first, it latches HIGH
        const high = self.mmio.read(u32, TIME_HIGH);
        const ns = (@as(u64, high) << 32) | low;
        return Rtc.fromUnix(@intCast(ns / NS_PER_S));
    }

    pub fn set(self: Goldfish, dt: Rtc.DateTime) void {
        const ns = @as(u64, @intCast(Rtc.toUnix(dt))) * NS_PER_S;
        self.mmio.write(u32, TIME_HIGH, @truncate(ns >> 32));
        self.mmio.write(u32, TIME_LOW, @truncate(ns));
    }

    pub fn rtc(self: *Goldfish) Rtc {
        return Rtc.from(self);
    }
};

pub fn bind(mmio: Mmio) Goldfish {
    return .{ .mmio = mmio };
}
