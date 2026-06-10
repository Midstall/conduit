//! ARM PL031 RTC driver, over an `Mmio`, implementing the `Rtc` contract. The
//! RTC on QEMU's aarch64 `virt` machine. A 32-bit seconds-since-epoch counter.
//!   0x00 DR (data, RO), 0x08 LR (load, WO), 0x0C CR (control).

const Mmio = @import("../mmio.zig");
const Rtc = @import("../device/rtc.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .rtc,
    .dt_compatible = &.{"arm,pl031"},
    .driver = "pl031",
};

pub const Pl031 = struct {
    mmio: Mmio,

    const DR = 0x00;
    const LR = 0x08;
    const CR = 0x0c;

    pub fn now(self: Pl031) Rtc.DateTime {
        return Rtc.fromUnix(self.mmio.read(u32, DR));
    }

    pub fn set(self: Pl031, dt: Rtc.DateTime) void {
        self.mmio.write(u32, LR, @intCast(Rtc.toUnix(dt)));
    }

    pub fn rtc(self: *Pl031) Rtc {
        return Rtc.from(self);
    }
};

pub fn bind(mmio: Mmio) Pl031 {
    mmio.write(u32, Pl031.CR, 1); // enable
    return .{ .mmio = mmio };
}
