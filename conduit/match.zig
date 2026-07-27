//! Device-class matching. A `Matcher` is a rule that says "a node carrying any
//! of these compatible strings (DT) or HIDs (ACPI) is a device of this class".
//! Matchers are `comptime`-constructible so the same table drives the comptime
//! `Builder` and the runtime `Registry`.

const std = @import("std");

/// The discovery/matching key. The contract type that drives each class is
/// named separately: `.uart` -> `device.Serial`, `.block` -> `device.Block`,
/// `.gpio` -> `device.Gpio`, `.i2c` -> `device.I2c`, `.spi` -> `device.Spi`,
/// `.rtc` -> `device.Rtc`, `.intc` -> `device.Intc`.
pub const Class = enum { uart, block, display, gpio, i2c, spi, rtc, intc, pci, timer, memory, flash, sdram, tpm };

/// A device identifier: a DT `compatible` string or an ACPI `_HID`/`_CID`.
pub const Id = []const u8;

/// Upper bound on identifiers carried by a single node (a `compatible` list is
/// rarely longer; ACPI gives a HID plus a CID).
pub const max_ids = 8;

/// A fixed-capacity, allocator-free list of a node's identifiers.
pub const IdList = struct {
    items: [max_ids]Id = undefined,
    len: usize = 0,

    pub fn append(self: *IdList, id: Id) void {
        if (self.len >= self.items.len) return;
        self.items[self.len] = id;
        self.len += 1;
    }

    pub fn slice(self: *const IdList) []const Id {
        return self.items[0..self.len];
    }
};

pub const Matcher = struct {
    class: Class,
    dt_compatible: []const []const u8 = &.{},
    acpi_hid: []const []const u8 = &.{},
    /// Name of the shipped driver that binds this match, if any.
    driver: ?[]const u8 = null,

    /// True if any of the node's ids is listed by this matcher.
    pub fn matches(self: Matcher, ids: []const Id) bool {
        for (ids) |id| {
            for (self.dt_compatible) |c| if (std.mem.eql(u8, id, c)) return true;
            for (self.acpi_hid) |h| if (std.mem.eql(u8, id, h)) return true;
        }
        return false;
    }
};

/// Find the first matcher in `table` that claims a node with these ids.
pub fn first(table: []const Matcher, ids: []const Id) ?Matcher {
    for (table) |m| if (m.matches(ids)) return m;
    return null;
}

test "matcher matches on any shared id" {
    const m = Matcher{ .class = .uart, .dt_compatible = &.{ "ns16550a", "ns16550" } };
    try std.testing.expect(m.matches(&.{ "fancyuart", "ns16550" }));
    try std.testing.expect(!m.matches(&.{"pl011"}));
}

test "first returns the claiming matcher" {
    const table = [_]Matcher{
        .{ .class = .uart, .dt_compatible = &.{"ns16550a"} },
        .{ .class = .block, .dt_compatible = &.{"virtio,mmio"} },
    };
    try std.testing.expectEqual(Class.block, first(&table, &.{"virtio,mmio"}).?.class);
    try std.testing.expect(first(&table, &.{"nope"}) == null);
}
