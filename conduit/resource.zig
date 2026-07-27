//! The normalized resource model. Both backends (device tree, ACPI) lower their
//! native descriptions into this union, so drivers and consumers never see a DT
//! cell or an ACPI `_CRS` descriptor, only a `Resource`.

const config = @import("config.zig");

pub const Trigger = enum { edge, level, unknown };
pub const Polarity = enum { high, low, unknown };

/// A cross-reference to another device (a clock provider, a GPIO controller).
/// Stored but not auto-resolved in the first cut.
pub const Ref = union(enum) {
    none,
    phandle: u32,
    name: []const u8,
};

pub const Resource = union(enum) {
    mmio: MmioRegion,
    irq: Irq,
    dma: Dma,
    clock: Clock,
    gpio: Gpio,
    reg_io: PortIo,

    /// An MMIO window, already translated from bus to CPU address space (via DT
    /// `ranges` or ACPI `_CRS`).
    pub const MmioRegion = struct { base: u64, size: u64 };
    pub const Irq = struct { number: u32, trigger: Trigger = .unknown, polarity: Polarity = .unknown };
    pub const Dma = struct { base: u64, size: u64, coherent: bool = false };
    pub const Clock = struct { id: u32 = 0, freq_hz: ?u64 = null };
    pub const Gpio = struct { controller: Ref = .none, pin: u32, flags: u32 = 0 };
    /// x86 port-IO / ACPI SystemIO.
    pub const PortIo = struct { port: u16, size: u16 };
};

/// A fixed-capacity, allocator-free list of resources attached to one device
/// node. Capacity is `config.resource_cap` (build option `-Dresource-cap`).
pub const List = struct {
    items: [config.resource_cap]Resource = undefined,
    len: usize = 0,

    pub fn append(self: *List, r: Resource) error{TooManyResources}!void {
        if (self.len >= self.items.len) return error.TooManyResources;
        self.items[self.len] = r;
        self.len += 1;
    }

    pub fn slice(self: *const List) []const Resource {
        return self.items[0..self.len];
    }

    /// First MMIO window, if any.
    pub fn mmio(self: *const List) ?Resource.MmioRegion {
        return self.mmioAt(0);
    }

    /// First Clock resource, if any (e.g. a device's own clock-frequency).
    pub fn clock(self: *const List) ?Resource.Clock {
        for (self.slice()) |r| switch (r) {
            .clock => |c| return c,
            else => {},
        };
        return null;
    }

    /// Nth MMIO window (0-based), if present. Devices like a GICv2 expose more
    /// than one (distributor + CPU interface).
    pub fn mmioAt(self: *const List, n: usize) ?Resource.MmioRegion {
        var i: usize = 0;
        for (self.slice()) |r| switch (r) {
            .mmio => |m| {
                if (i == n) return m;
                i += 1;
            },
            else => {},
        };
        return null;
    }

    /// Nth IRQ (0-based), if present.
    pub fn irq(self: *const List, n: usize) ?Resource.Irq {
        var i: usize = 0;
        for (self.slice()) |r| switch (r) {
            .irq => |q| {
                if (i == n) return q;
                i += 1;
            },
            else => {},
        };
        return null;
    }
};
