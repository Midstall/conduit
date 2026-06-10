//! ACPI discovery backend, over almanac. Two modes, both conforming to the
//! `backend` contract so the Registry/Builder treat them exactly like the
//! device-tree backend:
//!
//!   * `Static(M)`: over already-discovered `almanac.Tables`. Synthesizes a node
//!     per recognized firmware table - SPCR (serial console), MCFG (PCI ECAM),
//!     and MADT interrupt controllers (I/O APIC, GIC distributor/redistributor).
//!     No allocator; usable very early.
//!   * `Full`: over an `almanac.Interpreter`, walking the AML namespace's
//!     `Device`s for `_HID`/`_CID` and lowering `_CRS` into resources. Needs the
//!     interpreter's arena; runtime only, but sees every enumerable device.
//!
//! Both reuse DT-style id strings (e.g. "ns16550a", "pci-host-ecam-generic") so
//! one matcher table covers both backends and the device-tree backend.

const std = @import("std");
const almanac = @import("almanac");
const backend = @import("../backend.zig");
const match = @import("../match.zig");
const resource = @import("../resource.zig");

// ---------------------------------------------------------------------------
// Static: firmware-table discovery (SPCR / MCFG / MADT)
// ---------------------------------------------------------------------------

pub fn Static(comptime M: type) type {
    return struct {
        const Self = @This();
        const Tables = almanac.TablesGeneric(M);

        tables: Tables,
        step: Step = .spcr,
        mcfg: ?almanac.Mcfg = null,
        mcfg_loaded: bool = false,
        mcfg_idx: usize = 0,
        madt: ?almanac.Madt = null,
        madt_loaded: bool = false,
        madt_idx: usize = 0,
        cur_resources: resource.List = .{},

        const Step = enum { spcr, mcfg, madt, done };

        pub fn init(tables: Tables) Self {
            return .{ .tables = tables };
        }

        pub fn any(self: *Self) backend.Backend {
            return backend.fromImpl(self);
        }

        pub fn reset(self: *Self) void {
            self.step = .spcr;
            self.mcfg = null;
            self.mcfg_loaded = false;
            self.mcfg_idx = 0;
            self.madt = null;
            self.madt_loaded = false;
            self.madt_idx = 0;
            self.cur_resources = .{};
        }

        pub fn next(self: *Self) backend.Error!?backend.Node {
            while (true) switch (self.step) {
                .spcr => {
                    self.step = .mcfg;
                    if ((self.tables.findAs(almanac.Spcr) catch return error.BadFormat)) |spcr| {
                        self.cur_resources = .{};
                        const gas = spcr.baseAddress();
                        switch (gas.space()) {
                            .system_memory => try self.cur_resources.append(.{ .mmio = .{ .base = gas.address, .size = 0x1000 } }),
                            .system_io => try self.cur_resources.append(.{ .reg_io = .{ .port = @truncate(gas.address), .size = 8 } }),
                            else => {},
                        }
                        const num: u32 = if (spcr.interruptType() & 0x1 != 0) spcr.irq() else spcr.globalSystemInterrupt();
                        if (num != 0) try self.cur_resources.append(.{ .irq = .{ .number = num } });
                        var ids = match.IdList{};
                        ids.append(spcrId(spcr.interfaceType()));
                        return .{ .ids = ids, .name = "spcr" };
                    }
                },
                .mcfg => {
                    if (!self.mcfg_loaded) {
                        self.mcfg_loaded = true;
                        self.mcfg = self.tables.findAs(almanac.Mcfg) catch return error.BadFormat;
                    }
                    if (self.mcfg) |m| {
                        var it = m.iterator();
                        var i: usize = 0;
                        while (it.next()) |a| : (i += 1) {
                            if (i != self.mcfg_idx) continue;
                            self.mcfg_idx += 1;
                            self.cur_resources = .{};
                            const buses: u64 = @as(u64, a.end_bus - a.start_bus) + 1;
                            try self.cur_resources.append(.{ .mmio = .{ .base = a.base_address, .size = buses * 0x10_0000 } });
                            var ids = match.IdList{};
                            ids.append("pci-host-ecam-generic");
                            return .{ .ids = ids, .name = "pci" };
                        }
                    }
                    self.step = .madt;
                },
                .madt => {
                    if (!self.madt_loaded) {
                        self.madt_loaded = true;
                        self.madt = self.tables.findAs(almanac.Madt) catch return error.BadFormat;
                    }
                    if (self.madt) |m| {
                        var it = m.iterator();
                        var i: usize = 0;
                        while (it.next() catch return error.BadFormat) |e| : (i += 1) {
                            if (i < self.madt_idx) continue;
                            self.madt_idx += 1;
                            if (try self.lowerMadt(e)) |node| return node;
                        }
                    }
                    self.step = .done;
                },
                .done => return null,
            };
        }

        /// Build an intc node from a MADT entry, or null to skip (CPU-interface
        /// and override entries are not standalone MMIO devices).
        fn lowerMadt(self: *Self, e: anytype) backend.Error!?backend.Node {
            var ids = match.IdList{};
            self.cur_resources = .{};
            switch (e) {
                .io_apic => |io| {
                    try self.cur_resources.append(.{ .mmio = .{ .base = io.address, .size = 0x1000 } });
                    ids.append("acpi,ioapic");
                },
                .gicd => |g| {
                    try self.cur_resources.append(.{ .mmio = .{ .base = g.physical_base_address, .size = 0x1000 } });
                    ids.append(if (g.gic_version >= 3) "arm,gic-v3" else "arm,gic-400");
                },
                .gicr => |g| {
                    try self.cur_resources.append(.{ .mmio = .{ .base = g.discovery_range_base_address, .size = g.discovery_range_length } });
                    ids.append("arm,gic-v3-redist");
                },
                else => return null,
            }
            return .{ .ids = ids, .name = "intc" };
        }

        pub fn resources(self: *Self, node: backend.Node, out: *resource.List) backend.Error!void {
            _ = node;
            out.* = self.cur_resources;
        }
    };
}

/// Map an SPCR interface type onto an id string the matcher table understands.
fn spcrId(iface: anytype) match.Id {
    return switch (iface) {
        .full_16550, .full_16550_dbgp => "ns16550a",
        .arm_pl011 => "arm,pl011",
        .arm_sbsa_generic => "arm,sbsa-uart",
        else => "acpi,serial",
    };
}

// ---------------------------------------------------------------------------
// Full: AML namespace device enumeration (_HID / _CID / _CRS)
// ---------------------------------------------------------------------------

pub const Full = struct {
    interp: *almanac.Interpreter,
    devs: ?almanac.interp.Devices = null,
    cur: ?almanac.interp.Device = null,
    id_storage: [2][9]u8 = undefined,
    name_storage: [4]u8 = undefined,

    pub fn init(interp: *almanac.Interpreter) Full {
        return .{ .interp = interp };
    }

    pub fn any(self: *Full) backend.Backend {
        return backend.fromImpl(self);
    }

    pub fn reset(self: *Full) void {
        if (self.devs) |*d| d.deinit();
        self.devs = null;
        self.cur = null;
    }

    pub fn next(self: *Full) backend.Error!?backend.Node {
        if (self.devs == null) self.devs = self.interp.devices(null) catch return error.BadFormat;
        const d = &self.devs.?;
        while (d.next() catch return error.BadFormat) |dev| {
            var ids = match.IdList{};
            if (dev.hid()) |h| ids.append(self.stash(0, h.str()));
            if (dev.cid()) |h| ids.append(self.stash(1, h.str()));
            if (ids.len == 0) continue; // unidentifiable device, skip

            self.cur = dev;
            self.name_storage = dev.segment();
            return .{ .ids = ids, .name = self.name_storage[0..] };
        }
        return null;
    }

    pub fn resources(self: *Full, node: backend.Node, out: *resource.List) backend.Error!void {
        _ = node;
        const dev = self.cur orelse return;
        var walk = dev.crs() catch return; // no _CRS -> no resources
        defer walk.deinit();
        var it = walk.iterator();
        while (it.next() catch return error.BadFormat) |r| try lowerCrs(r, out);
    }

    /// Copy an id string into stable per-node storage (valid until the next
    /// `next`). Matching happens during iteration, so this is sufficient.
    fn stash(self: *Full, slot: usize, s: []const u8) match.Id {
        const n = @min(s.len, self.id_storage[slot].len);
        @memcpy(self.id_storage[slot][0..n], s[0..n]);
        return self.id_storage[slot][0..n];
    }
};

/// Lower one ACPI `_CRS` descriptor into the normalized resource model.
fn lowerCrs(r: almanac.resource.Resource, out: *resource.List) backend.Error!void {
    switch (r) {
        .fixed_memory32 => |m| try out.append(.{ .mmio = .{ .base = m.base, .size = m.length } }),
        .memory32 => |m| try out.append(.{ .mmio = .{ .base = m.min, .size = m.length } }),
        .memory24 => |m| try out.append(.{ .mmio = .{ .base = @as(u64, m.min) << 8, .size = @as(u64, m.length) << 8 } }),
        .fixed_io => |io| try out.append(.{ .reg_io = .{ .port = io.base, .size = io.length } }),
        .io => |io| try out.append(.{ .reg_io = .{ .port = io.min, .size = io.length } }),
        .word_address, .dword_address, .qword_address => |as| try lowerAddressSpace(as, out),
        .irq => |q| {
            var mask = q.mask;
            var n: u5 = 0;
            while (mask != 0) : (n +%= 1) {
                if (mask & 1 != 0) try out.append(.{ .irq = .{ .number = n } });
                mask >>= 1;
            }
        },
        .extended_irq => |xi| for (xi.interrupts) |num| {
            try out.append(.{ .irq = .{
                .number = num,
                .trigger = if (xi.flags & 0x2 != 0) .edge else .level,
                .polarity = if (xi.flags & 0x4 != 0) .low else .high,
            } });
        },
        .generic_register => |g| if (g.address_space == 0) {
            try out.append(.{ .mmio = .{ .base = g.address, .size = g.bit_width / 8 } });
        },
        else => {}, // gpio/serial_bus/vendor/etc. are not a device's own register window
    }
}

fn lowerAddressSpace(as: almanac.resource.AddressSpace, out: *resource.List) backend.Error!void {
    switch (as.resource_type) {
        0 => try out.append(.{ .mmio = .{ .base = as.min, .size = as.length } }), // memory
        1 => try out.append(.{ .reg_io = .{ .port = @truncate(as.min), .size = @truncate(as.length) } }), // io
        else => {}, // bus-number ranges are not a register window
    }
}
