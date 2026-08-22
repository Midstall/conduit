//! Device-tree discovery backend, over a `dtree.Reader`.
//!
//! Walks the flattened tree yielding one node per device, lowering `reg` into
//! MMIO resources (interpreted with the parent's `#address-cells`/`#size-cells`)
//! and `interrupts`/`interrupts-extended` into IRQ resources. Works at comptime
//! (over an `@embedFile`'d DTB via `Reader.initBuffer`) and at runtime.
//!
//! Interrupt decoding is controller-aware. A one-time pre-pass builds a map from
//! interrupt-controller phandle to its `#interrupt-cells` and specifier format
//! (ARM GIC `<type irq flags>` vs generic `<irq ...>`). conduit then decodes a
//! device's interrupts against its effective `interrupt-parent` (declared on the
//! node or inherited from an ancestor). For `interrupts-extended`, it decodes
//! against the controller that each entry names.
//!
//! Properties of a node precede its subnodes in the DTB stream, and dtree
//! reports a node's own props at one depth below the node's begin. We exploit
//! that: on entering a begin token we eagerly consume the node's property
//! tokens, save/restore the iterator to "peek" the first non-prop token.

const std = @import("std");
const dtree = @import("dtree");
const backend = @import("../backend.zig");
const match = @import("../match.zig");
const resource = @import("../resource.zig");

const max_depth = 32;
const max_controllers = 64;

/// The specifier format an interrupt controller uses.
const Domain = enum { generic, arm_gic };

const Controller = struct { phandle: u32, cells: u32, domain: Domain };

/// Values inherited down the tree. `addr`/`size` are the cells a node declares
/// for its children's `reg`. `intr_parent` is the effective interrupt-parent.
/// `ranges`/`parent_addr` describe how this node (a bus) translates its child
/// addresses up into its parent's address space.
const Inherited = struct {
    addr: u32 = 2,
    size: u32 = 1,
    intr_parent: ?u32 = null,
    /// This node's `ranges` bytes (null = absent, empty slice = identity 1:1).
    ranges: ?[]const u8 = null,
    /// `#address-cells` of this node's parent (to size ranges' parent-address).
    parent_addr: u32 = 2,
};

pub const DtBackend = struct {
    reader: *const dtree.Reader,
    iter: dtree.Reader.NodeIterator,
    /// One entry per open node: what that node passes to its children.
    stack: [max_depth]Inherited = undefined,
    depth: usize = 0,
    /// Inherited values (from the parent) for the node most recently returned.
    cur: Inherited = .{},
    /// Resources of the node most recently returned, lowered eagerly. `resources`
    /// hands this back. (The contract guarantees `resources` is called for a node
    /// before the following `next`.)
    cur_resources: resource.List = .{},
    /// Interrupt-controller map, built lazily on first use.
    controllers: [max_controllers]Controller = undefined,
    nctrl: usize = 0,
    map_built: bool = false,

    pub fn init(reader: *const dtree.Reader) DtBackend {
        return .{ .reader = reader, .iter = reader.nodeIterator() };
    }

    pub fn any(self: *DtBackend) backend.Backend {
        return backend.fromImpl(self);
    }

    pub fn reset(self: *DtBackend) void {
        self.iter = self.reader.nodeIterator();
        self.depth = 0;
        self.cur = .{};
        self.cur_resources = .{};
    }

    pub fn next(self: *DtBackend) backend.Error!?backend.Node {
        if (!self.map_built) {
            self.buildControllerMap();
            self.map_built = true;
        }
        while (true) {
            const n = (self.iter.next() catch return error.BadFormat) orelse return null;
            switch (n) {
                .end => {
                    if (self.depth > 0) self.depth -= 1;
                    continue;
                },
                .prop => continue, // a stray token: the begin arm consumes a node's props
                .begin => |b| {
                    self.cur = if (self.depth > 0) self.stack[self.depth - 1] else Inherited{};

                    var ids = match.IdList{};
                    var child = Inherited{ .intr_parent = self.cur.intr_parent, .parent_addr = self.cur.addr };
                    self.cur_resources = .{};
                    try self.scanProps(&ids, &child, &self.cur_resources);

                    if (self.depth < max_depth) {
                        self.stack[self.depth] = child;
                        self.depth += 1;
                    }

                    return backend.Node{ .ids = ids, .name = b.name };
                },
            }
        }
    }

    pub fn resources(self: *DtBackend, node: backend.Node, out: *resource.List) backend.Error!void {
        _ = node;
        out.* = self.cur_resources;
    }

    /// Scan the current node's properties: collect ids, the cells it declares for
    /// children, its effective interrupt-parent, and lower reg/interrupts into
    /// `out`. Interrupt props are captured and lowered after the loop, since the
    /// node's `interrupt-parent` may appear after its `interrupts`.
    fn scanProps(self: *DtBackend, ids: *match.IdList, child: *Inherited, out: *resource.List) backend.Error!void {
        var own_parent: ?u32 = null;
        var raw_interrupts: ?[]const u8 = null;
        var raw_extended: ?[]const u8 = null;

        while (true) {
            const saved = self.iter;
            const n = (self.iter.next() catch return error.BadFormat) orelse break;
            if (n != .prop) {
                self.iter = saved;
                break;
            }
            const p = n.prop;
            if (std.mem.eql(u8, p.name, "compatible")) {
                splitCompatible(p.value, ids);
            } else if (std.mem.eql(u8, p.name, "device_type")) {
                // Nodes like `/memory` carry no `compatible`. Expose their
                // `device_type` string (e.g. "memory") as an id. A Matcher then
                // claims them by class.
                var dt = p.value;
                if (dt.len > 0 and dt[dt.len - 1] == 0) dt = dt[0 .. dt.len - 1];
                if (dt.len > 0) ids.append(dt);
            } else if (std.mem.eql(u8, p.name, "clock-frequency")) {
                // A device's own clock rate (e.g. the UART baud clock). Lower it
                // into a Clock resource so a consumer reads it off the match.
                if (beCell(p.value)) |hz| try out.append(.{ .clock = .{ .freq_hz = hz } });
            } else if (std.mem.eql(u8, p.name, "#address-cells")) {
                child.addr = beCell(p.value) orelse child.addr;
            } else if (std.mem.eql(u8, p.name, "#size-cells")) {
                child.size = beCell(p.value) orelse child.size;
            } else if (std.mem.eql(u8, p.name, "reg")) {
                try self.lowerReg(p.value, out);
            } else if (std.mem.eql(u8, p.name, "ranges")) {
                child.ranges = p.value; // empty slice = identity
            } else if (std.mem.eql(u8, p.name, "interrupt-parent")) {
                own_parent = beCell(p.value);
            } else if (std.mem.eql(u8, p.name, "interrupts")) {
                raw_interrupts = p.value;
            } else if (std.mem.eql(u8, p.name, "interrupts-extended")) {
                raw_extended = p.value;
            } else if (p.value.len == 0) {
                // An empty-valued property is a DT boolean (e.g. `harbor,dma;`).
                // Lower it as a named capability flag a consumer can test. The
                // valued and structural props are all handled above, so only real
                // boolean flags reach here.
                try out.append(.{ .flag = p.name });
            }
        }

        const eff_parent: ?u32 = if (own_parent) |x| x else self.cur.intr_parent;
        child.intr_parent = eff_parent;

        // interrupts-extended is self-describing. Each entry names its controller,
        // and it supersedes interrupts. Otherwise decode against the effective parent.
        if (raw_extended) |v| {
            try self.lowerExtended(v, out);
        } else if (raw_interrupts) |v| {
            if (eff_parent) |ph| try self.lowerInterrupts(v, ph, out);
        }
    }

    /// reg is a list of (address, size) records, each `addr_cells`/`size_cells`
    /// u32 cells wide, big-endian. The ancestor `ranges` chain translates each
    /// base from the parent bus's child-address space up to CPU space.
    fn lowerReg(self: *const DtBackend, value: []const u8, out: *resource.List) backend.Error!void {
        const ca = self.cur.addr;
        const cs = self.cur.size;
        const stride = (ca + cs) * 4;
        if (stride == 0) return;
        var off: usize = 0;
        while (off + stride <= value.len) : (off += stride) {
            const base = readCells(value[off..], ca);
            const size = readCells(value[off + ca * 4 ..], cs);
            try out.append(.{ .mmio = .{ .base = self.translate(base), .size = size } });
        }
    }

    /// Translate a child-bus address to CPU space by applying each ancestor
    /// bus's `ranges`, from the device's immediate parent up to the root.
    fn translate(self: *const DtBackend, addr: u64) u64 {
        var a = addr;
        var d = self.depth;
        while (d > 0) {
            d -= 1;
            a = applyRanges(self.stack[d], a);
        }
        return a;
    }

    fn lookup(self: *const DtBackend, phandle: u32) ?Controller {
        for (self.controllers[0..self.nctrl]) |c| {
            if (c.phandle == phandle) return c;
        }
        return null;
    }

    /// Decode a plain `interrupts` list against controller `phandle`.
    fn lowerInterrupts(self: *const DtBackend, value: []const u8, phandle: u32, out: *resource.List) backend.Error!void {
        const ctrl = self.lookup(phandle) orelse return; // unknown parent: cannot size cells
        const stride = ctrl.cells * 4;
        if (stride == 0) return;
        var off: usize = 0;
        while (off + stride <= value.len) : (off += stride) {
            try decodeSpecifier(ctrl, value[off .. off + stride], out);
        }
    }

    /// Decode an `interrupts-extended` list (phandle + cells, repeated).
    fn lowerExtended(self: *const DtBackend, value: []const u8, out: *resource.List) backend.Error!void {
        var off: usize = 0;
        while (off + 4 <= value.len) {
            const phandle = std.mem.readInt(u32, value[off..][0..4], .big);
            off += 4;
            const ctrl = self.lookup(phandle) orelse return; // unknown stride: stop
            const stride = ctrl.cells * 4;
            if (off + stride > value.len) break;
            try decodeSpecifier(ctrl, value[off .. off + stride], out);
            off += stride;
        }
    }

    /// Build the interrupt-controller map: every node carrying both a `phandle`
    /// and `#interrupt-cells`, keyed by phandle. Single accumulator works because
    /// in the DTB a node's properties always precede its subnodes.
    fn buildControllerMap(self: *DtBackend) void {
        self.nctrl = 0;
        var it = self.reader.nodeIterator();
        var ph: ?u32 = null;
        var cells: ?u32 = null;
        var gic = false;
        var in_node = false;
        while (it.next() catch return) |n| {
            switch (n) {
                .begin => {
                    self.finalizeController(&ph, &cells, &gic);
                    in_node = true;
                },
                .end => {
                    self.finalizeController(&ph, &cells, &gic);
                    in_node = false;
                },
                .prop => |p| {
                    if (!in_node) continue;
                    if (std.mem.eql(u8, p.name, "phandle") or std.mem.eql(u8, p.name, "linux,phandle")) {
                        ph = beCell(p.value);
                    } else if (std.mem.eql(u8, p.name, "#interrupt-cells")) {
                        cells = beCell(p.value);
                    } else if (std.mem.eql(u8, p.name, "compatible")) {
                        gic = std.mem.indexOf(u8, p.value, "gic") != null;
                    }
                },
            }
        }
        self.finalizeController(&ph, &cells, &gic);
    }

    fn finalizeController(self: *DtBackend, ph: *?u32, cells: *?u32, gic: *bool) void {
        if (ph.* != null and cells.* != null and self.nctrl < max_controllers) {
            self.controllers[self.nctrl] = .{
                .phandle = ph.*.?,
                .cells = cells.*.?,
                .domain = if (gic.*) .arm_gic else .generic,
            };
            self.nctrl += 1;
        }
        ph.* = null;
        cells.* = null;
        gic.* = false;
    }
};

/// Decode one interrupt specifier (already sliced to `cells` words) per its
/// controller's domain, appending an `irq` resource.
fn decodeSpecifier(ctrl: Controller, bytes: []const u8, out: *resource.List) backend.Error!void {
    switch (ctrl.domain) {
        .arm_gic => {
            if (ctrl.cells < 3 or bytes.len < 12) {
                try out.append(.{ .irq = .{ .number = std.mem.readInt(u32, bytes[0..4], .big) } });
                return;
            }
            const kind = std.mem.readInt(u32, bytes[0..4], .big);
            const irq = std.mem.readInt(u32, bytes[4..8], .big);
            const flags = std.mem.readInt(u32, bytes[8..12], .big);
            // GIC: type 0 = SPI (+32), type 1 = PPI (+16). The flags low nibble
            // is the trigger/polarity (1 edge-rising, 2 edge-falling, 4 level-high,
            // 8 level-low).
            const number: u32 = switch (kind) {
                0 => irq + 32,
                1 => irq + 16,
                else => irq,
            };
            try out.append(.{ .irq = .{
                .number = number,
                .trigger = if (flags & 0x3 != 0) .edge else if (flags & 0xc != 0) .level else .unknown,
                .polarity = if (flags & 0xa != 0) .low else .high,
            } });
        },
        .generic => {
            try out.append(.{ .irq = .{ .number = std.mem.readInt(u32, bytes[0..4], .big) } });
        },
    }
}

fn splitCompatible(value: []const u8, ids: *match.IdList) void {
    var it = std.mem.splitScalar(u8, value, 0);
    while (it.next()) |s| {
        if (s.len == 0) continue;
        ids.append(s);
    }
}

fn beCell(value: []const u8) ?u32 {
    if (value.len < 4) return null;
    return std.mem.readInt(u32, value[0..4], .big);
}

/// Apply one bus's `ranges` to translate `addr` from the bus's child-address
/// space into its parent's. A null `ranges` (absent) or empty `ranges` (1:1) is
/// identity. An address outside every window passes through unchanged.
fn applyRanges(bus: Inherited, addr: u64) u64 {
    const raw = bus.ranges orelse return addr;
    if (raw.len == 0) return addr; // empty ranges: identity
    const ca = bus.addr; // child address cells
    const pa = bus.parent_addr; // parent address cells
    const sz = bus.size; // child size cells
    const stride = (ca + pa + sz) * 4;
    if (stride == 0) return addr;
    var off: usize = 0;
    while (off + stride <= raw.len) : (off += stride) {
        const child_base = readCells(raw[off..], ca);
        const parent_base = readCells(raw[off + ca * 4 ..], pa);
        const length = readCells(raw[off + (ca + pa) * 4 ..], sz);
        if (addr >= child_base and addr < child_base + length) {
            return addr - child_base + parent_base;
        }
    }
    return addr; // no matching window: passthrough
}

/// Read up to `count` big-endian u32 cells into a u64. It keeps the low `count`
/// cells. If `count > 2`, it drops the most-significant cells beyond 64 bits.
fn readCells(value: []const u8, count: u32) u64 {
    var v: u64 = 0;
    var i: u32 = 0;
    while (i < count) : (i += 1) {
        const off = i * 4;
        if (off + 4 > value.len) break;
        v = (v << 32) | std.mem.readInt(u32, value[off..][0..4], .big);
    }
    return v;
}

fn cells3(a: u32, b: u32, c: u32) [12]u8 {
    var out: [12]u8 = undefined;
    std.mem.writeInt(u32, out[0..4], a, .big);
    std.mem.writeInt(u32, out[4..8], b, .big);
    std.mem.writeInt(u32, out[8..12], c, .big);
    return out;
}

test "gic specifier: SPI/PPI offset and trigger/polarity" {
    const gic = Controller{ .phandle = 1, .cells = 3, .domain = .arm_gic };
    var out = resource.List{};

    // SPI 5, level / active-high (flags=4): number = 5 + 32 = 37.
    try decodeSpecifier(gic, &cells3(0, 5, 4), &out);
    try std.testing.expectEqual(@as(u32, 37), out.items[0].irq.number);
    try std.testing.expectEqual(resource.Trigger.level, out.items[0].irq.trigger);
    try std.testing.expectEqual(resource.Polarity.high, out.items[0].irq.polarity);

    // PPI 2, edge / active-low (flags=2): number = 2 + 16 = 18.
    try decodeSpecifier(gic, &cells3(1, 2, 2), &out);
    try std.testing.expectEqual(@as(u32, 18), out.items[1].irq.number);
    try std.testing.expectEqual(resource.Trigger.edge, out.items[1].irq.trigger);
    try std.testing.expectEqual(resource.Polarity.low, out.items[1].irq.polarity);
}

test "generic specifier: first cell is the irq number" {
    const plic = Controller{ .phandle = 2, .cells = 1, .domain = .generic };
    var out = resource.List{};
    var b: [4]u8 = undefined;
    std.mem.writeInt(u32, &b, 10, .big);
    try decodeSpecifier(plic, &b, &out);
    try std.testing.expectEqual(@as(u32, 10), out.items[0].irq.number);
}

test "ranges translation walks the bus chain" {
    var be: DtBackend = undefined;
    be.depth = 2;
    // root (stack[0]): no ranges -> identity at the top.
    be.stack[0] = .{ .addr = 2, .size = 2, .ranges = null };
    // intermediate bus (stack[1]): child cells 1, parent cells 2, size 1. It maps
    // child window [0x0, 0x1000) onto parent base 0x4000_0000.
    var r: [16]u8 = undefined;
    std.mem.writeInt(u32, r[0..4], 0x0, .big); // child base (1 cell)
    std.mem.writeInt(u64, r[4..12], 0x4000_0000, .big); // parent base (2 cells)
    std.mem.writeInt(u32, r[12..16], 0x1000, .big); // length (1 cell)
    be.stack[1] = .{ .addr = 1, .size = 1, .parent_addr = 2, .ranges = &r };

    // A device reg base of 0x100 in the bus's child space -> CPU 0x4000_0100.
    try std.testing.expectEqual(@as(u64, 0x4000_0100), be.translate(0x100));
    // An address outside every window passes through unchanged.
    try std.testing.expectEqual(@as(u64, 0x9999), be.translate(0x9999));
}
