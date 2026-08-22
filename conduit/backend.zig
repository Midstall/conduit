//! The discovery-backend interface: the extension point that lets conduit pull
//! devices from device tree, ACPI, or a host's own model (Ferrite plugs its
//! kernel/9P discovery here). A backend yields raw device nodes and lowers each
//! node's native resource description into the normalized `resource.List`.
//!
//! Two ways to consume a backend:
//!   * The runtime `Registry` takes a type-erased `Backend` (this vtable).
//!   * The comptime `Builder` duck-types over a backend value directly (calling
//!     `.next()`/`.resources()`), because `anyopaque` indirection is not
//!     comptime-friendly.
//!
//! `fromImpl` wraps any impl that exposes `next`, `resources`, and `reset`
//! methods into a `Backend`.

const std = @import("std");
const match = @import("match.zig");
const resource = @import("resource.zig");

pub const Error = error{ Truncated, BadFormat, Unsupported, TooManyResources };

/// A PCI device's bus geometry and config-space identity. A `Node`/`Match`
/// carries it out-of-band because PCI matches on NUMERIC vendor/device/class
/// triples. The string-based `ids`/`Matcher` model cannot represent those. The
/// PCI backend also puts one synthetic id string ("pci") into `ids`. The
/// ordinary string `Matcher`/`Registry` path then still claims PCI nodes, and a
/// consumer refines on these exact numbers.
pub const PciInfo = struct {
    segment: u16 = 0,
    bus: u8 = 0,
    device: u8 = 0,
    function: u8 = 0,
    vendor_id: u16 = 0,
    device_id: u16 = 0,
    /// Base class code (PCI class register byte [23:16]), 0x03 = display.
    class_code: u8 = 0,
    subclass: u8 = 0,
    prog_if: u8 = 0,
    revision: u8 = 0,
};

/// A backend-neutral device node. `ids` are the node's identifiers (DT
/// `compatible` strings and/or ACPI HID/CID). `token` is an opaque cursor the
/// backend uses to re-find the node when lowering its resources. Only the PCI
/// backend sets `pci`. It carries that node's numeric bus identity.
pub const Node = struct {
    ids: match.IdList = .{},
    name: []const u8 = "",
    token: u64 = 0,
    pci: ?PciInfo = null,
};

pub const Backend = struct {
    ctx: ?*anyopaque,
    next_fn: *const fn (ctx: ?*anyopaque) Error!?Node,
    resources_fn: *const fn (ctx: ?*anyopaque, node: Node, out: *resource.List) Error!void,
    reset_fn: *const fn (ctx: ?*anyopaque) void,

    /// Advance to the next device node, or null when exhausted.
    pub fn next(self: Backend) Error!?Node {
        return self.next_fn(self.ctx);
    }

    /// Lower one node's resources into `out`. Call this for a node before the
    /// following `next`, because a backend may track the current node's context.
    pub fn resources(self: Backend, node: Node, out: *resource.List) Error!void {
        return self.resources_fn(self.ctx, node, out);
    }

    /// Restart iteration from the first node.
    pub fn reset(self: Backend) void {
        self.reset_fn(self.ctx);
    }
};

/// Wrap any backend impl pointer (with `next`/`resources`/`reset` methods) into
/// a type-erased `Backend`.
pub fn fromImpl(impl: anytype) Backend {
    const P = @TypeOf(impl);
    const gen = struct {
        fn next(ctx: ?*anyopaque) Error!?Node {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.next();
        }
        fn resources(ctx: ?*anyopaque, node: Node, out: *resource.List) Error!void {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.resources(node, out);
        }
        fn reset(ctx: ?*anyopaque) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.reset();
        }
    };
    return .{
        .ctx = impl,
        .next_fn = gen.next,
        .resources_fn = gen.resources,
        .reset_fn = gen.reset,
    };
}
