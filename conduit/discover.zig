//! The two discovery entry points, sharing one `Match` type.
//!
//!   * `Registry` (runtime): wraps a type-erased `Backend` and lazily iterates
//!     every device of a class on demand.
//!   * `Builder` (comptime): duck-types over a backend value, baking a static
//!     `Match` table into the binary.
//!
//! A driver written against `Match` + `Mmio` never knows which discovered it.

const std = @import("std");
const backend = @import("backend.zig");
const match = @import("match.zig");
const resource = @import("resource.zig");

const Matcher = match.Matcher;
const Class = match.Class;

/// One discovered, matched device.
pub const Match = struct {
    class: Class,
    ids: match.IdList = .{},
    name: []const u8 = "",
    resources: resource.List = .{},
    /// Name of the shipped driver that binds this match, if the matcher named one.
    driver: ?[]const u8 = null,

    pub fn mmio(self: *const Match) ?resource.Resource.MmioRegion {
        return self.resources.mmio();
    }

    pub fn mmioAt(self: *const Match, n: usize) ?resource.Resource.MmioRegion {
        return self.resources.mmioAt(n);
    }

    pub fn irq(self: *const Match, n: usize) ?resource.Resource.Irq {
        return self.resources.irq(n);
    }
};

fn claim(matchers: []const Matcher, ids: []const match.Id, class: ?Class) ?Matcher {
    for (matchers) |m| {
        if (class) |c| if (m.class != c) continue;
        if (m.matches(ids)) return m;
    }
    return null;
}

/// Runtime discovery over a `Backend`.
pub const Registry = struct {
    be: backend.Backend,
    matchers: []const Matcher,

    pub fn init(be: backend.Backend, matchers: []const Matcher) Registry {
        return .{ .be = be, .matchers = matchers };
    }

    /// Iterate every device of `class`.
    pub fn iter(self: *const Registry, class: Class) Iterator {
        self.be.reset();
        return .{ .reg = self, .class = class };
    }

    /// Iterate every matched device, any class.
    pub fn iterAll(self: *const Registry) Iterator {
        self.be.reset();
        return .{ .reg = self, .class = null };
    }

    /// Convenience: the first device of `class`, if any.
    pub fn find(self: *const Registry, class: Class) backend.Error!?Match {
        var it = self.iter(class);
        return it.next();
    }

    pub const Iterator = struct {
        reg: *const Registry,
        class: ?Class,

        pub fn next(self: *Iterator) backend.Error!?Match {
            while (try self.reg.be.next()) |node| {
                const m = claim(self.reg.matchers, node.ids.slice(), self.class) orelse continue;
                var list = resource.List{};
                try self.reg.be.resources(node, &list);
                return Match{
                    .class = m.class,
                    .ids = node.ids,
                    .name = node.name,
                    .resources = list,
                    .driver = m.driver,
                };
            }
            return null;
        }
    };
};

/// Upper bound on devices a comptime scan bakes.
pub const max_matches = 256;

/// Comptime discovery. `be` is a pointer to a backend impl exposing
/// `next`/`resources`/`reset` (e.g. `*DtBackend`). Returns a comptime slice of
/// every matched device; the data is baked into the binary.
pub const Builder = struct {
    pub fn scan(be: anytype, matchers: []const Matcher) []const Match {
        if (!@inComptime()) @compileError("Builder.scan is comptime-only; use Registry at runtime");
        be.reset();
        var buf: [max_matches]Match = undefined;
        var n: usize = 0;
        while (be.next() catch |e| @compileError("conduit: device-tree scan failed: " ++ @errorName(e))) |node| {
            const m = claim(matchers, node.ids.slice(), null) orelse continue;
            var list = resource.List{};
            be.resources(node, &list) catch |e|
                @compileError("conduit: resource lowering failed for node '" ++ node.name ++ "': " ++ @errorName(e));
            if (n >= max_matches) @compileError("conduit: more than max_matches devices; raise max_matches");
            buf[n] = .{
                .class = m.class,
                .ids = node.ids,
                .name = node.name,
                .resources = list,
                .driver = m.driver,
            };
            n += 1;
        }
        const out = buf;
        const len = n;
        return out[0..len];
    }
};

/// Lazily filter a baked `Match` slice to one class. Works at comptime or
/// runtime, needs no storage. Typically used over `Builder.scan`'s output.
pub fn ofClass(matches: []const Match, class: Class) ClassIter {
    return .{ .matches = matches, .class = class, .i = 0 };
}

pub const ClassIter = struct {
    matches: []const Match,
    class: Class,
    i: usize = 0,

    pub fn next(self: *ClassIter) ?*const Match {
        while (self.i < self.matches.len) {
            const m = &self.matches[self.i];
            self.i += 1;
            if (m.class == self.class) return m;
        }
        return null;
    }
};
