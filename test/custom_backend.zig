//! Proves the Ferrite extension point: a hand-rolled backend (no dtree, no
//! almanac) drives the same Registry and device-class machinery.

const std = @import("std");
const backend = @import("conduit").backend;
const match = @import("conduit").match;
const resource = @import("conduit").resource;
const discover = @import("conduit").discover;

const FakeDevice = struct { ids: []const []const u8, base: u64 };

/// A backend over an in-memory device list, standing in for a host's own
/// discovery model (e.g. Ferrite's kernel/9P probe).
const FakeBackend = struct {
    devices: []const FakeDevice,
    i: usize = 0,

    pub fn reset(self: *FakeBackend) void {
        self.i = 0;
    }

    pub fn next(self: *FakeBackend) backend.Error!?backend.Node {
        if (self.i >= self.devices.len) return null;
        const d = self.devices[self.i];
        self.i += 1;
        var ids = match.IdList{};
        for (d.ids) |id| ids.append(id);
        return backend.Node{ .ids = ids, .name = "dev", .token = @intCast(self.i - 1) };
    }

    pub fn resources(self: *FakeBackend, node: backend.Node, out: *resource.List) backend.Error!void {
        const d = self.devices[@intCast(node.token)];
        try out.append(.{ .mmio = .{ .base = d.base, .size = 0x1000 } });
    }
};

test "custom backend drives discovery without dtree or almanac" {
    const devs = [_]FakeDevice{
        .{ .ids = &.{"acme,uart"}, .base = 0x4000_0000 },
        .{ .ids = &.{"acme,gpio"}, .base = 0x4001_0000 },
        .{ .ids = &.{"acme,uart"}, .base = 0x4002_0000 },
    };
    var fb = FakeBackend{ .devices = &devs };
    const matchers = [_]match.Matcher{
        .{ .class = .uart, .dt_compatible = &.{"acme,uart"} },
        .{ .class = .gpio, .dt_compatible = &.{"acme,gpio"} },
    };
    const reg = discover.Registry.init(backend.fromImpl(&fb), &matchers);

    // First uart.
    const u = (try reg.find(.uart)) orelse return error.NoUart;
    try std.testing.expectEqual(@as(u64, 0x4000_0000), u.mmio().?.base);

    // Iterate ALL uarts: there are two.
    var it = reg.iter(.uart);
    var count: usize = 0;
    while (try it.next()) |_| count += 1;
    try std.testing.expectEqual(@as(usize, 2), count);
}
