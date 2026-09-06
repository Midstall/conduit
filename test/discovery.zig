//! End-to-end discovery tests over a real QEMU riscv-virt device tree.

const std = @import("std");
const dtree = @import("dtree");
const dt = @import("conduit").backend.dtree;
const discover = @import("conduit").discover;
const match = @import("conduit").match;
const Matcher = match.Matcher;

const blob = @embedFile("riscv-virt.dtb").*;

const matchers = [_]Matcher{
    .{ .class = .uart, .dt_compatible = &.{ "ns16550a", "ns16550", "snps,dw-apb-uart" }, .driver = "ns16550a" },
    .{ .class = .block, .dt_compatible = &.{"virtio,mmio"}, .driver = "virtio_blk" },
    .{ .class = .intc, .dt_compatible = &.{ "riscv,plic0", "sifive,plic-1.0.0" }, .driver = "plic" },
};

test "runtime: registry finds the ns16550a uart with an mmio base" {
    var reader = try dtree.Reader.initBuffer(&blob);
    var be = dt.DtBackend.init(&reader);
    const reg = discover.Registry.init(be.any(), &matchers);

    const uart = (try reg.find(.uart)) orelse return error.NoUart;
    const region = uart.mmio() orelse return error.NoMmio;
    // QEMU virt puts the 16550 at 0x1000_0000.
    try std.testing.expectEqual(@as(u64, 0x1000_0000), region.base);
}

test "runtime: iterate ALL virtio-mmio block devices" {
    var reader = try dtree.Reader.initBuffer(&blob);
    var be = dt.DtBackend.init(&reader);
    const reg = discover.Registry.init(be.any(), &matchers);

    var it = reg.iter(.block);
    var count: usize = 0;
    while (try it.next()) |m| {
        try std.testing.expect(m.mmio() != null);
        count += 1;
    }
    // QEMU virt has several virtio-mmio slots.
    try std.testing.expect(count >= 1);
}

test "runtime: an empty-valued DT property lowers to a capability flag" {
    var reader = try dtree.Reader.initBuffer(&blob);
    var be = dt.DtBackend.init(&reader);
    const reg = discover.Registry.init(be.any(), &matchers);

    // The PLIC node carries the empty boolean `interrupt-controller;`, so the
    // match must report it as a flag, and must not report one it does not have.
    const plic = (try reg.find(.intc)) orelse return error.NoPlic;
    try std.testing.expect(plic.hasFlag("interrupt-controller"));
    try std.testing.expect(!plic.hasFlag("harbor,dma"));
}

test "comptime: Builder bakes the uart base into a constant" {
    const baked = comptime blk: {
        @setEvalBranchQuota(20_000_000);
        var reader = dtree.Reader.initBuffer(&blob) catch unreachable;
        var be = dt.DtBackend.init(&reader);
        break :blk discover.Builder.scan(&be, &matchers);
    };

    // Find the uart in the baked table at comptime.
    var found: ?u64 = null;
    inline for (baked) |m| {
        if (m.class == .uart) found = m.mmio().?.base;
    }
    try std.testing.expectEqual(@as(u64, 0x1000_0000), found.?);
}

test "runtime: the architectural timebase comes from the tree, not from a timer device" {
    var reader = try dtree.Reader.initBuffer(&blob);
    var be = dt.DtBackend.init(&reader);
    const reg = discover.Registry.init(be.any(), &matchers);

    // QEMU virt puts `timebase-frequency` on the /cpus parent, the older form of
    // the RISC-V binding, and runs mtime at 10 MHz.
    try std.testing.expectEqual(@as(?u64, 10_000_000), try reg.timebaseHz());
}

test "comptime: Builder bakes the architectural timebase into a constant" {
    const baked = comptime blk: {
        @setEvalBranchQuota(20_000_000);
        var reader = dtree.Reader.initBuffer(&blob) catch unreachable;
        var be = dt.DtBackend.init(&reader);
        break :blk discover.Builder.timebaseHz(&be);
    };
    try std.testing.expectEqual(@as(?u64, 10_000_000), baked);
}
