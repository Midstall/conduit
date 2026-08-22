//! A small demonstration of conduit's two discovery entry points over a real
//! QEMU riscv-virt device tree. It shows a comptime `Builder` bake and a runtime
//! `Registry`. The shipped driver matcher table drives both. This does discovery
//! only. It does not poke real MMIO. The printed bases are physical addresses.

const std = @import("std");
const conduit = @import("conduit");
const dtree = @import("dtree");

const blob = @embedFile("test/riscv-virt.dtb").*;

// Comptime: bake the discovered device table straight into the binary.
const baked = blk: {
    @setEvalBranchQuota(20_000_000);
    var reader = dtree.Reader.initBuffer(&blob) catch unreachable;
    var be = conduit.backend.dtree.DtBackend.init(&reader);
    break :blk conduit.Builder.scan(&be, &conduit.all_matchers);
};

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    var out_buf: [512]u8 = undefined;
    var out = std.Io.File.stdout().writer(io, &out_buf);
    const w = &out.interface;

    try w.print("conduit: {d} device(s) baked at comptime from riscv-virt.dtb\n", .{baked.len});
    for (baked) |m| {
        try w.print("  {s:<6} {s:<22}", .{ @tagName(m.class), m.name });
        if (m.mmio()) |r| try w.print(" mmio=0x{x}", .{r.base});
        if (m.irq(0)) |q| try w.print(" irq={d}", .{q.number});
        if (m.driver) |d| try w.print(" driver={s}", .{d});
        try w.writeByte('\n');
    }

    // Runtime: run the same matcher table over a live Registry. Iterate every UART.
    var reader = try dtree.Reader.initBuffer(&blob);
    var be = conduit.backend.dtree.DtBackend.init(&reader);
    const reg = conduit.Registry.init(be.any(), &conduit.all_matchers);

    try w.writeAll("\nruntime: every .uart device:\n");
    var it = reg.iter(.uart);
    while (try it.next()) |m| {
        try w.print("  {s} at 0x{x}\n", .{ m.name, m.mmio().?.base });
    }

    try w.flush();
}
