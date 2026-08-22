//! This test discovers full ACPI (AML) devices. It loads a compiled DSDT into an
//! almanac namespace and interpreter. Then it enumerates devices through
//! conduit's Full backend. It matches on _HID and lowers _CRS into resources.

const std = @import("std");
const almanac = @import("almanac");
const acpi_be = @import("conduit").backend.almanac;
const discover = @import("conduit").discover;
const match = @import("conduit").match;

const dsdt = @embedFile("conduit-dsdt.aml").*;

test "acpi full: enumerate an AML device by _HID and lower its _CRS" {
    const gpa = std.testing.allocator;

    var namespace = try almanac.Namespace.init(gpa);
    defer namespace.deinit();
    try namespace.loadTable(&dsdt);

    var interp = try almanac.Interpreter.init(gpa, &namespace, .{});
    defer interp.deinit();

    var be = acpi_be.Full.init(&interp);
    defer be.reset(); // free the device walk's arena state

    const matchers = [_]match.Matcher{
        .{ .class = .uart, .acpi_hid = &.{"PNP0501"}, .driver = "ns16550a" },
    };
    const reg = discover.Registry.init(be.any(), &matchers);

    const uart = (try reg.find(.uart)) orelse return error.NoUart;
    try std.testing.expectEqual(@as(u64, 0x1000_0000), uart.mmio().?.base);
    try std.testing.expectEqual(@as(u32, 10), uart.irq(0).?.number);
}
