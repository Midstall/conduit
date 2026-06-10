//! ACPI static-table discovery test. Builds a real RSDP/XSDT/SPCR/MCFG/MADT set
//! with almanac's producer API, then discovers through conduit's Static backend.

const std = @import("std");
const almanac = @import("almanac");
const acpi_be = @import("conduit").backend.almanac;
const discover = @import("conduit").discover;
const match = @import("conduit").match;
const Matcher = match.Matcher;

const matchers = [_]Matcher{
    .{ .class = .uart, .dt_compatible = &.{"ns16550a"}, .driver = "ns16550a" },
    .{ .class = .pci, .dt_compatible = &.{"pci-host-ecam-generic"} },
    .{ .class = .intc, .dt_compatible = &.{ "acpi,ioapic", "arm,gic-v3" }, .driver = "ioapic" },
};

fn buildTables(buf: []u8) !almanac.TablesGeneric(almanac.OffsetMapper) {
    var b = almanac.Builder.init(buf, @intFromPtr(buf.ptr));

    // SPCR: ns16550a at system-memory 0x1000_0000, GSI 10.
    var spcr: [22]u8 = [_]u8{0} ** 22;
    spcr[0] = 0x00; // interface type: full 16550
    spcr[4] = 0; // GAS address space: system memory
    std.mem.writeInt(u64, spcr[8..16], 0x1000_0000, .little); // GAS address
    spcr[16] = 0; // interrupt type: no PIC bit -> use GSI
    std.mem.writeInt(u32, spcr[18..22], 10, .little); // GSI
    const spcr_phys = try b.addTable("SPCR", &spcr, 2);

    // MCFG: one ECAM allocation, base 0xE000_0000, bus 0..0.
    var mcfg: [24]u8 = [_]u8{0} ** 24;
    std.mem.writeInt(u64, mcfg[8..16], 0xE000_0000, .little); // base_address
    mcfg[18] = 0; // start_bus
    mcfg[19] = 0; // end_bus
    const mcfg_phys = try b.addTable("MCFG", &mcfg, 1);

    // MADT: one I/O APIC at 0xFEC0_0000.
    var madt: [20]u8 = [_]u8{0} ** 20;
    // madt[0..4] local apic addr, madt[4..8] flags, madt[8..] entries.
    madt[8] = 0x01; // type: I/O APIC
    madt[9] = 12; // length
    std.mem.writeInt(u32, madt[12..16], 0xFEC0_0000, .little); // address
    const madt_phys = try b.addTable("APIC", &madt, 4);

    const xsdt_phys = try b.xsdt(&.{ spcr_phys, mcfg_phys, madt_phys });
    const rsdp_phys = try b.rsdp(xsdt_phys);

    const Tables = almanac.TablesGeneric(almanac.OffsetMapper);
    return Tables.init(.{ .offset = 0 }, rsdp_phys);
}

test "acpi static: discover serial, pci ecam, and ioapic" {
    var buf: [1024]u8 align(16) = undefined;
    const tabs = try buildTables(&buf);

    var be = acpi_be.Static(almanac.OffsetMapper).init(tabs);
    const reg = discover.Registry.init(be.any(), &matchers);

    const uart = (try reg.find(.uart)) orelse return error.NoUart;
    try std.testing.expectEqual(@as(u64, 0x1000_0000), uart.mmio().?.base);

    const pci = (try reg.find(.pci)) orelse return error.NoPci;
    try std.testing.expectEqual(@as(u64, 0xE000_0000), pci.mmio().?.base);
    try std.testing.expectEqual(@as(u64, 0x10_0000), pci.mmio().?.size); // one bus

    const intc = (try reg.find(.intc)) orelse return error.NoIntc;
    try std.testing.expectEqual(@as(u64, 0xFEC0_0000), intc.mmio().?.base);
}
