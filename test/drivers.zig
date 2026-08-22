//! Round-trip tests for the GPIO/I2C/SPI/timer/block/intc drivers over fake
//! register files. The file driver.zig covers the serial drivers.

const std = @import("std");
const fake = @import("fake.zig");
const harbor_gpio = @import("conduit").driver.harbor_gpio;
const harbor_i2c = @import("conduit").driver.harbor_i2c;
const harbor_spi = @import("conduit").driver.harbor_spi;
const clint = @import("conduit").driver.clint;
const harbor_sdio = @import("conduit").driver.harbor_sdio;
const plic = @import("conduit").driver.plic;
const gicv2 = @import("conduit").driver.gicv2;
const gicv3 = @import("conduit").driver.gicv3;
const goldfish_rtc = @import("conduit").driver.goldfish_rtc;
const pl031 = @import("conduit").driver.pl031;

test "harbor_gpio: direction, output, input" {
    // Registers sit 8 bytes apart. With 4-byte spacing DIR and OUTPUT aliased
    // each other, so the direction write landed on the output value.
    var regs = fake.Flat(64){};
    std.mem.writeInt(u32, regs.buf[0x00..][0..4], 1 << 5, .little); // INPUT: pin 5 high
    var dev = harbor_gpio.bind(regs.mmio());
    const g = dev.gpio();

    g.direction(3, .output);
    try std.testing.expectEqual(@as(u32, 1 << 3), std.mem.readInt(u32, regs.buf[0x10..][0..4], .little)); // DIR
    g.set(3, true);
    try std.testing.expectEqual(@as(u32, 1 << 3), std.mem.readInt(u32, regs.buf[0x08..][0..4], .little)); // OUTPUT
    try std.testing.expect(g.get(5));
    try std.testing.expect(!g.get(4));

    // The direction write did not disturb the output register, and vice versa.
    dev.interruptEnable(7, true, true);
    try std.testing.expectEqual(@as(u32, 1 << 7), std.mem.readInt(u32, regs.buf[0x18..][0..4], .little)); // IRQ_EN
    try std.testing.expectEqual(@as(u32, 1 << 7), std.mem.readInt(u32, regs.buf[0x28..][0..4], .little)); // IRQ_EDGE
    try std.testing.expectEqual(@as(u32, 1 << 3), std.mem.readInt(u32, regs.buf[0x10..][0..4], .little)); // DIR intact
}

test "harbor_i2c: write phase acks and the last byte reaches DATA" {
    // Registers sit 8 bytes apart. STATUS is forced to CMD_DONE|ACK.
    var regs = fake.Flat(64){ .force_off = 0x08, .force_val = 0x22 };
    var dev = harbor_i2c.bind(regs.mmio(), .{ .prescale = 9 });
    const bus = dev.i2c();

    try std.testing.expect(bus.transfer(0x50, &.{ 0xab, 0xcd }, &.{}));
    try std.testing.expectEqual(@as(u32, 0xcd), std.mem.readInt(u32, regs.buf[0x10..][0..4], .little)); // DATA
    try std.testing.expectEqual(@as(u32, 9), std.mem.readInt(u32, regs.buf[0x20..][0..4], .little)); // PRESCALE
    try std.testing.expectEqual(@as(u32, 1), std.mem.readInt(u32, regs.buf[0x00..][0..4], .little)); // CTRL enable
}

test "harbor_i2c: a controller that never reports CMD_DONE fails the transfer" {
    // STATUS reads 0 forever, so no command ever completes. The driver must
    // give up rather than spin.
    var regs = fake.Flat(64){};
    var dev = harbor_i2c.bind(regs.mmio(), .{ .prescale = 9 });
    try std.testing.expect(!dev.i2c().transfer(0x50, &.{0xab}, &.{}));
}

test "harbor_spi: full-duplex loopback" {
    var regs = fake.Flat(64){}; // STATUS busy bit clear
    var dev = harbor_spi.bind(regs.mmio(), .{ .divider = 4, .mode = 0 });
    const bus = dev.spi();

    var rx: [3]u8 = undefined;
    bus.transfer(&.{ 1, 2, 3 }, &rx);
    try std.testing.expectEqualSlices(u8, &.{ 1, 2, 3 }, &rx); // DATA reads back what was written
    try std.testing.expectEqual(@as(u32, 4), std.mem.readInt(u32, regs.buf[0x18..][0..4], .little)); // DIVIDER
    try std.testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, regs.buf[0x20..][0..4], .little)); // CS released
}

test "harbor_spi: a short tx clocks the idle byte, and CS picks its own line" {
    var regs = fake.Flat(64){};
    var dev = harbor_spi.bind(regs.mmio(), .{ .mode = 3, .cs_line = 2 });

    // Mode 3 is CPOL + CPHA, and the enable bit stays set.
    try std.testing.expectEqual(@as(u32, 0b111), std.mem.readInt(u32, regs.buf[0x00..][0..4], .little));

    dev.chipSelect(true);
    try std.testing.expectEqual(@as(u32, 1 << 2), std.mem.readInt(u32, regs.buf[0x20..][0..4], .little));

    // Read four bytes while writing one: the exchange runs for four, and the
    // three unwritten slots clock 0xFF rather than zero.
    var rx: [4]u8 = undefined;
    dev.spi().transfer(&.{0xa5}, &rx);
    try std.testing.expectEqualSlices(u8, &.{ 0xa5, 0xff, 0xff, 0xff }, &rx);
}

test "clint: timer and IPI registers" {
    var regs = fake.Flat(0xc000){};
    std.mem.writeInt(u64, regs.buf[0xbff8..][0..8], 0x9999, .little); // MTIME
    const dev = clint.bind(regs.mmio());

    try std.testing.expectEqual(@as(u64, 0x9999), dev.time());
    dev.setTimecmp(0, 0x1234_5678_9abc);
    try std.testing.expectEqual(@as(u64, 0x1234_5678_9abc), std.mem.readInt(u64, regs.buf[0x4000..][0..8], .little));
    dev.sendIpi(1);
    try std.testing.expectEqual(@as(u32, 1), std.mem.readInt(u32, regs.buf[0x0004..][0..4], .little)); // MSIP[1]
}

test "harbor_sdio: a dead controller yields an empty block device" {
    // A flat register file never drives a response, so every command reports a
    // timeout and identification must give up rather than report a phantom card.
    var regs = fake.Flat(128){};
    var dev = harbor_sdio.bind(regs.mmio(), .{ .freq = 50_000_000 });
    try std.testing.expect(!dev.cardPresent());

    const blk = dev.block();
    try std.testing.expectEqual(@as(u64, 0), blk.num_blocks);
    var buf: [512]u8 = undefined;
    try std.testing.expect(!blk.readBlocks(0, 1, &buf));
    try std.testing.expect(!blk.writeBlocks(0, 1, &buf));
}

test "plic: enable programs priority + enable bit, claim reads the claim reg" {
    var regs = fake.Sparse{};
    const dev = plic.bind(regs.mmio(), .{ .context = 0 });

    dev.enable(10);
    try std.testing.expectEqual(@as(u64, 1), regs.lastWrite(10 * 4).?); // priority
    try std.testing.expectEqual(@as(u64, 1 << 10), regs.lastWrite(0x2000).?); // enable bits

    regs.setRead(0x200004, 7); // claim/complete register
    try std.testing.expectEqual(@as(u32, 7), dev.claim().?);
}

test "gicv2: enable sets ISENABLER, claim/complete hit the CPU interface" {
    var dist = fake.Sparse{};
    var cpu = fake.Sparse{};
    const g = gicv2.bind(dist.mmio(), cpu.mmio());

    g.enable(33); // SPI 33 -> ISENABLER bank 1, bit 1
    try std.testing.expectEqual(@as(u64, 1 << 1), dist.lastWrite(0x100 + 4).?);

    cpu.setRead(0x0c, 42); // GICC_IAR
    try std.testing.expectEqual(@as(u32, 42), g.claim().?);
    g.complete(42);
    try std.testing.expectEqual(@as(u64, 42), cpu.lastWrite(0x10).?); // GICC_EOIR
}

test "gicv3: SPI via distributor, PPI via redistributor" {
    // Construct directly, not via bind. The init(), claim(), and complete() calls
    // touch the EL1-only ICC_* system registers. Those registers trap from a
    // userspace test. The enable paths are pure MMIO. This test verifies them.
    var dist = fake.Sparse{};
    var redist = fake.Sparse{};
    const g = gicv3.Gicv3{ .dist = dist.mmio(), .redist = redist.mmio() };

    g.enable(40); // SPI -> GICD_ISENABLER bank 1, bit 8
    try std.testing.expectEqual(@as(u64, 1 << 8), dist.lastWrite(0x100 + 4).?);

    g.enable(5); // PPI -> GICR_ISENABLER0, bit 5
    try std.testing.expectEqual(@as(u64, 1 << 5), redist.lastWrite(0x10000 + 0x100).?);
}

test "goldfish_rtc: read nanoseconds and convert to a date" {
    var regs = fake.Flat(64){};
    const ns: u64 = 1609459200 * 1_000_000_000; // 2021-01-01 00:00:00 UTC
    std.mem.writeInt(u32, regs.buf[0x00..][0..4], @truncate(ns), .little);
    std.mem.writeInt(u32, regs.buf[0x04..][0..4], @truncate(ns >> 32), .little);

    var dev = goldfish_rtc.bind(regs.mmio());
    const dt = dev.rtc().now();
    try std.testing.expectEqual(@as(u16, 2021), dt.year);
    try std.testing.expectEqual(@as(u8, 1), dt.month);
    try std.testing.expectEqual(@as(u8, 1), dt.day);
}

test "pl031: read seconds, and set writes the load register" {
    var regs = fake.Flat(64){};
    std.mem.writeInt(u32, regs.buf[0x00..][0..4], 1609459200, .little); // DR
    var dev = pl031.bind(regs.mmio());

    const r = dev.rtc();
    try std.testing.expectEqual(@as(u16, 2021), r.now().year);

    _ = r.set(.{ .year = 2000, .month = 1, .day = 1, .hour = 0, .minute = 0, .second = 0 });
    try std.testing.expectEqual(@as(u32, 946684800), std.mem.readInt(u32, regs.buf[0x08..][0..4], .little)); // LR = Y2K epoch
}
