//! SD/MMC card in SPI mode, over a Harbor SPI master. It implements the `Block`
//! contract. A PmodSD or the OrangeCrab on-board slot wires an SD card to four
//! SPI lines (CS/MOSI/MISO/SCK). The driver runs the SPI-mode identification
//! sequence and the single-block PIO read and write. It uses the harbor_spi
//! transport. A card then serves as a generic block device. It needs no
//! dedicated SD host controller.
//!
//! The card is a child of the SPI bus. It is not its own MMIO node. Discovery
//! binds the SPI controller. This driver then probes for a card on the bus.

const std = @import("std");
const Mmio = @import("../mmio.zig");
const Block = @import("../device/block.zig");
const sd = @import("../device/sd.zig");
const match = @import("../match.zig");
const harbor_spi = @import("harbor_spi.zig");

pub const matcher = match.Matcher{
    .class = .block,
    .dt_compatible = &.{ "mmc-spi-slot", "sd-spi", "digilent,pmod-sd" },
    .driver = "sd_spi",
};

pub const Options = struct {
    /// SPI clock divider for the slow identification phase. SD cards must run
    /// identification at 100..400 kHz. On a 25 MHz SPI clock, 99 gives ~125 kHz.
    init_divider: u32 = 99,
    /// SPI clock divider for the data phase. The driver applies it after it finds
    /// a card. On a 25 MHz SPI clock, 3 gives ~3 MHz. 0 keeps the slow rate.
    data_divider: u32 = 3,
    /// The SPI controller has the DMA block engine. Passed through to the block
    /// read path, which uses DMA for the 512-byte payload when set.
    dma: bool = false,
};

// Untyped so it coerces to u32 (block_size), u64 (byte address), and usize
// (buffer offset) without per-site casts.
const BLOCK_LEN = 512;

// SD command opcodes (SPI mode).
const CMD0_GO_IDLE: u8 = 0;
const CMD8_SEND_IF_COND: u8 = 8;
const CMD9_SEND_CSD: u8 = 9;
const CMD10_SEND_CID: u8 = 10;
const CMD16_SET_BLOCKLEN: u8 = 16;
const CMD17_READ_SINGLE: u8 = 17;
const CMD24_WRITE_SINGLE: u8 = 24;
const ACMD41_SD_OP_COND: u8 = 41;
const CMD55_APP: u8 = 55;
const CMD58_READ_OCR: u8 = 58;

const R1_IDLE: u8 = 0x01;
const TOKEN_START: u8 = 0xFE; // single-block read/write data start
const DR_ACCEPTED: u8 = 0x05; // data-response token: data accepted

const DIVIDER_REG: u32 = 0x18;

/// Evaluate a call for its side effects and discard its result. A polled SPI
/// exchange clocks pad, token, and fill bytes it never reads, and a few commands
/// run only to advance the card. The discard is explicit so it reads as intended.
fn drop(value: anytype) void {
    _ = value;
}

pub const SdSpi = struct {
    spi: harbor_spi.HarborSpi,
    high_capacity: bool = false,
    num_blocks: u64 = 0,
    present: bool = false,
    /// Card Identification register (CMD10): manufacturer/OEM/product/serial/date.
    cid: [16]u8 = [_]u8{0} ** 16,

    /// CRC7 (poly 0x09) over the command bytes. It fills the trailing frame byte.
    /// The card checks only CMD0 and CMD8 in SPI mode. A correct CRC is cheap and
    /// always valid.
    fn crc7(data: []const u8) u8 {
        var crc: u8 = 0;
        for (data) |byte| {
            var i: u8 = 0;
            while (i < 8) : (i += 1) {
                const in: u8 = (byte >> @as(u3, @intCast(7 - i))) & 1;
                // 7-bit CRC, poly x^7+x^3+1: MSB is bit 6.
                const feedback: u8 = ((crc >> 6) & 1) ^ in;
                crc = (crc << 1) & 0x7f;
                if (feedback != 0) crc ^= 0x09;
            }
        }
        return crc & 0x7f;
    }

    fn xfer(self: *SdSpi, out: u8) u8 {
        return self.spi.transferByte(out);
    }

    /// Send a 6-byte command frame with CS already asserted, then poll for the
    /// R1 response (first byte with the top bit clear). Returns 0xFF on timeout.
    fn sendCmd(self: *SdSpi, cmd: u8, arg: u32) u8 {
        var frame = [6]u8{
            0x40 | cmd,
            @truncate(arg >> 24),
            @truncate(arg >> 16),
            @truncate(arg >> 8),
            @truncate(arg),
            0,
        };
        frame[5] = (crc7(frame[0..5]) << 1) | 1;
        drop(self.xfer(0xFF)); // Ncr pad byte
        for (frame) |b| drop(self.xfer(b));
        var i: u8 = 0;
        var r1: u8 = 0xFF;
        while (i < 10) : (i += 1) {
            r1 = self.xfer(0xFF);
            if (r1 & 0x80 == 0) break;
        }
        return r1;
    }

    /// A full CS-managed command returning R1. Releases the bus with a trailing
    /// byte so the card finishes its internal work.
    fn cmdR1(self: *SdSpi, cmd: u8, arg: u32) u8 {
        self.spi.chipSelect(true);
        const r = self.sendCmd(cmd, arg);
        self.spi.chipSelect(false);
        drop(self.xfer(0xFF));
        return r;
    }

    /// An application command (CMD55 prefix + ACMDn), CS held across both.
    fn appCmd(self: *SdSpi, acmd: u8, arg: u32) u8 {
        self.spi.chipSelect(true);
        drop(self.sendCmd(CMD55_APP, 0));
        const r = self.sendCmd(acmd, arg);
        self.spi.chipSelect(false);
        drop(self.xfer(0xFF));
        return r;
    }

    /// Poll for a data start token, then read `buf`. Discard the 2 CRC bytes.
    /// SPI mode turns CRC off by default, so the driver ignores those bytes.
    fn readData(self: *SdSpi, buf: []u8) bool {
        var tries: u32 = 0;
        var token: u8 = 0xFF;
        while (tries < 100000) : (tries += 1) {
            token = self.xfer(0xFF);
            if (token == TOKEN_START) break;
            // A low-nibble error token (top 3 bits zero) aborts the read.
            if (token != 0xFF and token & 0xE0 == 0) return false;
        }
        if (token != TOKEN_START) return false;
        // DMA the payload when the controller has the engine and the buffer fits
        // its rules, else clock it in a byte at a time. The token above and the
        // CRC below stay polled: the engine moves only the fixed-size payload.
        switch (self.spi.readDma(buf)) {
            .done => {},
            .failed => return false,
            .unavailable => {
                for (buf) |*b| b.* = self.xfer(0xFF);
            },
        }
        drop(self.xfer(0xFF)); // CRC16 high
        drop(self.xfer(0xFF)); // CRC16 low
        return true;
    }

    /// Card capacity in 512-byte blocks from the CSD (CMD9). Handles both CSD v2
    /// (SDHC/SDXC, block count = (C_SIZE+1) * 1024) and v1 (SDSC).
    fn readCapacity(self: *SdSpi) u64 {
        self.spi.chipSelect(true);
        const r = self.sendCmd(CMD9_SEND_CSD, 0);
        var csd: [16]u8 = undefined;
        var ok = false;
        if (r == 0) ok = self.readData(&csd);
        self.spi.chipSelect(false);
        drop(self.xfer(0xFF));
        if (!ok) return 0;

        const csd_ver = csd[0] >> 6;
        if (csd_ver == 1) {
            // CSD v2: C_SIZE is bits [69:48] = csd[7][5:0] : csd[8] : csd[9].
            const c_size: u64 = ((@as(u64, csd[7]) & 0x3f) << 16) |
                (@as(u64, csd[8]) << 8) | csd[9];
            return (c_size + 1) * 1024;
        }
        // CSD v1: capacity = (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^READ_BL_LEN.
        const read_bl_len: u6 = @intCast(csd[5] & 0x0f);
        const c_size: u64 = ((@as(u64, csd[6]) & 0x03) << 10) |
            (@as(u64, csd[7]) << 2) | (csd[8] >> 6);
        const c_size_mult: u3 = @intCast(((csd[9] & 0x03) << 1) | (csd[10] >> 7));
        const mult = @as(u64, 1) << (@as(u6, c_size_mult) + 2);
        const block_len = @as(u64, 1) << read_bl_len;
        return ((c_size + 1) * mult * block_len) / BLOCK_LEN;
    }

    /// Run the SPI-mode identification sequence. Returns true if a card answers.
    pub fn init(self: *SdSpi) bool {
        // Wake the card into SPI mode. The spec floor is 74 clocks with CS
        // de-asserted, but a cold card wants far more before it answers. Clock a
        // generous burst so a card that just got power has time to come up.
        self.spi.chipSelect(false);
        var w: u16 = 0;
        while (w < 80) : (w += 1) drop(self.xfer(0xFF));

        // CMD0: go idle. A cold card misses the first tries, so retry many times
        // and keep clocking idle bytes between tries to let it settle. At the
        // slow init rate each try is about 1.5 ms, so this waits several hundred
        // ms before it gives up.
        var idle = false;
        var t: u16 = 0;
        while (t < 500) : (t += 1) {
            if (self.cmdR1(CMD0_GO_IDLE, 0) == R1_IDLE) {
                idle = true;
                break;
            }
            drop(self.xfer(0xFF));
            drop(self.xfer(0xFF));
        }
        if (!idle) return false;

        // CMD8: voltage check. 0x1AA = 2.7-3.6V, check pattern 0xAA. An R1 with
        // the echo back marks a v2 card. An illegal-command bit marks v1.
        var v2 = false;
        self.spi.chipSelect(true);
        const r8 = self.sendCmd(CMD8_SEND_IF_COND, 0x1AA);
        if (r8 == R1_IDLE) {
            var b: [4]u8 = undefined;
            for (&b) |*x| x.* = self.xfer(0xFF);
            const echo = (@as(u32, b[0]) << 24) | (@as(u32, b[1]) << 16) |
                (@as(u32, b[2]) << 8) | b[3];
            if (echo & 0xFFF == 0x1AA) v2 = true;
        }
        self.spi.chipSelect(false);
        drop(self.xfer(0xFF));

        // ACMD41: initialise. HCS bit (0x40000000) asks for high-capacity on v2.
        var ready = false;
        var a: u32 = 0;
        const hcs: u32 = if (v2) 0x40000000 else 0;
        while (a < 20000) : (a += 1) {
            if (self.appCmd(ACMD41_SD_OP_COND, hcs) == 0) {
                ready = true;
                break;
            }
        }
        if (!ready) return false;

        // CMD58: read OCR. CCS (bit 30) distinguishes block-addressed SDHC/SDXC
        // from byte-addressed SDSC.
        if (v2) {
            self.spi.chipSelect(true);
            const r58 = self.sendCmd(CMD58_READ_OCR, 0);
            if (r58 == 0) {
                var ocr: [4]u8 = undefined;
                for (&ocr) |*x| x.* = self.xfer(0xFF);
                if (ocr[0] & 0x40 != 0) self.high_capacity = true;
            }
            self.spi.chipSelect(false);
            drop(self.xfer(0xFF));
        }

        // Byte-addressed cards need an explicit 512-byte block length.
        if (!self.high_capacity) drop(self.cmdR1(CMD16_SET_BLOCKLEN, BLOCK_LEN));

        self.num_blocks = self.readCapacity();

        // CMD10: read the CID (manufacturer, OEM, product, serial, date).
        self.spi.chipSelect(true);
        if (self.sendCmd(CMD10_SEND_CID, 0) == 0) drop(self.readData(&self.cid));
        self.spi.chipSelect(false);
        drop(self.xfer(0xFF));

        self.present = true;
        return true;
    }

    /// The shared CID accessor type. See device/sd.zig for its field methods
    /// (manufacturerId, oemId, productName, revision, serialNumber,
    /// manufactureDate).
    pub const Cid = sd.Cid;

    /// The card identity, read over SPI with CMD10 at bring-up. The SPI path
    /// reads the 16 CID bytes directly, so this is already in standard order.
    pub fn cidInfo(self: *const SdSpi) Cid {
        return .{ .raw = self.cid };
    }

    /// Convert a logical block address to the card's addressing (block for
    /// high-capacity, byte for standard).
    fn cardAddr(self: *const SdSpi, lba: u64) u32 {
        return if (self.high_capacity)
            @intCast(lba)
        else
            @intCast(lba * BLOCK_LEN);
    }

    fn readBlocksImpl(ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]u8) bool {
        const self: *SdSpi = @ptrCast(@alignCast(ctx));
        if (!self.present) return false;
        var i: u32 = 0;
        while (i < count) : (i += 1) {
            self.spi.chipSelect(true);
            const r = self.sendCmd(CMD17_READ_SINGLE, self.cardAddr(lba + i));
            var ok = false;
            if (r == 0) {
                const out = (buf + @as(usize, i) * BLOCK_LEN)[0..BLOCK_LEN];
                ok = self.readData(out);
            }
            self.spi.chipSelect(false);
            drop(self.xfer(0xFF));
            if (!ok) return false;
        }
        return true;
    }

    fn writeBlocksImpl(ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]const u8) bool {
        const self: *SdSpi = @ptrCast(@alignCast(ctx));
        if (!self.present) return false;
        var i: u32 = 0;
        while (i < count) : (i += 1) {
            self.spi.chipSelect(true);
            const r = self.sendCmd(CMD24_WRITE_SINGLE, self.cardAddr(lba + i));
            if (r != 0) {
                self.spi.chipSelect(false);
                drop(self.xfer(0xFF));
                return false;
            }
            drop(self.xfer(0xFF)); // gap before the data token
            drop(self.xfer(TOKEN_START));
            const src = (buf + @as(usize, i) * BLOCK_LEN)[0..BLOCK_LEN];
            for (src) |b| drop(self.xfer(b));
            drop(self.xfer(0xFF)); // CRC16 (ignored, CRC off)
            drop(self.xfer(0xFF));
            const dr = self.xfer(0xFF) & 0x1f;
            // Wait out the card's programming busy (holds MISO low = 0x00).
            var busy: u32 = 0;
            while (busy < 1000000) : (busy += 1) {
                if (self.xfer(0xFF) != 0x00) break;
            }
            self.spi.chipSelect(false);
            drop(self.xfer(0xFF));
            if (dr != DR_ACCEPTED) return false;
        }
        return true;
    }

    /// Present the card as a generic read/write block device.
    pub fn block(self: *SdSpi) Block {
        return .{
            .ctx = self,
            .block_size = BLOCK_LEN,
            .num_blocks = self.num_blocks,
            .read_blocks = readBlocksImpl,
            .write_blocks = writeBlocksImpl,
        };
    }

    pub fn cardPresent(self: *const SdSpi) bool {
        return self.present;
    }
};

/// Bind an SD card on the SPI controller at `mmio` and run identification. The
/// probe runs at the slow init rate. On success the driver raises the clock to
/// the data rate.
pub fn bind(mmio: Mmio, opts: Options) SdSpi {
    const spi = harbor_spi.bind(mmio, .{
        .divider = opts.init_divider,
        .mode = 0,
        .dma = opts.dma,
    });
    var dev = SdSpi{ .spi = spi };
    if (dev.init() and opts.data_divider != 0) {
        mmio.write(u32, DIVIDER_REG, opts.data_divider);
    }
    return dev;
}

// ---------------------------------------------------------------------------
// Host test: drive the driver against a fake SDHC card behind a fake SPI master.
// The fake `Mmio` emulates the HarborSpiController register map (STATUS never
// busy, DATA computes the MISO byte from the card FSM) so the real SdSpi ->
// HarborSpi -> Mmio path runs end to end.
// ---------------------------------------------------------------------------

const FakeCard = struct {
    const STATUS = 0x08;
    const DATA = 0x10;
    const CS = 0x20;
    const DMA_ADDR = 0x28;
    const DMA_LEN = 0x30;
    const DMA_CTRL = 0x38;
    const ST_DMA_BUSY: u64 = 1 << 3;
    const ST_DMA_DONE: u64 = 1 << 4;
    const DMA_START: u64 = 1 << 0;

    collecting: bool = false,
    cmd_buf: [6]u8 = [_]u8{0} ** 6,
    cmd_len: u8 = 0,
    resp: [700]u8 = [_]u8{0} ** 700,
    head: usize = 0,
    tail: usize = 0,
    last_in: u8 = 0xFF,
    selected: bool = false,
    // A cold card can miss its first CMD0s. This many CMD0s answer 0xFF (no idle
    // response) before the card finally reports idle (0x01), exercising the
    // driver's CMD0 retry budget.
    cmd0_misses: u8 = 0,
    // DMA engine model. A START pulls DMA_LEN bytes from the same card stream the
    // byte loop reads and writes them to host memory at DMA_ADDR, then latches
    // DMA_DONE. dma_count lets a test confirm the DMA path actually ran.
    dma_addr: usize = 0,
    dma_len: usize = 0,
    dma_done: bool = false,
    dma_count: usize = 0,
    // Models DMA_BUSY: real hardware holds STATUS.DMA_BUSY (bit 3) high while the
    // engine runs and clears it when the last beat lands. The engine's data is
    // written on START, but BUSY stays asserted for a few STATUS reads so the
    // driver's BUSY-gated wait (assert then clear) exercises the real handshake.
    busy_ticks: u8 = 0,
    // A transfer whose data lands only when BUSY clears (not on START), so a
    // driver that exits early on a stale DONE reads an unwritten buffer.
    dma_pending: bool = false,
    // When set, DMA_DONE ignores its write-1-to-clear (models the real SPI
    // controller RTL bug the BUSY gate defends against: a stuck DMA_DONE).
    done_sticky: bool = false,

    fn push(self: *FakeCard, b: u8) void {
        self.resp[self.tail] = b;
        self.tail += 1;
    }

    fn process(self: *FakeCard) void {
        const cmd = self.cmd_buf[0] & 0x3f;
        switch (cmd) {
            0 => { // CMD0 -> idle, after any cold-card misses
                if (self.cmd0_misses > 0) {
                    self.cmd0_misses -= 1;
                    self.push(0xFF); // not ready: no valid R1 yet
                } else {
                    self.push(0x01); // idle
                }
            },
            55 => self.push(0x01), // CMD55 -> idle
            8 => { // CMD8 -> R7 echo of 0x1AA
                self.push(0x01);
                self.push(0x00);
                self.push(0x00);
                self.push(0x01);
                self.push(0xAA);
            },
            58 => { // CMD58 -> OCR with CCS (high capacity) set
                self.push(0x00);
                self.push(0xC0);
                self.push(0xFF);
                self.push(0x80);
                self.push(0x00);
            },
            9 => { // CMD9 -> CSD v2, C_SIZE = 0x1FFF -> 8388608 blocks (4 GiB)
                self.push(0x00);
                self.push(TOKEN_START);
                var csd = [_]u8{0} ** 16;
                csd[0] = 0x40; // CSD structure v2
                csd[7] = 0x00;
                csd[8] = 0x1F;
                csd[9] = 0xFF;
                for (csd) |b| self.push(b);
                self.push(0xFF);
                self.push(0xFF);
            },
            10 => { // CMD10 -> CID: MID 0x03 (SanDisk), OEM "SD", PNM "SC64G"
                self.push(0x00);
                self.push(TOKEN_START);
                const cid = [16]u8{
                    0x03, 'S',  'D',  'S',  'C',  '6',  '4',  'G',
                    0x80, 0x11, 0x22, 0x33, 0x44, 0x01, 0x59, 0x00,
                };
                for (cid) |b| self.push(b);
                self.push(0xFF);
                self.push(0xFF);
            },
            17 => { // CMD17 -> single block, data[i] = i & 0xFF
                self.push(0x00);
                self.push(TOKEN_START);
                var i: u32 = 0;
                while (i < 512) : (i += 1) self.push(@truncate(i));
                self.push(0xFF);
                self.push(0xFF);
            },
            else => self.push(0x00), // ACMD41 ready, and any other command accepted
        }
    }

    fn onByte(self: *FakeCard, out: u8) u8 {
        if (self.head < self.tail) {
            const b = self.resp[self.head];
            self.head += 1;
            if (self.head == self.tail) {
                self.head = 0;
                self.tail = 0;
            }
            return b;
        }
        if (self.collecting) {
            self.cmd_buf[self.cmd_len] = out;
            self.cmd_len += 1;
            if (self.cmd_len == 6) {
                self.collecting = false;
                self.process();
            }
            return 0xFF;
        }
        if (out & 0xC0 == 0x40) {
            self.collecting = true;
            self.cmd_buf[0] = out;
            self.cmd_len = 1;
        }
        return 0xFF;
    }

    fn readFn(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const self: *FakeCard = @ptrCast(@alignCast(ctx));
        return switch (off) {
            STATUS => blk: {
                if (self.busy_ticks > 0) {
                    self.busy_ticks -= 1;
                    // The data lands and DONE latches only as the engine
                    // finishes, i.e. when BUSY is about to clear.
                    if (self.busy_ticks == 0 and self.dma_pending) {
                        const dst: [*]u8 = @ptrFromInt(self.dma_addr);
                        var i: usize = 0;
                        while (i < self.dma_len) : (i += 1) dst[i] = self.onByte(0xFF);
                        self.dma_done = true;
                        self.dma_count += 1;
                        self.dma_pending = false;
                    }
                    // Expose BUSY plus any leftover (possibly stuck) DONE. A
                    // driver that gated on DONE would wrongly exit here on a
                    // stale DONE while the engine is still running; a BUSY-gated
                    // one keeps waiting.
                    break :blk ST_DMA_BUSY | (if (self.dma_done) ST_DMA_DONE else 0);
                }
                break :blk if (self.dma_done) ST_DMA_DONE else 0;
            },
            DATA => self.last_in,
            else => 0,
        };
    }

    fn writeFn(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const self: *FakeCard = @ptrCast(@alignCast(ctx));
        switch (off) {
            STATUS => {
                // Write-1-to-clear DMA_DONE, unless done_sticky models the RTL
                // bug where the clear never lands.
                if (val & ST_DMA_DONE != 0 and !self.done_sticky) self.dma_done = false;
            },
            DATA => self.last_in = self.onByte(@truncate(val)),
            CS => {
                const active = val & 1 != 0;
                if (active and !self.selected) {
                    self.collecting = false;
                    self.cmd_len = 0;
                    self.head = 0;
                    self.tail = 0;
                }
                self.selected = active;
            },
            DMA_ADDR => self.dma_addr = @intCast(val),
            DMA_LEN => self.dma_len = @intCast(val),
            DMA_CTRL => {
                // Only the SD -> mem read direction is modelled (DIR clear).
                if (val & DMA_START != 0) {
                    // Arm the transfer. The data lands when BUSY clears (see
                    // readFn), not here, so an early-exiting driver sees an
                    // unwritten buffer.
                    self.busy_ticks = 3; // BUSY high for a few STATUS polls
                    self.dma_pending = true;
                }
            },
            else => {},
        }
    }

    fn mmio(self: *FakeCard) Mmio {
        return .{
            .ctx = self,
            .base = 0,
            .read_fn = readFn,
            .write_fn = writeFn,
        };
    }
};

test "sd-spi identifies an SDHC card and reads a block" {
    var card = FakeCard{};
    var dev = bind(card.mmio(), .{});

    try std.testing.expect(dev.present);
    try std.testing.expect(dev.high_capacity);
    try std.testing.expectEqual(@as(u64, 8388608), dev.num_blocks);

    // CID decode: MID 0x03, OEM "SD", product "SC64G", made 2021-09.
    const cid = dev.cidInfo();
    try std.testing.expectEqual(@as(u8, 0x03), cid.manufacturerId());
    try std.testing.expectEqualSlices(u8, "SD", &cid.oemId());
    try std.testing.expectEqualSlices(u8, "SC64G", &cid.productName());
    const date = cid.manufactureDate();
    try std.testing.expectEqual(@as(u16, 2021), date.year);
    try std.testing.expectEqual(@as(u8, 9), date.month);
    try std.testing.expectEqual(@as(u8, 8), cid.revision().major);
    try std.testing.expectEqual(@as(u8, 0), cid.revision().minor);
    try std.testing.expectEqual(@as(u32, 0x11223344), cid.serialNumber());

    var buf: [512]u8 = undefined;
    const blk = dev.block();
    try std.testing.expect(blk.readBlocks(0, 1, &buf));
    var i: usize = 0;
    while (i < 512) : (i += 1) try std.testing.expectEqual(@as(u8, @truncate(i)), buf[i]);
}

test "sd-spi reads a block through the DMA engine" {
    var card = FakeCard{};
    var dev = bind(card.mmio(), .{ .dma = true });
    try std.testing.expect(dev.present);

    // Ignore any DMA used while reading the CSD/CID during identification.
    card.dma_count = 0;

    var buf: [512]u8 align(8) = undefined;
    const blk = dev.block();
    try std.testing.expect(blk.readBlocks(0, 1, &buf));

    // The engine served the 512-byte payload, so the byte loop did not.
    try std.testing.expectEqual(@as(usize, 1), card.dma_count);
    var i: usize = 0;
    while (i < 512) : (i += 1) try std.testing.expectEqual(@as(u8, @truncate(i)), buf[i]);
}

test "sd-spi falls back to the byte loop for an unaligned buffer" {
    var card = FakeCard{};
    var dev = bind(card.mmio(), .{ .dma = true });
    try std.testing.expect(dev.present);
    card.dma_count = 0;

    // A buffer offset by one byte breaks the DMA alignment rule, so the driver
    // must read it through the polled path and still return the right data.
    var backing: [513]u8 align(8) = undefined;
    const buf = backing[1..513];
    const blk = dev.block();
    try std.testing.expect(blk.readBlocks(0, 1, buf));

    try std.testing.expectEqual(@as(usize, 0), card.dma_count);
    var i: usize = 0;
    while (i < 512) : (i += 1) try std.testing.expectEqual(@as(u8, @truncate(i)), buf[i]);
}

test "sd-spi DMA read is immune to a stuck DMA_DONE (BUSY-gated)" {
    var card = FakeCard{};
    var dev = bind(card.mmio(), .{ .dma = true });
    try std.testing.expect(dev.present);
    card.dma_count = 0;

    // Reproduce the real SPI-controller bug: DMA_DONE is stuck set from an
    // earlier transfer and its write-1-to-clear does nothing, and the fake
    // engine only lands its data as BUSY clears. A driver that polled DMA_DONE
    // would see the stale DONE, exit its wait immediately, and read an
    // unwritten buffer. The BUSY-gated readDma must ignore the stale DONE, wait
    // for the transfer, and return the real block data.
    card.dma_done = true;
    card.done_sticky = true;

    var buf: [512]u8 align(8) = undefined;
    @memset(&buf, 0xEE); // poison: an early-exit read would leave this
    const blk = dev.block();
    try std.testing.expect(blk.readBlocks(0, 1, &buf));

    try std.testing.expectEqual(@as(usize, 1), card.dma_count); // engine ran
    var i: usize = 0;
    while (i < 512) : (i += 1) try std.testing.expectEqual(@as(u8, @truncate(i)), buf[i]);
}

test "sd-spi init rides out a cold card that misses early CMD0s" {
    var card = FakeCard{};
    // The card ignores its first 50 CMD0 attempts (a stingy 10-retry budget
    // would give up here). Identification must still succeed.
    card.cmd0_misses = 50;
    var dev = bind(card.mmio(), .{});
    try std.testing.expect(dev.present);
    try std.testing.expectEqual(@as(u8, 0x03), dev.cidInfo().manufacturerId());
}

test "crc7 matches the known SD command CRC7 values" {
    // CMD0 frame -> CRC7 0x4A (trailing byte 0x95). CMD8 -> 0x43 (byte 0x87).
    try std.testing.expectEqual(@as(u8, 0x4A), SdSpi.crc7(&.{ 0x40, 0, 0, 0, 0 }));
    try std.testing.expectEqual(@as(u8, 0x43), SdSpi.crc7(&.{ 0x48, 0, 0, 0x01, 0xAA }));
}
