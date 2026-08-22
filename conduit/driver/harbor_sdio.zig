//! Harbor SD/SDIO/eMMC host driver, over an `Mmio`. It implements the `Block`
//! contract. The controller is `HarborSdioController` (harbor's
//! lib/src/peripherals/sdio.dart). It is NOT the SD Host Controller Standard
//! (SDHCI) register map, so this driver must not be confused with a generic
//! sdhci one: the register layout below is Harbor's own.
//!
//! Each register sits in its own 8-byte slot, because the controller sits on a
//! byte-addressed fabric that decodes byte-offset >> 3 (like every other Harbor
//! peripheral). 4-byte spacing aliases every register.
//!   0x00 CTRL, 0x08 STATUS, 0x10 CLK_DIV, 0x18 CMD, 0x20 CMD_ARG,
//!   0x28 RESP0, 0x30 RESP1, 0x38 RESP2, 0x40 RESP3, 0x48 DATA,
//!   0x50 BLK_SIZE, 0x58 BLK_COUNT, 0x60 INT_STATUS, 0x68 INT_ENABLE,
//!   0x70 ADMA_ADDR.
//!
//! The driver runs the SD identification sequence, then reads and writes blocks
//! through the ADMA descriptor engine. ADMA, not PIO: the controller's DAT
//! engine paces itself from the SD clock, and only the ADMA path has the
//! flow control (it stalls the SD clock while a word waits) that keeps a
//! transfer correct at the 25 MHz data rate.

const std = @import("std");
const builtin = @import("builtin");
const Mmio = @import("../mmio.zig");
const Block = @import("../device/block.zig");
const sd = @import("../device/sd.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .block,
    // `harbor,sdio` is the name for this controller. `harbor,sdhci` and
    // `harbor,sdhci-emmc` are what the hardware description emits today, and
    // the `midstall,` forms are the older vendor-prefixed spellings. All four
    // name the same register map.
    .dt_compatible = &.{
        "harbor,sdio",
        "harbor,sdhci",
        "harbor,sdhci-emmc",
        "midstall,harbor-sdio",
        "midstall,harbor-sdhci",
    },
    .driver = "harbor_sdio",
};

/// Data-bus width to run the card at. The controller clamps the selection to
/// the width it was built with, so asking for more lanes than the hardware has
/// falls back to the hardware maximum.
pub const BusWidth = enum(u2) {
    one = 0,
    four = 1,
    eight = 2,
};

pub const Options = struct {
    /// Controller input clock in Hz. The driver derives the SD clock divider
    /// from it. 0 leaves the divider at its reset value, so the card runs at
    /// whatever rate the hardware came up with.
    freq: u32 = 0,
    /// Bus width to negotiate after identification. `.one` skips ACMD6.
    bus_width: BusWidth = .four,
    /// SD clock during identification. The spec window is 100..400 kHz.
    init_hz: u32 = 400_000,
    /// SD clock for data transfers, applied after identification.
    data_hz: u32 = 25_000_000,
};

pub const HarborSdio = struct {
    mmio: Mmio,
    clk_freq: u32 = 0,
    bus_width: BusWidth = .four,
    data_hz: u32 = 25_000_000,
    rca: u32 = 0,
    high_capacity: bool = false,
    num_blocks: u64 = 0,
    present: bool = false,
    // Card identity, captured from the ALL_SEND_CID R2 response at bring-up.
    cid: sd.Cid = .{ .raw = [_]u8{0} ** 16 },
    // ADMA descriptor for one transfer: [buffer addr], [len[15:0] | end<<31].
    // It lives in the instance (DRAM in M-mode) so it has a stable physical
    // address the DMA master can fetch. 8-byte aligned for the two-word fetch.
    desc: [2]u32 align(8) = .{ 0, 0 },
    // Aligned bounce buffer, one multi-block chunk wide. The ADMA moves 32-bit
    // beats to and from a physical address, so it needs an aligned target the
    // caller's `buf` may not give.
    bounce: [CHUNK_BLOCKS * BLOCK_LEN]u8 align(8) = [_]u8{0} ** (CHUNK_BLOCKS * BLOCK_LEN),

    const CTRL = 0x00;
    const STATUS = 0x08;
    const CLK_DIV = 0x10;
    const CMD = 0x18;
    const CMD_ARG = 0x20;
    const RESP0 = 0x28; // RESP1..3 follow at +8, +16, +24
    const DATA = 0x48;
    const BLK_SIZE = 0x50;
    const BLK_COUNT = 0x58;
    const INT_STATUS = 0x60; // write-1-to-clear
    const INT_ENABLE = 0x68;
    const ADMA_ADDR = 0x70; // ADMA descriptor pointer (physical, 32-bit)

    // STATUS is read-only.
    const ST_CARD_DETECT: u32 = 1 << 0;
    const ST_BUSY: u32 = 1 << 8;
    const ST_DATA_VALID: u32 = 1 << 9; // a block word is ready to pop from DATA

    // CTRL[0] enable, CTRL[5:4] active bus width (0:1-bit, 1:4-bit, 2:8-bit).
    const CTRL_ENABLE: u32 = 1 << 0;
    const CTRL_WIDTH_SHIFT: u5 = 4;

    // INT_STATUS bits (write-1-to-clear).
    const I_CMD_DONE: u32 = 0x01;
    const I_DATA_DONE: u32 = 0x02; // block transfer finished
    const I_DATA_REQ: u32 = 0x04; // the DATA holding register needs service
    const I_DATA_CRC: u32 = 0x08; // a received block failed CRC16
    const I_CMD_TIMEOUT: u32 = 0x10; // the card never drove a response
    const I_WRITE_ERR: u32 = 0x20; // the card rejected a written block
    const I_ALL: u32 = 0x3f;

    // CMD register: [5:0] index, [7:6] response type, [8] data present, [9] read
    // direction, [10] ADMA DMA. The controller uses the response-type bits to
    // decide whether (and how long) to capture the card's response. Without them
    // it sends every command as no-response and never fills RESP0..3.
    const R_NONE = 0; // no response (CMD0)
    const R_SHORT = 1; // 48-bit R1/R3/R6/R7
    const R_LONG = 2; // 136-bit R2 (CID/CSD)
    const R_BUSY = 3; // R1b (short + busy)
    const F_DATA = 1 << 8; // the command carries a data phase
    const F_READ = 1 << 9; // data direction card -> host (clear = host -> card)
    const F_DMA = 1 << 10; // stream the data phase through the ADMA engine
    const F_DATA_READ = F_DATA | F_READ;
    const F_DATA_WRITE = F_DATA;

    const GO_IDLE = 0;
    const ALL_SEND_CID = 2;
    const SEND_REL_ADDR = 3;
    const SELECT_CARD = 7;
    const SEND_IF_COND = 8;
    const SEND_CSD = 9;
    const STOP_TRANSMISSION = 12; // CMD12, ends an open-ended multi-block read
    const SET_BLOCKLEN = 16;
    const READ_SINGLE = 17;
    const READ_MULTIPLE = 18; // CMD18, streams BLK_COUNT blocks in one transfer
    const WRITE_SINGLE = 24;
    const SET_BUS_WIDTH = 6; // ACMD6, arg 2 = 4-bit
    const APP_CMD = 55;
    const SD_SEND_OP_COND = 41; // ACMD41

    const BLOCK_LEN = 512;
    // Blocks per multi-block DMA transfer. One CMD18 + one ADMA descriptor + one
    // coherency flush covers a whole chunk, amortising the per-block cost.
    //
    // CAPPED AT 16: a single CMD18 chunk larger than 16 blocks (>8 KiB) returns
    // CORRUPT data on this hardware (HW-measured: n=16 reads correct, n>=32
    // returns ok=false, and on the direct-DMA path the corruption escalates to a
    // core reset mid-transfer). The descriptor length field is fine (16-bit,
    // loads 16384 correctly), so the fault is in the CMD18/DAT receive path for
    // long continuous streams, not the descriptor. Until that RTL bug is fixed,
    // 16-block chunks are the largest verified-correct size; a larger logical
    // read is split into 16-block chunks by readBlocksImpl. n=16 runs at
    // ~545 us/blk, ~24x the old bounce-copy driver.
    const CHUNK_BLOCKS = 16;

    // A command clears BUSY within a bounded number of SD clocks (the response
    // window, or the data phase plus the card's programming time). This cap sits
    // far above that at any divider, so a healthy transfer never reaches it. A
    // controller that never clears BUSY (unclocked or wedged) hits the cap and
    // the command fails, rather than hanging the machine on the poll.
    const BUSY_POLL_MAX: usize = 4_000_000;

    fn waitCmd(self: *HarborSdio) bool {
        var t: usize = BUSY_POLL_MAX;
        while (self.mmio.read(u32, STATUS) & ST_BUSY != 0) {
            t -= 1;
            if (t == 0) return false;
        }
        return true;
    }

    fn cmd(self: *HarborSdio, opcode: u32, arg: u32, resp: u32) ?[4]u32 {
        return self.cmdFlags(opcode, arg, resp, 0);
    }

    /// Issue one command. Returns the four response words, or null when the
    /// controller never went idle or the card never answered. Clearing
    /// INT_STATUS first is what makes the timeout check below mean this command
    /// and not a previous one.
    fn cmdFlags(self: *HarborSdio, opcode: u32, arg: u32, resp: u32, flags: u32) ?[4]u32 {
        self.mmio.write(u32, INT_STATUS, I_ALL);
        self.mmio.write(u32, CMD_ARG, arg);
        self.mmio.write(u32, CMD, (opcode & 0x3f) | (resp << 6) | flags);
        if (!self.waitCmd()) return null;
        if (self.mmio.read(u32, INT_STATUS) & I_CMD_TIMEOUT != 0) return null;
        return .{
            self.mmio.read(u32, RESP0),
            self.mmio.read(u32, RESP0 + 8),
            self.mmio.read(u32, RESP0 + 16),
            self.mmio.read(u32, RESP0 + 24),
        };
    }

    /// Program the SD clock. The controller halves the input clock once more
    /// (CLK_DIV counts full periods), so the divider is freq/(2*hz) - 1.
    fn setClock(self: *HarborSdio, hz: u32) void {
        if (hz == 0 or self.clk_freq == 0) return;
        const half = self.clk_freq / (2 * hz);
        self.mmio.write(u32, CLK_DIV, if (half > 0) half - 1 else 0);
    }

    fn setCtrl(self: *HarborSdio, width: BusWidth) void {
        const w: u32 = @intFromEnum(width);
        self.mmio.write(u32, CTRL, CTRL_ENABLE | (w << CTRL_WIDTH_SHIFT));
    }

    /// True while a card is in the slot, per the card-detect pin.
    pub fn cardDetect(self: *const HarborSdio) bool {
        return self.mmio.read(u32, STATUS) & ST_CARD_DETECT != 0;
    }

    /// Bring up the host and initialize the inserted card. Returns true on
    /// success. This is the SD (not eMMC) sequence: an eMMC part answers CMD1
    /// instead of ACMD41 and does not report an RCA, so it needs its own path.
    pub fn init(self: *HarborSdio) bool {
        self.present = false;
        self.mmio.write(u32, CTRL, CTRL_ENABLE); // power on, 1-bit
        self.mmio.write(u32, INT_ENABLE, 0); // poll, do not raise the line
        self.mmio.write(u32, INT_STATUS, I_ALL);
        self.mmio.write(u32, BLK_SIZE, BLOCK_LEN);
        self.mmio.write(u32, BLK_COUNT, 1);
        self.setClock(self.init_hz_or_default());

        // The card-detect pin is not gated on here. Its pull-up and polarity are
        // board wiring the hardware description does not always constrain, so a
        // wrong read would abort identification before the card sees a command.
        // CMD0 and CMD8 are the real presence test. `cardDetect()` stays public
        // for a board that knows its own CD wiring is good.

        _ = self.cmd(GO_IDLE, 0, R_NONE) orelse return false;

        const if_cond = self.cmd(SEND_IF_COND, 0x1aa, R_SHORT);
        const v2 = if (if_cond) |c| (c[0] & 0xff) == 0xaa else false;

        // ACMD41 with the host-capacity-support bit, until the card leaves its
        // power-up state (OCR bit 31).
        const hcs: u32 = if (v2) 0x40000000 else 0;
        var tries: usize = 100_000;
        var ocr: u32 = 0;
        while (tries > 0) : (tries -= 1) {
            _ = self.cmd(APP_CMD, 0, R_SHORT) orelse return false;
            const resp = self.cmd(SD_SEND_OP_COND, hcs | 0x00ff8000, R_SHORT) orelse return false;
            ocr = resp[0];
            if (ocr & 0x80000000 != 0) break;
        }
        if (ocr & 0x80000000 == 0) return false;
        self.high_capacity = ocr & 0x40000000 != 0;

        self.cid = sd.Cid.fromR2(self.cmd(ALL_SEND_CID, 0, R_LONG) orelse return false);
        const rel = self.cmd(SEND_REL_ADDR, 0, R_SHORT) orelse return false;
        self.rca = rel[0] >> 16;

        // CMD9 (SEND_CSD) is a stand-by-state command, so read the CSD BEFORE
        // selecting the card. After CMD7 the card is in transfer state and does
        // not answer CMD9.
        self.num_blocks = self.readCapacity();

        _ = self.cmd(SELECT_CARD, self.rca << 16, R_BUSY) orelse return false;

        if (!self.high_capacity) _ = self.cmd(SET_BLOCKLEN, BLOCK_LEN, R_SHORT) orelse return false;

        // Widen the bus. ACMD6 tells the card (arg 2 = 4-bit), then CTRL[5:4]
        // sets the controller lanes to match. Both sides must agree or the
        // receive engine samples the wrong lanes. eMMC 8-bit uses CMD6 (SWITCH),
        // not ACMD6, so only the 4-bit widening is done here.
        if (self.bus_width == .four) {
            _ = self.cmd(APP_CMD, self.rca << 16, R_SHORT) orelse return false;
            _ = self.cmd(SET_BUS_WIDTH, 2, R_SHORT) orelse return false;
            self.setCtrl(.four);
        } else {
            self.setCtrl(.one);
        }

        self.setClock(self.data_hz);
        self.present = true;
        return true;
    }

    fn init_hz_or_default(self: *const HarborSdio) u32 {
        _ = self;
        return 400_000;
    }

    /// Card capacity in 512-byte blocks, from the CSD (CMD9, R2).
    ///
    /// The controller stores the R2 response with direct alignment: resp[i]
    /// holds CSD bits [32*i+31 : 32*i], so resp[3] is CSD[127:96]. CSD_STRUCTURE
    /// is CSD[127:126], the top 2 bits of resp[3].
    fn readCapacity(self: *HarborSdio) u64 {
        const csd = self.cmd(SEND_CSD, self.rca << 16, R_LONG) orelse return 0;
        const csd_structure = (csd[3] >> 30) & 0x3;
        if (csd_structure == 1) {
            // CSD v2 (SDHC/SDXC): C_SIZE is CSD[69:48].
            const c_size: u64 = ((@as(u64, csd[2]) & 0x3f) << 16) | (csd[1] >> 16);
            return (c_size + 1) * 1024;
        }
        if (csd_structure == 0) {
            // CSD v1 (SDSC): capacity = (C_SIZE+1) * 2^(C_SIZE_MULT+2) blocks of
            // 2^READ_BL_LEN bytes. C_SIZE is CSD[73:62], C_SIZE_MULT CSD[49:47],
            // READ_BL_LEN CSD[83:80].
            const read_bl_len: u6 = @intCast((csd[2] >> 16) & 0x0f);
            const c_size: u64 = ((@as(u64, csd[2]) & 0x3ff) << 2) | (csd[1] >> 30);
            const c_size_mult: u6 = @intCast((csd[1] >> 15) & 0x07);
            const mult = @as(u64, 1) << (c_size_mult + 2);
            const block_len = @as(u64, 1) << read_bl_len;
            return ((c_size + 1) * mult * block_len) / BLOCK_LEN;
        }
        return 0; // an unknown CSD version: report unknown, do not guess
    }

    // Flush the whole cache to keep DMA and the CPU coherent. On River any fence
    // flushes the entire D+I cache and the fetch TLB, so there is no line-address
    // math. This leans on that River behaviour, which is safe because the ADMA
    // master is River.
    fn cacheFlush() void {
        switch (builtin.target.cpu.arch) {
            .riscv64, .riscv32 => asm volatile ("fence.i" ::: .{ .memory = true }),
            else => asm volatile ("" ::: .{ .memory = true }),
        }
    }

    // DIAG ONLY: busy-wait to space out back-to-back DMA reads, to isolate
    // whether inter-read spacing is the missing factor vs the RTL idle-drain.
    fn drainCycles(n: u32) void {
        var i: u32 = 0;
        while (i < n) : (i += 1) asm volatile ("" ::: .{ .memory = true });
    }

    // Wait for everything already queued to DRAM to land, by reading one word of
    // it back.
    //
    // A DRAM write ACK does NOT mean the data reached the array: the controller
    // acks when it accepts the command. The fabric path to DRAM does serve one
    // request at a time from a single in-order queue though, and the burst
    // adapter flushes a held write burst that a read needs, so a read issued
    // after a write cannot complete before that write has landed. One round trip
    // replaces a spin of thousands of cycles.
    //
    // `addr` must be a location whose ordering matters, and must not be
    // satisfiable from cache: either just stored to (a write-through store
    // invalidates the line) or read after a cacheFlush.
    fn readBarrier(addr: *const volatile u32) void {
        _ = addr.*;
    }

    // Order and drain CPU stores to memory. The descriptor and (on a write) the
    // payload must reach DRAM before the DMA engine fetches them, or the engine
    // reads a posted stale copy.
    fn memFence() void {
        switch (builtin.target.cpu.arch) {
            .riscv64, .riscv32 => asm volatile ("fence rw, rw" ::: .{ .memory = true }),
            else => asm volatile ("" ::: .{ .memory = true }),
        }
    }

    // The ADMA master drives 32-bit addresses, so every buffer it touches must
    // live below 4 GiB. Report the failure rather than truncating the address
    // and letting the engine write over an unrelated page.
    fn phys32(ptr: *const anyopaque) ?u32 {
        const addr = @intFromPtr(ptr);
        if (addr > std.math.maxInt(u32)) return null;
        return @intCast(addr);
    }

    // Point the descriptor at physical address `dst` for `blocks` blocks, and
    // publish it to memory before the engine pre-fetches it at command issue.
    // One descriptor spans the whole transfer, so a multi-block CMD18 read
    // streams every block into `dst`..`dst`+blocks*512 through this single
    // descriptor. `blocks`*512 must fit the 16-bit length field (max 127 blocks).
    fn armDescriptor(self: *HarborSdio, dst: u32, blocks: u16) bool {
        // Direct single-buffer DMA: ADMA_ADDR is the buffer physical address and
        // BLK_COUNT*BLK_SIZE is the length. The controller reads NO descriptor
        // from DRAM, so there is no CPU-store-vs-engine-fetch race on the
        // descriptor (which on silicon let a stale fetch write a chunk to the
        // wrong buffer / the stack). All state goes straight into MMIO registers.
        self.mmio.write(u32, ADMA_ADDR, dst);
        self.mmio.write(u32, BLK_SIZE, BLOCK_LEN);
        self.mmio.write(u32, BLK_COUNT, blocks);
        return true;
    }

    // Read one block from `card_addr` straight into physical address `dst` over
    // ADMA. No cache maintenance here: the caller flushes ONCE for the whole
    // transfer. Single-block DMA is the proven path (a multi-block CMD18 burst
    // deadlocks the fabric under DDR posted-write backpressure).
    fn readOneDma(self: *HarborSdio, dst: u32, card_addr: u32) bool {
        if (!self.armDescriptor(dst, 1)) return false;
        // cmdFlags blocks on STATUS busy, which the controller holds through the
        // data phase, so it returns only once ADMA has streamed the block out.
        _ = self.cmdFlags(READ_SINGLE, card_addr, R_SHORT, F_DATA_READ | F_DMA) orelse return false;
        const status = self.mmio.read(u32, INT_STATUS);
        if (status & I_DATA_DONE == 0) return false; // never finished
        if (status & I_DATA_CRC != 0) return false; // bad CRC16
        self.mmio.write(u32, INT_STATUS, I_ALL);
        return true;
    }

    // Read `n` blocks from `card_addr` into physical `dst` in ONE CMD18
    // multi-block transfer over ADMA. The controller streams all n blocks'
    // words continuously into `dst`..`dst`+n*512 through one descriptor, so the
    // command issue, descriptor fetch, and completion handshake are paid once
    // for the whole chunk instead of per block. That, with the pipelined write
    // path feeding the posted DDR bridge, is what lets the read run at the SD
    // data rate rather than the per-block DDR-write-latency rate.
    //
    // CMD18 is open-ended: the card keeps streaming until CMD12. The controller
    // takes exactly BLK_COUNT (n) blocks then stops sampling, so CMD12 after the
    // data phase stops the card and leaves it idle for the next command.
    fn readChunkDma(self: *HarborSdio, dst: u32, card_addr: u32, n: u16) bool {
        if (!self.armDescriptor(dst, n)) return false;
        // cmdFlags blocks on STATUS busy, which the controller holds through the
        // whole multi-block data phase, so it returns only once ADMA has
        // streamed all n blocks out.
        _ = self.cmdFlags(READ_MULTIPLE, card_addr, R_SHORT, F_DATA_READ | F_DMA) orelse return false;
        const status = self.mmio.read(u32, INT_STATUS);
        self.mmio.write(u32, INT_STATUS, I_ALL);
        // End the open-ended read so the card stops streaming. R1b: the card
        // holds BUSY through the stop, which waitCmd rides out.
        _ = self.cmd(STOP_TRANSMISSION, 0, R_BUSY);
        if (status & I_DATA_DONE == 0) return false; // never finished
        if (status & I_DATA_CRC != 0) return false; // a block failed CRC16
        return true;
    }

    // Write the bounce buffer out as one 512-byte block over ADMA. The engine
    // fetches each word from memory and the DAT serializer stalls the SD clock
    // until the word arrives, so the block goes out gapless.
    fn writeBlockDma(self: *HarborSdio, addr: u32) bool {
        // The payload is already in `bounce`, at offset 0 (see writeBlocksImpl).
        // Publish it before the engine fetches it, pointing the descriptor at the
        // bounce buffer's physical address.
        const bounce_addr = phys32(&self.bounce) orelse return false;
        if (!self.armDescriptor(bounce_addr, 1)) return false;

        _ = self.cmdFlags(WRITE_SINGLE, addr, R_SHORT, F_DATA_WRITE | F_DMA) orelse return false;

        const status = self.mmio.read(u32, INT_STATUS);
        if (status & I_DATA_DONE == 0) return false; // never finished
        if (status & I_WRITE_ERR != 0) return false; // card rejected the block
        self.mmio.write(u32, INT_STATUS, I_ALL);
        return true;
    }

    /// Convert a logical block address to the card's addressing (block for
    /// high-capacity, byte for standard).
    fn cardAddr(self: *const HarborSdio, lba: u64) u32 {
        return if (self.high_capacity)
            @intCast(lba)
        else
            @intCast(lba * BLOCK_LEN);
    }

    fn readBlocksImpl(ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]u8) bool {
        const self: *HarborSdio = @ptrCast(@alignCast(ctx));
        if (!self.present) return false;

        // DMA STRAIGHT INTO the caller's buffer when it is 32-bit-addressable
        // and word-aligned: the bounce buffer's only purpose was one known
        // region to keep coherent, but the CPU-copy out of it costs ~27us/word
        // on the microcode core (a 4 KiB copy measured 28 ms, ~44x the raw
        // hardware transfer). The direct path skips the copy entirely; a whole-
        // cache fence.i (7 us) still invalidates the stale lines, and the
        // read-back proves the engine's writes landed. The bounce + copy stays
        // only as a fallback for an unaligned or above-4-GiB destination.
        var done: u32 = 0;
        var need_flush = false;
        while (done < count) {
            const n = @min(count - done, CHUNK_BLOCKS);
            const out: [*]align(1) u8 = buf + @as(usize, done) * BLOCK_LEN;
            const direct = (@intFromPtr(out) & 3) == 0;
            if (direct) {
                if (phys32(out)) |dst| {
                    if (!self.readChunkDma(dst, self.cardAddr(lba + done), @intCast(n))) return false;
                    // Land THIS chunk's ADMA writes before moving on, with a
                    // read-back of its last written word. The in-order fabric
                    // cannot serve that read until the write burst ahead of it
                    // has landed, so one load makes the whole chunk durable. This
                    // replaces the old hack of DEFERRING a single whole-cache
                    // fence.i to the end of the call: that left the tail chunks'
                    // posted writes un-landed when the caller read a large
                    // transfer, corrupting reads past ~1024 blocks per call. A
                    // read-back is not a fence.i, so it does not flush the CPU's
                    // stack/code and cannot glitch the core mid-stream. The dst
                    // region is DMA-written only (the CPU never cached it), so
                    // the read-back hits DRAM, not a stale line. A single final
                    // fence.i below still invalidates any pre-existing stale
                    // lines for a buffer the caller had touched before.
                    readBarrier(@ptrFromInt(dst + @as(u32, n) * BLOCK_LEN - 4));
                    need_flush = true;
                    done += n;
                    continue;
                }
            }
            // Fallback: bounce + CPU copy (unaligned or high destination). The
            // copy reads the bounce this iteration, so it must flush per chunk.
            const bounce_addr = phys32(&self.bounce) orelse return false;
            if (!self.readChunkDma(bounce_addr, self.cardAddr(lba + done), @intCast(n))) return false;
            cacheFlush();
            @memcpy(out[0 .. @as(usize, n) * BLOCK_LEN], self.bounce[0 .. @as(usize, n) * BLOCK_LEN]);
            done += n;
        }
        if (need_flush) cacheFlush();
        return true;
    }

    fn writeBlocksImpl(ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]const u8) bool {
        const self: *HarborSdio = @ptrCast(@alignCast(ctx));
        if (!self.present) return false;
        var i: u32 = 0;
        while (i < count) : (i += 1) {
            const src: [*]align(1) const u8 = buf + @as(usize, i) * BLOCK_LEN;
            @memcpy(self.bounce[0..BLOCK_LEN], src[0..BLOCK_LEN]);
            if (!self.writeBlockDma(self.cardAddr(lba + i))) return false;
        }
        return true;
    }

    /// Present the card as a generic read/write block device.
    pub fn block(self: *HarborSdio) Block {
        return .{
            .ctx = self,
            .block_size = BLOCK_LEN,
            .num_blocks = self.num_blocks,
            .read_blocks = readBlocksImpl,
            .write_blocks = writeBlocksImpl,
        };
    }

    /// The shared CID accessor type. See device/sd.zig for its field methods.
    pub const Cid = sd.Cid;

    /// The card identity, captured from ALL_SEND_CID (R2) at bring-up. It reads
    /// all zeros (`known()` false) when no card was identified.
    pub fn cidInfo(self: *const HarborSdio) Cid {
        return self.cid;
    }

    pub fn cardPresent(self: *const HarborSdio) bool {
        return self.present;
    }
};

pub fn bind(mmio: Mmio, opts: Options) HarborSdio {
    var dev = HarborSdio{
        .mmio = mmio,
        .clk_freq = opts.freq,
        .bus_width = opts.bus_width,
        .data_hz = opts.data_hz,
    };
    _ = dev.init();
    return dev;
}

// ---------------------------------------------------------------------------
// Host test: drive the driver against a fake controller + card behind a fake
// `Mmio`. The model answers each command the way the real controller does
// (RESP0..3 filled per the response type, INT_STATUS write-1-to-clear) and
// walks the ADMA descriptor for a data phase, so the real HarborSdio -> Mmio
// path runs end to end.
// ---------------------------------------------------------------------------

const FakeHost = struct {
    const S = HarborSdio;

    ctrl: u32 = 0,
    clk_div: u32 = 0,
    cmd_arg: u32 = 0,
    blk_size: u32 = 0,
    blk_count: u32 = 0,
    adma: u32 = 0,
    int_status: u32 = 0,
    resp: [4]u32 = .{ 0, 0, 0, 0 },
    // ACMD41 reports "still powering up" this many times before it is ready, so
    // the driver's poll loop is exercised.
    op_cond_busy: u32 = 2,
    acmd: bool = false,
    /// CSD returned by CMD9, as the four response registers. Defaults to a
    /// CSD v2 (SDHC) with C_SIZE 0x1fff, so (0x1fff + 1) * 1024 blocks.
    csd: [4]u32 = .{ 0, 0x1fff0000, 0, 0x40000000 },
    /// CID returned by ALL_SEND_CID (R2), as the four response registers with
    /// direct alignment. This encodes MID 0x03, OEM "SD", product "SC64G", PRV
    /// 8.0, PSN 0x11223344, made 2021-09.
    cid: [4]u32 = .{ 0x44015900, 0x80112233, 0x43363447, 0x03534453 },
    // The card's one-block backing store, plus a note of what the driver did.
    media: [512]u8 = [_]u8{0} ** 512,
    reads: usize = 0,
    writes: usize = 0,
    last_addr: u32 = 0,
    width_set: u32 = 0,
    // Make one command time out, to check the driver reports the failure.
    timeout_cmd: ?u32 = null,

    fn descriptor(self: *FakeHost) struct { addr: u32, len: u32 } {
        // Direct single-buffer DMA (matches armDescriptor): ADMA_ADDR is the
        // buffer physical address itself and BLK_SIZE*BLK_COUNT is the length.
        // The controller fetches NO descriptor from DRAM, so ADMA_ADDR is not a
        // descriptor pointer to dereference.
        return .{ .addr = self.adma, .len = self.blk_size * self.blk_count };
    }

    fn dataPhase(self: *FakeHost, read: bool) void {
        const d = self.descriptor();
        const buf: [*]u8 = @ptrFromInt(d.addr);
        const n = @min(d.len, self.media.len);
        if (read) {
            @memcpy(buf[0..n], self.media[0..n]);
            self.reads += 1;
        } else {
            @memcpy(self.media[0..n], buf[0..n]);
            self.writes += 1;
        }
        self.int_status |= S.I_DATA_DONE;
    }

    fn onCmd(self: *FakeHost, value: u32) void {
        const index = value & 0x3f;
        const has_data = value & S.F_DATA != 0;
        const is_read = value & S.F_READ != 0;
        self.resp = .{ 0, 0, 0, 0 };
        self.int_status |= S.I_CMD_DONE;

        if (self.timeout_cmd) |t| if (t == index and !self.acmd) {
            self.int_status |= S.I_CMD_TIMEOUT;
            self.acmd = false;
            return;
        };

        const app = self.acmd;
        self.acmd = false;
        if (app) {
            switch (index) {
                S.SD_SEND_OP_COND => {
                    if (self.op_cond_busy > 0) {
                        self.op_cond_busy -= 1;
                        self.resp[0] = 0x00ff8000; // still powering up
                    } else {
                        self.resp[0] = 0xc0ff8000; // ready | CCS (high capacity)
                    }
                },
                S.SET_BUS_WIDTH => self.width_set = self.cmd_arg,
                else => {},
            }
            return;
        }

        switch (index) {
            S.APP_CMD => self.acmd = true,
            S.SEND_IF_COND => self.resp[0] = 0x1aa, // v2, pattern echoed
            S.SEND_REL_ADDR => self.resp[0] = 0x4567 << 16,
            S.SEND_CSD => self.resp = self.csd,
            S.ALL_SEND_CID => self.resp = self.cid,
            else => {},
        }
        if (has_data) {
            // Record the data command's address before any trailing CMD12
            // (STOP_TRANSMISSION, arg 0) overwrites cmd_arg.
            self.last_addr = self.cmd_arg;
            self.dataPhase(is_read);
        }
    }

    fn readFn(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const self: *FakeHost = @ptrCast(@alignCast(ctx));
        return switch (off) {
            S.CTRL => self.ctrl,
            S.STATUS => S.ST_CARD_DETECT, // card in the slot, never busy
            S.CLK_DIV => self.clk_div,
            S.CMD_ARG => self.cmd_arg,
            S.RESP0 => self.resp[0],
            S.RESP0 + 8 => self.resp[1],
            S.RESP0 + 16 => self.resp[2],
            S.RESP0 + 24 => self.resp[3],
            S.BLK_SIZE => self.blk_size,
            S.BLK_COUNT => self.blk_count,
            S.INT_STATUS => self.int_status,
            S.ADMA_ADDR => self.adma,
            else => 0,
        };
    }

    fn writeFn(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const self: *FakeHost = @ptrCast(@alignCast(ctx));
        const v: u32 = @truncate(val);
        switch (off) {
            S.CTRL => self.ctrl = v,
            S.CLK_DIV => self.clk_div = v,
            S.CMD_ARG => self.cmd_arg = v,
            S.CMD => self.onCmd(v),
            S.BLK_SIZE => self.blk_size = v,
            S.BLK_COUNT => self.blk_count = v,
            S.INT_STATUS => self.int_status &= ~v, // write-1-to-clear
            S.ADMA_ADDR => self.adma = v,
            else => {},
        }
    }

    fn mmio(self: *FakeHost) Mmio {
        return .{
            .ctx = self,
            .base = 0,
            .read_fn = readFn,
            .write_fn = writeFn,
        };
    }
};

// The ADMA engine reaches 32 bits, and the driver's descriptor and bounce
// buffer live inside the device struct. On the target that struct sits in
// identity-mapped DRAM below 4 GiB; a 64-bit test host puts it far above. Map a
// low page and build the instance there, so the DMA path runs for real instead
// of being rejected by the reach check. Returns null when the host cannot give
// out that address, and the caller then skips.
fn lowInstance() ?*HarborSdio {
    if (@sizeOf(usize) <= 4) return null; // any address already fits
    if (builtin.os.tag != .linux) return null;
    const size = (@sizeOf(HarborSdio) + 0xffff) & ~@as(usize, 0xffff);
    const mem = std.posix.mmap(
        @ptrFromInt(0x3000_0000),
        size,
        .{ .READ = true, .WRITE = true },
        .{ .TYPE = .PRIVATE, .ANONYMOUS = true, .FIXED_NOREPLACE = true },
        -1,
        0,
    ) catch return null;
    if (@intFromPtr(mem.ptr) > std.math.maxInt(u32)) {
        std.posix.munmap(mem);
        return null;
    }
    return @ptrCast(@alignCast(mem.ptr));
}

test "harbor_sdio identifies an SDHC card, then reads and writes a block" {
    var host = FakeHost{};
    for (&host.media, 0..) |*b, i| b.* = @truncate(i);

    const dev = lowInstance() orelse return error.SkipZigTest;
    dev.* = bind(host.mmio(), .{ .freq = 50_000_000 });
    try std.testing.expect(dev.cardPresent());
    try std.testing.expect(dev.high_capacity);
    try std.testing.expectEqual(@as(u32, 0x4567), dev.rca);
    try std.testing.expectEqual(@as(u64, 8388608), dev.num_blocks); // 4 GiB

    // CID from the ALL_SEND_CID R2 response, decoded through the shared parser.
    const cid = dev.cidInfo();
    try std.testing.expect(cid.known());
    try std.testing.expectEqual(@as(u8, 0x03), cid.manufacturerId());
    try std.testing.expectEqualSlices(u8, "SD", &cid.oemId());
    try std.testing.expectEqualSlices(u8, "SC64G", &cid.productName());
    try std.testing.expectEqual(@as(u8, 8), cid.revision().major);
    try std.testing.expectEqual(@as(u32, 0x11223344), cid.serialNumber());
    try std.testing.expectEqual(@as(u16, 2021), cid.manufactureDate().year);
    try std.testing.expectEqual(@as(u8, 9), cid.manufactureDate().month);

    // ACMD6 asked the card for 4-bit, and CTRL[5:4] made the host match.
    try std.testing.expectEqual(@as(u32, 2), host.width_set);
    try std.testing.expectEqual(
        HarborSdio.CTRL_ENABLE | (@as(u32, 1) << 4),
        host.ctrl,
    );
    // 50 MHz / (2 * 25 MHz) - 1 = 0.
    try std.testing.expectEqual(@as(u32, 0), host.clk_div);

    const blk = dev.block();
    try std.testing.expect(blk.writable());
    try std.testing.expectEqual(@as(u32, 512), blk.block_size);

    var buf: [512]u8 = undefined;
    try std.testing.expect(blk.readBlocks(7, 1, &buf));
    try std.testing.expectEqual(@as(usize, 1), host.reads);
    try std.testing.expectEqualSlices(u8, host.media[0..], buf[0..]);

    // A high-capacity card is block-addressed, so the argument is the LBA.
    try std.testing.expectEqual(@as(u32, 7), host.last_addr);

    var out: [512]u8 = undefined;
    for (&out, 0..) |*b, i| b.* = @truncate(i ^ 0xa5); // a distinct pattern
    try std.testing.expect(blk.writeBlocks(3, 1, &out));
    try std.testing.expectEqual(@as(usize, 1), host.writes);
    try std.testing.expectEqualSlices(u8, &out, host.media[0..]);
    try std.testing.expectEqual(@as(u32, 3), host.last_addr);
}

test "harbor_sdio refuses a transfer the DMA engine cannot reach" {
    if (@sizeOf(usize) <= 4) return error.SkipZigTest; // every address fits
    var host = FakeHost{};
    // On a 64-bit host this instance sits far above 4 GiB, which is exactly the
    // case the reach check exists for: report the failure rather than truncate
    // the address and let the engine write over an unrelated page.
    var dev = bind(host.mmio(), .{ .freq = 50_000_000 });
    try std.testing.expect(dev.cardPresent());

    const blk = dev.block();
    var buf: [512]u8 = undefined;
    try std.testing.expect(!blk.readBlocks(0, 1, &buf));
    try std.testing.expectEqual(@as(usize, 0), host.reads); // never issued
}

test "harbor_sdio runs a 1-bit card and leaves the bus narrow" {
    var host = FakeHost{};
    var dev = bind(host.mmio(), .{ .freq = 50_000_000, .bus_width = .one });
    try std.testing.expect(dev.cardPresent());
    try std.testing.expectEqual(@as(u32, 0), host.width_set); // no ACMD6
    try std.testing.expectEqual(HarborSdio.CTRL_ENABLE, host.ctrl);
}

test "harbor_sdio reports a card that never answers CMD8/ACMD41" {
    var host = FakeHost{};
    host.timeout_cmd = HarborSdio.APP_CMD; // CMD55 always times out
    var dev = bind(host.mmio(), .{ .freq = 50_000_000 });
    try std.testing.expect(!dev.cardPresent());

    const blk = dev.block();
    try std.testing.expectEqual(@as(u64, 0), blk.num_blocks);
    var buf: [512]u8 = undefined;
    try std.testing.expect(!blk.readBlocks(0, 1, &buf));
    try std.testing.expect(!blk.writeBlocks(0, 1, &buf));
}

test "harbor_sdio derives the SD clock divider from the input clock" {
    var host = FakeHost{};
    var dev = HarborSdio{ .mmio = host.mmio(), .clk_freq = 100_000_000 };
    dev.setClock(400_000); // 100e6 / 800e3 - 1
    try std.testing.expectEqual(@as(u32, 124), host.clk_div);
    dev.setClock(25_000_000); // 100e6 / 50e6 - 1
    try std.testing.expectEqual(@as(u32, 1), host.clk_div);
    // A rate at or above half the input clock cannot divide further.
    dev.setClock(100_000_000);
    try std.testing.expectEqual(@as(u32, 0), host.clk_div);
}

test "harbor_sdio decodes a CSD v1 (SDSC) capacity" {
    var host = FakeHost{};
    var dev = HarborSdio{ .mmio = host.mmio() };
    // A 1 GiB SDSC: READ_BL_LEN 10 (1024 B), C_SIZE 3963, C_SIZE_MULT 7.
    // Blocks = (3963+1) * 2^9 * 1024 / 512 = 4059136.
    host.csd = .{
        0, // RESP0: CSD[31:0], not used by the capacity fields
        (@as(u32, 3963 & 0x3) << 30) | (7 << 15), // C_SIZE[1:0], C_SIZE_MULT
        (10 << 16) | (3963 >> 2), // READ_BL_LEN, C_SIZE[11:2]
        0, // CSD_STRUCTURE = 0
    };
    try std.testing.expectEqual(@as(u64, 4059136), dev.readCapacity());
}
