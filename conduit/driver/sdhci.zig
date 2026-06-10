//! Harbor SD/MMC host driver, over an `Mmio`, implementing the `Block` contract.
//! Ported from Weir's drivers/sdhci.zig (made instance-based: the card state
//! lives in the struct, not module globals). Polled PIO; runs the SD init
//! sequence so a card can serve as a block device.
//!   0x00 CTRL, 0x04 STATUS, 0x08 CLK_DIV, 0x0C CMD, 0x10 CMD_ARG,
//!   0x14.. RESP0..3, 0x24 DATA FIFO, 0x28 BLK_SIZE, 0x2C BLK_COUNT.

const Mmio = @import("../mmio.zig");
const Block = @import("../device/block.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .block,
    .dt_compatible = &.{ "midstall,harbor-sdhci", "harbor,sdhci" },
    .driver = "sdhci",
};

pub const Options = struct {
    /// Controller input clock in Hz, used to derive the SD clock divider.
    freq: u32 = 0,
};

pub const Sdhci = struct {
    mmio: Mmio,
    clk_freq: u32 = 0,
    rca: u32 = 0,
    high_capacity: bool = false,
    num_blocks: u64 = 0,
    present: bool = false,

    const CTRL = 0x00;
    const STATUS = 0x04;
    const CLK_DIV = 0x08;
    const CMD = 0x0c;
    const CMD_ARG = 0x10;
    const RESP0 = 0x14;
    const DATA = 0x24;
    const BLK_SIZE = 0x28;
    const BLK_COUNT = 0x2c;

    const ST_CARD_DETECT: u32 = 1 << 0;
    const ST_BUSY: u32 = 1 << 8;

    const GO_IDLE = 0;
    const ALL_SEND_CID = 2;
    const SEND_REL_ADDR = 3;
    const SELECT_CARD = 7;
    const SEND_IF_COND = 8;
    const SEND_CSD = 9;
    const SET_BLOCKLEN = 16;
    const READ_SINGLE = 17;
    const APP_CMD = 55;
    const SD_SEND_OP_COND = 41; // ACMD41

    const BLOCK_LEN = 512;

    fn waitCmd(self: *Sdhci) bool {
        var t: usize = 1_000_000;
        while (self.mmio.read(u32, STATUS) & ST_BUSY != 0) {
            t -= 1;
            if (t == 0) return false;
        }
        return true;
    }

    fn cmd(self: *Sdhci, opcode: u32, arg: u32) ?[4]u32 {
        self.mmio.write(u32, CMD_ARG, arg);
        self.mmio.write(u32, CMD, opcode & 0x3f);
        if (!self.waitCmd()) return null;
        return .{
            self.mmio.read(u32, RESP0),
            self.mmio.read(u32, RESP0 + 4),
            self.mmio.read(u32, RESP0 + 8),
            self.mmio.read(u32, RESP0 + 12),
        };
    }

    fn setClock(self: *Sdhci, hz: u32) void {
        if (hz != 0 and self.clk_freq != 0) self.mmio.write(u32, CLK_DIV, self.clk_freq / (2 * hz) - 1);
    }

    /// Bring up the host and initialize the inserted card. Returns true on success.
    pub fn init(self: *Sdhci) bool {
        self.present = false;
        self.mmio.write(u32, CTRL, 1); // power on
        self.setClock(400_000); // identification clock

        if (self.mmio.read(u32, STATUS) & ST_CARD_DETECT == 0) return false; // no card

        _ = self.cmd(GO_IDLE, 0) orelse return false;

        const if_cond = self.cmd(SEND_IF_COND, 0x1aa);
        const v2 = if (if_cond) |c| (c[0] & 0xff) == 0xaa else false;

        const hcs: u32 = if (v2) 0x40000000 else 0;
        var tries: usize = 100_000;
        var ocr: u32 = 0;
        while (tries > 0) : (tries -= 1) {
            _ = self.cmd(APP_CMD, 0) orelse return false;
            const resp = self.cmd(SD_SEND_OP_COND, hcs | 0x00ff8000) orelse return false;
            ocr = resp[0];
            if (ocr & 0x80000000 != 0) break;
        }
        if (ocr & 0x80000000 == 0) return false;
        self.high_capacity = ocr & 0x40000000 != 0;

        _ = self.cmd(ALL_SEND_CID, 0) orelse return false;
        const rel = self.cmd(SEND_REL_ADDR, 0) orelse return false;
        self.rca = rel[0] >> 16;
        _ = self.cmd(SELECT_CARD, self.rca << 16) orelse return false;

        if (!self.high_capacity) _ = self.cmd(SET_BLOCKLEN, BLOCK_LEN) orelse return false;

        self.setClock(25_000_000); // transfer clock
        self.num_blocks = self.readCapacity();
        self.present = true;
        return true;
    }

    /// Card capacity in 512-byte blocks from the CSD (CMD9, R2). CSD v1 parsing
    /// is omitted (capacity reported as unknown), as in the Weir original.
    fn readCapacity(self: *Sdhci) u64 {
        const csd = self.cmd(SEND_CSD, self.rca << 16) orelse return 0;
        const csd_structure = (csd[3] >> 22) & 0x3;
        if (csd_structure == 1) {
            const c_size: u64 = ((@as(u64, csd[2]) & 0x3f) << 16) | (csd[1] >> 16);
            return (c_size + 1) * 1024;
        }
        return 0;
    }

    fn readBlocksImpl(ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]u8) bool {
        const self: *Sdhci = @ptrCast(@alignCast(ctx));
        if (!self.present) return false;
        var i: u32 = 0;
        while (i < count) : (i += 1) {
            const addr: u32 = if (self.high_capacity) @intCast(lba + i) else @intCast((lba + i) * BLOCK_LEN);
            self.mmio.write(u32, BLK_SIZE, BLOCK_LEN);
            self.mmio.write(u32, BLK_COUNT, 1);
            _ = self.cmd(READ_SINGLE, addr) orelse return false;
            const out: [*]align(1) u8 = buf + @as(usize, i) * BLOCK_LEN;
            var j: usize = 0;
            while (j < BLOCK_LEN / 4) : (j += 1) {
                const word = self.mmio.read(u32, DATA);
                const dst: *align(1) u32 = @ptrCast(out + j * 4);
                dst.* = word;
            }
        }
        return true;
    }

    /// Present the card as a generic block device.
    pub fn block(self: *Sdhci) Block {
        return .{
            .ctx = self,
            .block_size = BLOCK_LEN,
            .num_blocks = self.num_blocks,
            .read_blocks = readBlocksImpl,
        };
    }

    pub fn cardPresent(self: *const Sdhci) bool {
        return self.present;
    }
};

pub fn bind(mmio: Mmio, opts: Options) Sdhci {
    var dev = Sdhci{ .mmio = mmio, .clk_freq = opts.freq };
    _ = dev.init();
    return dev;
}
