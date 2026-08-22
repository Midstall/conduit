//! Harbor SPI master driver, over an `Mmio`. Full-duplex, one byte per polled
//! transfer. Each register sits in its own 64-bit slot, so a 32-bit access lands
//! in the low word on any fabric width:
//!   0x00 CTRL, 0x08 STATUS, 0x10 DATA, 0x18 DIVIDER, 0x20 CS.
//! A controller built with DMA adds a block engine (bind with `dma = true`):
//!   0x28 DMA_ADDR, 0x30 DMA_LEN, 0x38 DMA_CTRL, plus STATUS DMA_BUSY/DMA_DONE.

const builtin = @import("builtin");
const Mmio = @import("../mmio.zig");
const Spi = @import("../device/spi.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .spi,
    .dt_compatible = &.{ "midstall,harbor-spi", "harbor,spi" },
    .driver = "harbor_spi",
};

pub const Options = struct {
    /// Clock divider. SCK is the input clock over 2 * (divider + 1). 0 leaves
    /// the register at its reset value.
    divider: u32 = 0,
    /// SPI mode 0..3 (CPOL/CPHA).
    mode: u2 = 0,
    /// Chip-select line this device sits on. A controller built with more than
    /// one CS drives one bit per line.
    cs_line: u5 = 0,
    /// Loop MOSI back to MISO inside the controller, for a self-test with no
    /// device attached.
    loopback: bool = false,
    /// The controller has the DMA block engine. Set from the hardware
    /// description (the `harbor,spi` DMA property). Enables the DMA read path.
    dma: bool = false,
};

/// The result of a DMA transfer attempt, so the caller can fall back to a polled
/// transfer when the engine is absent or the buffer does not fit its rules.
pub const DmaResult = enum {
    /// No DMA engine, or the buffer breaks the alignment/length rules. The caller
    /// must run the polled path.
    unavailable,
    /// The engine moved the data. It is in the buffer.
    done,
    /// The engine was used but did not finish. The transfer failed.
    failed,
};

pub const HarborSpi = struct {
    mmio: Mmio,
    dma: bool = false,
    /// Chip-select line this device sits on (see `Options.cs_line`).
    cs_line: u5 = 0,

    // Each register in its own 64-bit-aligned slot (see HarborSpiController), so
    // a 32-bit access lands in the low word on any fabric width.
    const CTRL = 0x00;
    const STATUS = 0x08;
    const DATA = 0x10;
    const DIVIDER = 0x18;
    const CS = 0x20;
    const DMA_ADDR = 0x28;
    const DMA_LEN = 0x30;
    const DMA_CTRL = 0x38;

    const CTRL_ENABLE: u32 = 1 << 0;
    const CTRL_CPOL: u32 = 1 << 1;
    const CTRL_CPHA: u32 = 1 << 2;
    const CTRL_LOOPBACK: u32 = 1 << 3;
    const ST_BUSY: u32 = 1 << 0;
    const ST_TX_EMPTY: u32 = 1 << 1;
    const ST_RX_READY: u32 = 1 << 2;
    const ST_DMA_BUSY: u32 = 1 << 3;
    const ST_DMA_DONE: u32 = 1 << 4; // write-1-to-clear
    const DMA_START: u32 = 1 << 0; // self-clearing
    // DMA_CTRL bit 1 selects the direction, but the controller implements only
    // SD -> mem: it latches the bit and never reads it, so a transfer started
    // with it set still clocks 0xFF out and writes the result over memory. The
    // constant is deliberately absent so no caller can reach for it.

    // A byte transfer clears BUSY within 8 SPI clocks. This cap is far above that
    // at any divider, so a normal transfer never reaches it. A controller that
    // never clears BUSY (unclocked, unpowered, or wedged) hits the cap and the
    // transfer gives up, rather than hanging the machine on the poll.
    const BUSY_POLL_MAX: u32 = 1_000_000;

    /// Assert (true) or release (false) this device's chip select. The register
    /// holds one active-high bit per line, which the controller inverts onto the
    /// active-low CS_N pads. Writing the whole register releases every other
    /// line, which is what a single-device bus wants and what the SD sequence
    /// relies on.
    pub fn chipSelect(self: HarborSpi, active: bool) void {
        self.mmio.write(u32, CS, if (active) @as(u32, 1) << self.cs_line else 0);
    }

    // Wait for the current byte to finish. Bounded: a controller that never
    // clears BUSY returns after the cap instead of hanging. On that timeout the
    // caller's next DATA read is a stale or idle byte, which the SD layer sees as
    // a bad token and fails the transfer through its own bounds.
    fn waitIdle(self: HarborSpi) void {
        var i: u32 = 0;
        while (i < BUSY_POLL_MAX) : (i += 1) {
            if (self.mmio.read(u32, STATUS) & ST_BUSY == 0) return;
        }
    }

    pub fn transferByte(self: HarborSpi, out: u8) u8 {
        self.mmio.write(u32, DATA, out);
        self.waitIdle();
        return @truncate(self.mmio.read(u32, DATA));
    }

    // Flush the whole cache to keep DMA and the CPU coherent. On River any fence
    // flushes the entire D+I cache and the fetch TLB, so there is no line-address
    // math. Plain riscv `fence.i` is only an I-fence, so this leans on that River
    // behaviour, which is safe because the DMA engine is River (`harbor,spi`).
    fn cacheFlush() void {
        switch (builtin.target.cpu.arch) {
            .riscv64, .riscv32 => asm volatile ("fence.i" ::: .{ .memory = true }),
            else => asm volatile ("" ::: .{ .memory = true }),
        }
    }

    // Spin `n` times with a memory barrier so the loop is not optimised away.
    // Used to let a posted DMA write drain to DRAM and a cache invalidate finish
    // before the caller reads the buffer.
    fn drainCycles(n: u32) void {
        var i: u32 = 0;
        while (i < n) : (i += 1) asm volatile ("" ::: .{ .memory = true });
    }

    // The engine writes full bus-width beats, so on a 64-bit fabric the base must
    // be 8-byte aligned and the length a multiple of 8. A 512-byte SD block meets
    // both. A shorter, non-multiple length would zero-pad its final beat, so the
    // caller must not expect a clean tail from one.
    const DMA_ALIGN = 8;

    /// Read `buf.len` bytes from the SPI stream into `buf` through the DMA engine,
    /// clocking 0xFF out (DIR = SD -> mem). Returns `unavailable` when there is no
    /// engine or `buf` breaks the hardware rules (8-byte-aligned base, length a
    /// multiple of 8), so the caller runs the polled path. Chip select must be
    /// asserted, as for the byte loop this replaces.
    pub fn readDma(self: HarborSpi, buf: []u8) DmaResult {
        if (!self.dma) return .unavailable;
        const addr = @intFromPtr(buf.ptr);
        if (buf.len == 0 or buf.len % DMA_ALIGN != 0 or addr % DMA_ALIGN != 0) {
            return .unavailable;
        }

        // No pre-flush. River's D-cache is write-through, so buf has no dirty
        // lines for the engine to race, and cacheFlush() here is a full I-cache
        // flush (fence.i) that would make a caller's read loop re-fetch its code
        // every block. The post-flush below is the one that matters.

        // Write the address at pointer width. Each register sits in its own
        // 64-bit slot, so this lands in the low word on hardware, and it keeps the
        // full address on a 64-bit host where the buffer sits above 4 GiB.
        self.mmio.write(usize, DMA_ADDR, addr);
        self.mmio.write(u32, DMA_LEN, @intCast(buf.len));
        self.mmio.write(u32, DMA_CTRL, DMA_START); // DIR clear: SD -> mem

        // Gate completion on DMA_BUSY, not DMA_DONE. DMA_DONE can stay set from a
        // prior transfer (its write-1-to-clear does not always land), so polling
        // it races: a later read sees the stale DONE and exits before its own
        // transfer finishes, returning an unwritten buffer. DMA_BUSY is set while
        // the engine runs and clears when the last beat lands, so wait for it to
        // assert (the engine took the job) then clear (the transfer finished).
        var i: u32 = 0;
        while (i < BUSY_POLL_MAX and self.mmio.read(u32, STATUS) & ST_DMA_BUSY == 0) : (i += 1) {}
        if (self.mmio.read(u32, STATUS) & ST_DMA_BUSY == 0) {
            self.mmio.write(u32, STATUS, ST_DMA_DONE);
            return .failed; // never started
        }
        i = 0;
        while (i < BUSY_POLL_MAX and self.mmio.read(u32, STATUS) & ST_DMA_BUSY != 0) : (i += 1) {}
        if (self.mmio.read(u32, STATUS) & ST_DMA_BUSY != 0) {
            self.mmio.write(u32, STATUS, ST_DMA_DONE);
            return .failed; // never finished
        }
        self.mmio.write(u32, STATUS, ST_DMA_DONE); // best-effort clear

        // Coherency: a tight sequence of DMA reads queues posted writes faster
        // than the DDR controller commits them, so let them drain, invalidate
        // the stale cache lines, and drain once more before the caller reads.
        drainCycles(8000);
        cacheFlush();
        drainCycles(8000);
        cacheFlush();
        return .done;
    }

    /// Full-duplex transfer. The exchange runs for max(tx, rx) bytes. Past the
    /// end of `tx` the driver clocks 0xFF, which holds MOSI idle high: a device
    /// that watches the bus for a command reads 0xFF as no command, where a zero
    /// byte can start one. The driver fills `rx` only within its bounds. The
    /// driver asserts chip select around the exchange.
    pub fn transfer(self: HarborSpi, tx: []const u8, rx: []u8) void {
        const len = @max(tx.len, rx.len);
        self.chipSelect(true);
        var i: usize = 0;
        while (i < len) : (i += 1) {
            const out: u8 = if (i < tx.len) tx[i] else 0xff;
            const in = self.transferByte(out);
            if (i < rx.len) rx[i] = in;
        }
        self.chipSelect(false);
    }

    pub fn spi(self: *HarborSpi) Spi {
        return Spi.from(self);
    }
};

pub fn bind(mmio: Mmio, opts: Options) HarborSpi {
    const dev = HarborSpi{
        .mmio = mmio,
        .dma = opts.dma,
        .cs_line = opts.cs_line,
    };
    if (opts.divider != 0) mmio.write(u32, HarborSpi.DIVIDER, opts.divider);
    // SPI mode: bit 0 is CPHA, bit 1 is CPOL.
    var ctrl: u32 = HarborSpi.CTRL_ENABLE;
    if (opts.mode & 0b01 != 0) ctrl |= HarborSpi.CTRL_CPHA;
    if (opts.mode & 0b10 != 0) ctrl |= HarborSpi.CTRL_CPOL;
    if (opts.loopback) ctrl |= HarborSpi.CTRL_LOOPBACK;
    mmio.write(u32, HarborSpi.CTRL, ctrl);
    // Release every chip select, so a warm restart does not leave a device
    // selected from the previous run.
    mmio.write(u32, HarborSpi.CS, 0);
    return dev;
}
