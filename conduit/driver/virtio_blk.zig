//! virtio-blk over the virtio-mmio transport (modern, v2), implementing the
//! `Block` contract. Ported from Weir's virtio/blk.zig: feature handshake, one
//! split virtqueue, polled completion. Made instance-based (the virtqueue lives
//! in the struct) and transport-agnostic (the slot is found by discovery, not
//! by scanning a fixed address window).
//!
//! IMPORTANT: the device DMAs into this struct's embedded virtqueue, so the
//! struct must live at a stable address before the queue is programmed. Hence
//! the two-step API: `bind` fills fields only; `start` performs the handshake
//! and programs the queue against the final `*Virtio`. Call `start` once the
//! value is in its permanent home. DMA uses `@intFromPtr` (identity-mapped, as
//! in Weir's M-mode boot); a paged host must place the struct and read buffers
//! in DMA-coherent memory.

const builtin = @import("builtin");
const Mmio = @import("../mmio.zig");
const Block = @import("../device/block.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .block,
    .dt_compatible = &.{"virtio,mmio"},
    .driver = "virtio_blk",
};

const MAGIC: u32 = 0x74726976; // 'virt'
const DEVICE_ID_BLOCK: u32 = 2;

const R_MAGIC = 0x000;
const R_VERSION = 0x004;
const R_DEVICE_ID = 0x008;
const R_DRIVER_FEATURES = 0x020;
const R_DRIVER_FEATURES_SEL = 0x024;
const R_QUEUE_SEL = 0x030;
const R_QUEUE_NUM_MAX = 0x034;
const R_QUEUE_NUM = 0x038;
const R_QUEUE_READY = 0x044;
const R_QUEUE_NOTIFY = 0x050;
const R_INTERRUPT_STATUS = 0x060;
const R_INTERRUPT_ACK = 0x064;
const R_STATUS = 0x070;
const R_QUEUE_DESC_LOW = 0x080;
const R_QUEUE_DRIVER_LOW = 0x090;
const R_QUEUE_DEVICE_LOW = 0x0a0;
const R_CONFIG = 0x100;

const S_ACKNOWLEDGE: u32 = 1;
const S_DRIVER: u32 = 2;
const S_DRIVER_OK: u32 = 4;
const S_FEATURES_OK: u32 = 8;

const F_VERSION_1_HI: u32 = 1; // VIRTIO_F_VERSION_1 = bit 32

const QSIZE = 8;
const VIRTQ_DESC_F_NEXT: u16 = 1;
const VIRTQ_DESC_F_WRITE: u16 = 2;
const VIRTIO_BLK_T_IN: u32 = 0; // read
const CHUNK: u32 = 128; // max sectors per request

const Desc = extern struct { addr: u64, len: u32, flags: u16, next: u16 };
const Avail = extern struct { flags: u16, idx: u16, ring: [QSIZE]u16, used_event: u16 };
const UsedElem = extern struct { id: u32, len: u32 };
const Used = extern struct { flags: u16, idx: u16, ring: [QSIZE]UsedElem, avail_event: u16 };
const ReqHdr = extern struct { type: u32, reserved: u32, sector: u64 };

pub const Virtio = struct {
    mmio: Mmio,
    present: bool = false,
    last_used: u16 = 0,

    desc: [QSIZE]Desc align(16) = undefined,
    avail: Avail align(16) = undefined,
    used: Used align(16) = undefined,
    req_hdr: ReqHdr align(16) = undefined,
    req_status: u8 align(16) = undefined,

    /// Run the virtio handshake and program the queue. Returns true if a v2
    /// block device is ready. Must be called on the struct's final address.
    pub fn start(self: *Virtio) bool {
        if (self.mmio.read(u32, R_MAGIC) != MAGIC) return false;
        if (self.mmio.read(u32, R_DEVICE_ID) != DEVICE_ID_BLOCK) return false;
        if (self.mmio.read(u32, R_VERSION) != 2) return false; // modern only

        self.mmio.write(u32, R_STATUS, 0); // reset
        var status: u32 = S_ACKNOWLEDGE;
        self.mmio.write(u32, R_STATUS, status);
        status |= S_DRIVER;
        self.mmio.write(u32, R_STATUS, status);

        // Accept exactly VIRTIO_F_VERSION_1; decline device-specific features.
        self.mmio.write(u32, R_DRIVER_FEATURES_SEL, 1);
        self.mmio.write(u32, R_DRIVER_FEATURES, F_VERSION_1_HI);
        self.mmio.write(u32, R_DRIVER_FEATURES_SEL, 0);
        self.mmio.write(u32, R_DRIVER_FEATURES, 0);

        status |= S_FEATURES_OK;
        self.mmio.write(u32, R_STATUS, status);
        if (self.mmio.read(u32, R_STATUS) & S_FEATURES_OK == 0) return false;

        self.mmio.write(u32, R_QUEUE_SEL, 0);
        if (self.mmio.read(u32, R_QUEUE_NUM_MAX) < QSIZE) return false;
        self.mmio.write(u32, R_QUEUE_NUM, QSIZE);
        self.setQueueAddr(R_QUEUE_DESC_LOW, @intFromPtr(&self.desc));
        self.setQueueAddr(R_QUEUE_DRIVER_LOW, @intFromPtr(&self.avail));
        self.setQueueAddr(R_QUEUE_DEVICE_LOW, @intFromPtr(&self.used));
        self.mmio.write(u32, R_QUEUE_READY, 1);

        status |= S_DRIVER_OK;
        self.mmio.write(u32, R_STATUS, status);
        self.present = true;
        return true;
    }

    fn setQueueAddr(self: *Virtio, off: usize, addr: usize) void {
        self.mmio.write(u32, off, @truncate(addr));
        self.mmio.write(u32, off + 4, @truncate(addr >> 32));
    }

    /// Disk capacity in 512-byte sectors.
    pub fn capacity(self: *Virtio) u64 {
        const lo = self.mmio.read(u32, R_CONFIG);
        const hi = self.mmio.read(u32, R_CONFIG + 4);
        return @as(u64, lo) | (@as(u64, hi) << 32);
    }

    fn readChunk(self: *Virtio, start_sector: u64, count: u32, buf: [*]u8) bool {
        self.req_hdr = .{ .type = VIRTIO_BLK_T_IN, .reserved = 0, .sector = start_sector };
        self.req_status = 0xff;

        self.desc[0] = .{ .addr = @intFromPtr(&self.req_hdr), .len = @sizeOf(ReqHdr), .flags = VIRTQ_DESC_F_NEXT, .next = 1 };
        self.desc[1] = .{ .addr = @intFromPtr(buf), .len = count * 512, .flags = VIRTQ_DESC_F_NEXT | VIRTQ_DESC_F_WRITE, .next = 2 };
        self.desc[2] = .{ .addr = @intFromPtr(&self.req_status), .len = 1, .flags = VIRTQ_DESC_F_WRITE, .next = 0 };

        self.avail.ring[self.avail.idx % QSIZE] = 0;
        barrier();
        self.avail.idx +%= 1;
        barrier();
        self.mmio.write(u32, R_QUEUE_NOTIFY, 0);

        while (@atomicLoad(u16, &self.used.idx, .acquire) == self.last_used) {
            asm volatile ("" ::: .{ .memory = true });
        }
        self.last_used = self.used.idx;
        self.mmio.write(u32, R_INTERRUPT_ACK, self.mmio.read(u32, R_INTERRUPT_STATUS));

        return self.req_status == 0;
    }

    fn readBlocksImpl(ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]u8) bool {
        const self: *Virtio = @ptrCast(@alignCast(ctx));
        if (!self.present) return false;
        var done: u32 = 0;
        while (done < count) {
            const n: u32 = @min(CHUNK, count - done);
            if (!self.readChunk(lba + done, n, buf + @as(usize, done) * 512)) return false;
            done += n;
        }
        return true;
    }

    /// Present the disk as a generic block device.
    pub fn block(self: *Virtio) Block {
        return .{
            .ctx = self,
            .block_size = 512,
            .num_blocks = self.capacity(),
            .read_blocks = readBlocksImpl,
        };
    }
};

/// Fill a `Virtio` over `mmio`. Call `start` once it is at its final address.
pub fn bind(mmio: Mmio) Virtio {
    return .{ .mmio = mmio };
}

/// A memory barrier ordering virtqueue writes against the device. Comptime-
/// selected per arch so conduit compiles for any target.
fn barrier() void {
    switch (builtin.cpu.arch) {
        .riscv64, .riscv32 => asm volatile ("fence" ::: .{ .memory = true }),
        .aarch64 => asm volatile ("dsb sy" ::: .{ .memory = true }),
        .x86_64, .x86 => asm volatile ("mfence" ::: .{ .memory = true }),
        else => asm volatile ("" ::: .{ .memory = true }),
    }
}
