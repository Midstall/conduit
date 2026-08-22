//! virtio-gpu 2D over the virtio-mmio transport (modern, v2). It brings up the
//! device and queries the scanout. It creates a host 2D resource. A
//! caller-supplied guest framebuffer backs that resource. It binds the resource
//! to scanout 0. To present, it transfers the framebuffer to the host resource
//! and flushes.
//!
//! The transport mirrors `virtio_blk.zig`. The transport covers the feature
//! handshake, one split virtqueue, polled completion, `setQueueAddr` for
//! QUEUE_DESC/DRIVER/DEVICE, and the notify. Only the queue index (the control
//! queue) and the command payloads differ. The protocol is the ctrl_hdr plus the
//! 2D command structs. It comes from Ferrite's working virtio-gpu driver.
//!
//! IMPORTANT: the device DMAs into this struct's embedded virtqueue and into
//! the command request/response buffers. So the struct must live at a stable
//! address before you program the queue. The API has two steps for this reason.
//! `bind` fills fields only. `start` performs the handshake and programs the
//! queue against the final `*Virtio`. Call `start` once the value is in its
//! permanent home. DMA uses `@intFromPtr` (identity-mapped, as in Weir's M-mode
//! boot). A paged host must place the struct and the framebuffer in DMA-coherent
//! memory.
//!
//! The framebuffer is B8G8R8X8 (little-endian u32 0x00RRGGBB), stride = w*4.

const builtin = @import("builtin");
const Mmio = @import("../mmio.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    // virtio-gpu is a display device, but conduit has no .display class yet. So
    // discovery uses the same virtio,mmio compatible the device exposes. A
    // consumer narrows to GPU by reading R_DEVICE_ID after binding.
    .class = .pci,
    .dt_compatible = &.{"virtio,mmio"},
    .driver = "virtio_gpu",
};

const MAGIC: u32 = 0x74726976; // 'virt'
const DEVICE_ID_GPU: u32 = 16;

const R_MAGIC = 0x000;
const R_VERSION = 0x004;
const R_DEVICE_ID = 0x008;
const R_DEVICE_FEATURES = 0x010;
const R_DEVICE_FEATURES_SEL = 0x014;
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
// VIRTIO_GPU_F_VIRGL = feature bit 0 (low dword). The driver accepts it to
// expose the 3D command set (CTX_CREATE / RESOURCE_CREATE_3D / SUBMIT_3D). This
// needs a virglrenderer-backed device (QEMU `-device virtio-gpu-gl-device`).
const F_VIRGL_LO: u32 = 1 << 0;

// The control queue is queue 0. This driver does not use the cursor queue (1).
const VQ_CONTROL: u32 = 0;
const QSIZE = 8;
const VIRTQ_DESC_F_NEXT: u16 = 1;
const VIRTQ_DESC_F_WRITE: u16 = 2;

// virtio-gpu control commands (virtio 1.2 §5.7.6).
const CMD_GET_DISPLAY_INFO: u32 = 0x0100;
const CMD_RESOURCE_CREATE_2D: u32 = 0x0101;
const CMD_SET_SCANOUT: u32 = 0x0103;
const CMD_RESOURCE_FLUSH: u32 = 0x0104;
const CMD_TRANSFER_TO_HOST_2D: u32 = 0x0105;
const CMD_RESOURCE_ATTACH_BACKING: u32 = 0x0106;
const RESP_OK_NODATA: u32 = 0x1100;
const RESP_OK_DISPLAY_INFO: u32 = 0x1101;

// virtio-gpu 3D control commands (virtio 1.2 §5.7.6.8, VIRGL feature).
const CMD_CTX_CREATE: u32 = 0x0200;
const CMD_CTX_DESTROY: u32 = 0x0201;
const CMD_CTX_ATTACH_RESOURCE: u32 = 0x0202;
const CMD_CTX_DETACH_RESOURCE: u32 = 0x0203;
const CMD_RESOURCE_CREATE_3D: u32 = 0x0204;
const CMD_TRANSFER_TO_HOST_3D: u32 = 0x0205;
const CMD_TRANSFER_FROM_HOST_3D: u32 = 0x0206;
const CMD_SUBMIT_3D: u32 = 0x0207;

// B8G8R8X8: bytes B,G,R,X in memory => little-endian u32 0x00RRGGBB.
const FORMAT_B8G8R8X8: u32 = 2;
const MAX_SCANOUTS: usize = 16;
const RESOURCE_ID: u32 = 1;

const Desc = extern struct { addr: u64, len: u32, flags: u16, next: u16 };
const Avail = extern struct { flags: u16, idx: u16, ring: [QSIZE]u16, used_event: u16 };
const UsedElem = extern struct { id: u32, len: u32 };
const Used = extern struct { flags: u16, idx: u16, ring: [QSIZE]UsedElem, avail_event: u16 };

// GPU command wire structs (all little-endian), ported from Ferrite.
const GpuRect = extern struct { x: u32, y: u32, width: u32, height: u32 };
const GpuHdr = extern struct {
    type: u32,
    flags: u32 = 0,
    fence_id: u64 = 0,
    ctx_id: u32 = 0,
    padding: u32 = 0,
};
const DisplayOne = extern struct { r: GpuRect, enabled: u32, flags: u32 };
const RespDisplayInfo = extern struct { hdr: GpuHdr, pmodes: [MAX_SCANOUTS]DisplayOne };
const ResourceCreate2D = extern struct {
    hdr: GpuHdr,
    resource_id: u32,
    format: u32,
    width: u32,
    height: u32,
};
const MemEntry = extern struct { addr: u64, length: u32, padding: u32 = 0 };
const AttachBacking = extern struct {
    hdr: GpuHdr,
    resource_id: u32,
    nr_entries: u32,
    entry: MemEntry,
};
const SetScanout = extern struct {
    hdr: GpuHdr,
    r: GpuRect,
    scanout_id: u32,
    resource_id: u32,
};
const TransferToHost2D = extern struct {
    hdr: GpuHdr,
    r: GpuRect,
    offset: u64,
    resource_id: u32,
    padding: u32 = 0,
};
const ResourceFlush = extern struct {
    hdr: GpuHdr,
    r: GpuRect,
    resource_id: u32,
    padding: u32 = 0,
};

// 3D command wire structs (all little-endian), ported from the virtio-gpu UAPI.
const CtxCreate = extern struct {
    hdr: GpuHdr,
    nlen: u32,
    context_init: u32 = 0,
    debug_name: [64]u8 = [_]u8{0} ** 64,
};
const CtxResource = extern struct {
    hdr: GpuHdr,
    resource_id: u32,
    padding: u32 = 0,
};
const ResourceCreate3D = extern struct {
    hdr: GpuHdr,
    resource_id: u32,
    target: u32,
    format: u32,
    bind: u32,
    width: u32,
    height: u32,
    depth: u32,
    array_size: u32,
    last_level: u32,
    nr_samples: u32,
    flags: u32,
    padding: u32 = 0,
};
const GpuBox = extern struct { x: u32, y: u32, z: u32, w: u32, h: u32, d: u32 };
const TransferHost3D = extern struct {
    hdr: GpuHdr,
    box: GpuBox,
    offset: u64,
    resource_id: u32,
    level: u32,
    stride: u32,
    layer_stride: u32,
};
const CmdSubmit3D = extern struct {
    hdr: GpuHdr,
    size: u32,
    padding: u32 = 0,
};

// virgl formats / bind flags / pipe enums (virgl_hw.h, p_defines.h).
const VIRGL_FORMAT_B8G8R8X8_UNORM: u32 = 2;
const VIRGL_FORMAT_B8G8R8A8_UNORM: u32 = 1;
const VIRGL_FORMAT_R32G32_FLOAT: u32 = 29;
const VIRGL_FORMAT_R32G32B32A32_FLOAT: u32 = 31;
const VIRGL_BIND_RENDER_TARGET: u32 = 1 << 1;
const VIRGL_BIND_VERTEX_BUFFER: u32 = 1 << 4;
const VIRGL_BIND_SCANOUT: u32 = 1 << 18;
const PIPE_TEXTURE_2D: u32 = 2;
const PIPE_BUFFER: u32 = 0;
const PIPE_PRIM_TRIANGLES: u32 = 4;
const PIPE_SHADER_VERTEX: u32 = 0;
const PIPE_SHADER_FRAGMENT: u32 = 1;

// virgl context command stream (virgl_protocol.h).
const VIRGL_CMD0_SHIFT_OBJ: u5 = 8;
const VIRGL_CMD0_SHIFT_LEN: u5 = 16;
const CCMD_CREATE_OBJECT: u32 = 1;
const CCMD_BIND_OBJECT: u32 = 2;
const CCMD_SET_VIEWPORT_STATE: u32 = 4;
const CCMD_SET_FRAMEBUFFER_STATE: u32 = 5;
const CCMD_SET_VERTEX_BUFFERS: u32 = 6;
const CCMD_CLEAR: u32 = 7;
const CCMD_DRAW_VBO: u32 = 8;
// BIND_SHADER = 31: after the 28 numbered ctx cmds (0..27) come SET_SUB_CTX(28),
// CREATE_SUB_CTX(29), DESTROY_SUB_CTX(30), then BIND_SHADER(31).
const CCMD_BIND_SHADER: u32 = 31;
const OBJ_BLEND: u32 = 1;
const OBJ_RASTERIZER: u32 = 2;
const OBJ_DSA: u32 = 3;
const OBJ_SHADER: u32 = 4;
const OBJ_VERTEX_ELEMENTS: u32 = 5;
const OBJ_SURFACE: u32 = 8;
// PIPE_CLEAR_COLOR0 | ... (clear all color buffers).
const PIPE_CLEAR_COLOR0: u32 = 1 << 2;

// One DMA buffer big enough for the largest request (AttachBacking) and the
// largest response (RespDisplayInfo). ResourceCreate3D is the largest 3D req.
const REQ_CAP = @max(
    @max(@sizeOf(AttachBacking), @max(@sizeOf(SetScanout), @sizeOf(TransferToHost2D))),
    @max(@sizeOf(CtxCreate), @max(@sizeOf(ResourceCreate3D), @sizeOf(TransferHost3D))),
);
const RESP_CAP = @sizeOf(RespDisplayInfo);

pub const Display = struct { width: u32 = 0, height: u32 = 0 };

pub const Virtio = struct {
    mmio: Mmio,
    present_ok: bool = false,
    last_used: u16 = 0,
    // When true, `start` accepts VIRTIO_GPU_F_VIRGL, so the device exposes the 3D
    // command set. It stays false for the plain 2D path. Then existing callers
    // (and the 2D smoke) keep the exact handshake they had.
    want_virgl: bool = false,
    virgl_ok: bool = false,

    // Bound framebuffer state. `setup` sets it.
    fb: [*]u8 = undefined,
    fb_w: u32 = 0,
    fb_h: u32 = 0,
    have_fb: bool = false,

    desc: [QSIZE]Desc align(16) = undefined,
    avail: Avail align(16) = undefined,
    used: Used align(16) = undefined,
    // Command request/response DMA region (the device reads req, writes resp).
    req_buf: [REQ_CAP]u8 align(16) = undefined,
    resp_buf: [RESP_CAP]u8 align(16) = undefined,

    /// Run the virtio handshake and program the control queue. Returns true if a
    /// v2 virtio-gpu device is ready. Call it on the struct's final address.
    pub fn start(self: *Virtio) bool {
        if (self.mmio.read(u32, R_MAGIC) != MAGIC) return false;
        if (self.mmio.read(u32, R_DEVICE_ID) != DEVICE_ID_GPU) return false;
        if (self.mmio.read(u32, R_VERSION) != 2) return false; // modern only

        self.mmio.write(u32, R_STATUS, 0); // reset
        var status: u32 = S_ACKNOWLEDGE;
        self.mmio.write(u32, R_STATUS, status);
        status |= S_DRIVER;
        self.mmio.write(u32, R_STATUS, status);

        // Accept VIRTIO_F_VERSION_1 (high dword). For the 3D path also accept
        // VIRTIO_GPU_F_VIRGL (low dword bit 0) when the device offers it. The
        // plain 2D path declines all device-specific features (no VIRGL, no
        // EDID), because a 2D scanout needs none of them.
        self.mmio.write(u32, R_DRIVER_FEATURES_SEL, 1);
        self.mmio.write(u32, R_DRIVER_FEATURES, F_VERSION_1_HI);
        self.mmio.write(u32, R_DRIVER_FEATURES_SEL, 0);
        var lo: u32 = 0;
        if (self.want_virgl) {
            self.mmio.write(u32, R_DEVICE_FEATURES_SEL, 0);
            const dev_lo = self.mmio.read(u32, R_DEVICE_FEATURES);
            if (dev_lo & F_VIRGL_LO != 0) {
                lo |= F_VIRGL_LO;
                self.virgl_ok = true;
            }
        }
        self.mmio.write(u32, R_DRIVER_FEATURES_SEL, 0);
        self.mmio.write(u32, R_DRIVER_FEATURES, lo);

        status |= S_FEATURES_OK;
        self.mmio.write(u32, R_STATUS, status);
        if (self.mmio.read(u32, R_STATUS) & S_FEATURES_OK == 0) return false;

        self.mmio.write(u32, R_QUEUE_SEL, VQ_CONTROL);
        if (self.mmio.read(u32, R_QUEUE_NUM_MAX) < QSIZE) return false;
        self.mmio.write(u32, R_QUEUE_NUM, QSIZE);

        self.avail = .{ .flags = 0, .idx = 0, .ring = [_]u16{0} ** QSIZE, .used_event = 0 };
        self.used = .{ .flags = 0, .idx = 0, .ring = [_]UsedElem{.{ .id = 0, .len = 0 }} ** QSIZE, .avail_event = 0 };
        self.last_used = 0;

        self.setQueueAddr(R_QUEUE_DESC_LOW, @intFromPtr(&self.desc));
        self.setQueueAddr(R_QUEUE_DRIVER_LOW, @intFromPtr(&self.avail));
        self.setQueueAddr(R_QUEUE_DEVICE_LOW, @intFromPtr(&self.used));
        self.mmio.write(u32, R_QUEUE_READY, 1);

        status |= S_DRIVER_OK;
        self.mmio.write(u32, R_STATUS, status);
        self.present_ok = true;
        return true;
    }

    fn setQueueAddr(self: *Virtio, off: usize, addr: usize) void {
        self.mmio.write(u32, off, @truncate(addr));
        self.mmio.write(u32, off + 4, @truncate(addr >> 32));
    }

    /// Submit a request/response pair on the control queue. Block until the
    /// device returns the descriptor. This copies `req` into the DMA request
    /// buffer. It returns the response header type (0 on timeout).
    fn submit(self: *Virtio, req: []const u8, resp_len: usize) u32 {
        @memcpy(self.req_buf[0..req.len], req);

        self.desc[0] = .{
            .addr = @intFromPtr(&self.req_buf),
            .len = @intCast(req.len),
            .flags = VIRTQ_DESC_F_NEXT,
            .next = 1,
        };
        self.desc[1] = .{
            .addr = @intFromPtr(&self.resp_buf),
            .len = @intCast(resp_len),
            .flags = VIRTQ_DESC_F_WRITE,
            .next = 0,
        };

        self.avail.ring[self.avail.idx % QSIZE] = 0;
        barrier();
        self.avail.idx +%= 1;
        barrier();
        self.mmio.write(u32, R_QUEUE_NOTIFY, VQ_CONTROL);

        while (@atomicLoad(u16, &self.used.idx, .acquire) == self.last_used) {
            asm volatile ("" ::: .{ .memory = true });
        }
        self.last_used = self.used.idx;
        self.mmio.write(u32, R_INTERRUPT_ACK, self.mmio.read(u32, R_INTERRUPT_STATUS));

        const hdr: *align(16) const GpuHdr = @ptrCast(&self.resp_buf);
        return hdr.type;
    }

    /// Like `submit`, but chains a second device-readable descriptor (`extra`)
    /// after the request header. SUBMIT_3D uses it. There the request is the
    /// fixed CmdSubmit3D header, followed by the variable-length virgl command
    /// stream. This copies `req` into req_buf. It does not copy `extra`. `extra`
    /// must already live in DMA-coherent memory, so the caller passes a pointer
    /// into such a buffer. Returns the response type.
    fn submit2(self: *Virtio, req: []const u8, extra: []const u8, resp_len: usize) u32 {
        @memcpy(self.req_buf[0..req.len], req);

        self.desc[0] = .{
            .addr = @intFromPtr(&self.req_buf),
            .len = @intCast(req.len),
            .flags = VIRTQ_DESC_F_NEXT,
            .next = 1,
        };
        self.desc[1] = .{
            .addr = @intFromPtr(extra.ptr),
            .len = @intCast(extra.len),
            .flags = VIRTQ_DESC_F_NEXT,
            .next = 2,
        };
        self.desc[2] = .{
            .addr = @intFromPtr(&self.resp_buf),
            .len = @intCast(resp_len),
            .flags = VIRTQ_DESC_F_WRITE,
            .next = 0,
        };

        self.avail.ring[self.avail.idx % QSIZE] = 0;
        barrier();
        self.avail.idx +%= 1;
        barrier();
        self.mmio.write(u32, R_QUEUE_NOTIFY, VQ_CONTROL);

        while (@atomicLoad(u16, &self.used.idx, .acquire) == self.last_used) {
            asm volatile ("" ::: .{ .memory = true });
        }
        self.last_used = self.used.idx;
        self.mmio.write(u32, R_INTERRUPT_ACK, self.mmio.read(u32, R_INTERRUPT_STATUS));

        const hdr: *align(16) const GpuHdr = @ptrCast(&self.resp_buf);
        return hdr.type;
    }

    /// Query scanout 0's preferred mode. Returns a sane default (1024x768) when
    /// the device reports no attached monitor (common headless).
    pub fn displayInfo(self: *Virtio) Display {
        const req = GpuHdr{ .type = CMD_GET_DISPLAY_INFO };
        const t = self.submit(asBytes(&req), @sizeOf(RespDisplayInfo));
        if (t != RESP_OK_DISPLAY_INFO) return .{ .width = 1024, .height = 768 };
        const info: *align(16) const RespDisplayInfo = @ptrCast(&self.resp_buf);
        const pm = info.pmodes[0];
        const w: u32 = if (pm.enabled != 0 and pm.r.width != 0) pm.r.width else 1024;
        const h: u32 = if (pm.enabled != 0 and pm.r.height != 0) pm.r.height else 768;
        return .{ .width = w, .height = h };
    }

    /// Bind a guest framebuffer (B8G8R8X8, stride w*4) as the scanout 0 surface:
    /// create the host 2D resource, attach `fb` as its backing, and set scanout
    /// 0 to it. After this, draw into `fb` and call `present`. Returns false if
    /// the device rejects any command.
    pub fn setup(self: *Virtio, fb: [*]u8, w: u32, h: u32) bool {
        const create = ResourceCreate2D{
            .hdr = .{ .type = CMD_RESOURCE_CREATE_2D },
            .resource_id = RESOURCE_ID,
            .format = FORMAT_B8G8R8X8,
            .width = w,
            .height = h,
        };
        if (self.submit(asBytes(&create), @sizeOf(GpuHdr)) != RESP_OK_NODATA) return false;

        const bytes: u64 = @as(u64, w) * @as(u64, h) * 4;
        const attach = AttachBacking{
            .hdr = .{ .type = CMD_RESOURCE_ATTACH_BACKING },
            .resource_id = RESOURCE_ID,
            .nr_entries = 1,
            .entry = .{ .addr = @intFromPtr(fb), .length = @intCast(bytes) },
        };
        if (self.submit(asBytes(&attach), @sizeOf(GpuHdr)) != RESP_OK_NODATA) return false;

        const scanout = SetScanout{
            .hdr = .{ .type = CMD_SET_SCANOUT },
            .r = .{ .x = 0, .y = 0, .width = w, .height = h },
            .scanout_id = 0,
            .resource_id = RESOURCE_ID,
        };
        if (self.submit(asBytes(&scanout), @sizeOf(GpuHdr)) != RESP_OK_NODATA) return false;

        self.fb = fb;
        self.fb_w = w;
        self.fb_h = h;
        self.have_fb = true;
        return true;
    }

    /// Push the current framebuffer contents to the host resource and flush the
    /// full scanout so the host displays it. Call after drawing into `fb`.
    pub fn present(self: *Virtio) bool {
        if (!self.have_fb) return false;
        const xfer = TransferToHost2D{
            .hdr = .{ .type = CMD_TRANSFER_TO_HOST_2D },
            .r = .{ .x = 0, .y = 0, .width = self.fb_w, .height = self.fb_h },
            .offset = 0,
            .resource_id = RESOURCE_ID,
        };
        if (self.submit(asBytes(&xfer), @sizeOf(GpuHdr)) != RESP_OK_NODATA) return false;

        const flush = ResourceFlush{
            .hdr = .{ .type = CMD_RESOURCE_FLUSH },
            .r = .{ .x = 0, .y = 0, .width = self.fb_w, .height = self.fb_h },
            .resource_id = RESOURCE_ID,
        };
        return self.submit(asBytes(&flush), @sizeOf(GpuHdr)) == RESP_OK_NODATA;
    }

    // All 3D commands carry ctx_id in the GpuHdr. They return the device
    // response type so the caller can distinguish OK_NODATA (0x1100) from the
    // 0x12xx error family.

    /// Create a 3D (virgl) context. `debug` names it in host logs.
    pub fn ctxCreate(self: *Virtio, ctx_id: u32, debug: []const u8) u32 {
        var cmd = CtxCreate{
            .hdr = .{ .type = CMD_CTX_CREATE, .ctx_id = ctx_id },
            .nlen = @intCast(@min(debug.len, 63)),
        };
        @memcpy(cmd.debug_name[0..cmd.nlen], debug[0..cmd.nlen]);
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }

    /// Create a 3D resource (texture or buffer) on the host. `target` is a
    /// PIPE_TEXTURE_/PIPE_BUFFER value, and `format`/`bind` are virgl enums.
    pub fn resourceCreate3D(self: *Virtio, res_id: u32, target: u32, format: u32, bind_flags: u32, w: u32, h: u32) u32 {
        const cmd = ResourceCreate3D{
            .hdr = .{ .type = CMD_RESOURCE_CREATE_3D },
            .resource_id = res_id,
            .target = target,
            .format = format,
            .bind = bind_flags,
            .width = w,
            .height = h,
            .depth = 1,
            .array_size = 1,
            .last_level = 0,
            .nr_samples = 0,
            .flags = 0,
        };
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }

    /// Attach a guest backing-store page list (single entry) to a resource.
    pub fn attachBacking(self: *Virtio, res_id: u32, addr: u64, len: u32) u32 {
        const cmd = AttachBacking{
            .hdr = .{ .type = CMD_RESOURCE_ATTACH_BACKING },
            .resource_id = res_id,
            .nr_entries = 1,
            .entry = .{ .addr = addr, .length = len },
        };
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }

    /// Bind a resource into a 3D context so the virgl stream can reference it.
    pub fn ctxAttachResource(self: *Virtio, ctx_id: u32, res_id: u32) u32 {
        const cmd = CtxResource{
            .hdr = .{ .type = CMD_CTX_ATTACH_RESOURCE, .ctx_id = ctx_id },
            .resource_id = res_id,
        };
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }

    /// Upload guest bytes for a 3D resource (e.g. a vertex buffer) to the host.
    pub fn transferToHost3D(self: *Virtio, ctx_id: u32, res_id: u32, box: GpuBox, stride: u32) u32 {
        const cmd = TransferHost3D{
            .hdr = .{ .type = CMD_TRANSFER_TO_HOST_3D, .ctx_id = ctx_id },
            .box = box,
            .offset = 0,
            .resource_id = res_id,
            .level = 0,
            .stride = stride,
            .layer_stride = 0,
        };
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }

    /// Read a 3D resource back into its guest backing (the rendered RT pixels).
    pub fn transferFromHost3D(self: *Virtio, ctx_id: u32, res_id: u32, box: GpuBox, stride: u32) u32 {
        const cmd = TransferHost3D{
            .hdr = .{ .type = CMD_TRANSFER_FROM_HOST_3D, .ctx_id = ctx_id },
            .box = box,
            .offset = 0,
            .resource_id = res_id,
            .level = 0,
            .stride = stride,
            .layer_stride = 0,
        };
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }

    /// Submit a virgl command stream (`stream` is little-endian u32 words, as
    /// raw bytes, living in DMA-coherent memory) to the context.
    pub fn submit3d(self: *Virtio, ctx_id: u32, stream: []const u8) u32 {
        const cmd = CmdSubmit3D{
            .hdr = .{ .type = CMD_SUBMIT_3D, .ctx_id = ctx_id },
            .size = @intCast(stream.len),
        };
        return self.submit2(asBytes(&cmd), stream, @sizeOf(GpuHdr));
    }

    /// Set scanout 0 (or `scanout`) to an arbitrary resource id. The 2D `setup`
    /// hardcodes RESOURCE_ID, so this lets the 3D path scan out a 3D render target.
    pub fn setScanoutRes(self: *Virtio, scanout: u32, res_id: u32, w: u32, h: u32) u32 {
        const cmd = SetScanout{
            .hdr = .{ .type = CMD_SET_SCANOUT },
            .r = .{ .x = 0, .y = 0, .width = w, .height = h },
            .scanout_id = scanout,
            .resource_id = res_id,
        };
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }

    /// Flush an arbitrary resource id's full extent to its scanout.
    pub fn flushRes(self: *Virtio, res_id: u32, w: u32, h: u32) u32 {
        const cmd = ResourceFlush{
            .hdr = .{ .type = CMD_RESOURCE_FLUSH },
            .r = .{ .x = 0, .y = 0, .width = w, .height = h },
            .resource_id = res_id,
        };
        return self.submit(asBytes(&cmd), @sizeOf(GpuHdr));
    }
};

/// Pack a virgl command-stream header dword: 8-bit command, 8-bit object type,
/// 16-bit length (in dwords, excluding the header word).
pub fn virglCmd0(cmd: u32, obj: u32, len: u32) u32 {
    return cmd | (obj << VIRGL_CMD0_SHIFT_OBJ) | (len << VIRGL_CMD0_SHIFT_LEN);
}

/// The virgl encoding constants a consumer needs to build a command stream and
/// create 3D resources. Re-exported so a kernel/harness can drive the 3D path
/// without re-deriving the magic numbers from virgl_protocol.h / virgl_hw.h.
pub const virgl = struct {
    pub const FORMAT_B8G8R8X8_UNORM: u32 = 2;
    pub const FORMAT_B8G8R8A8_UNORM: u32 = 1;
    pub const FORMAT_R32G32_FLOAT: u32 = 29;
    pub const FORMAT_R32G32B32A32_FLOAT: u32 = 31;
    pub const BIND_RENDER_TARGET: u32 = 1 << 1;
    pub const BIND_VERTEX_BUFFER: u32 = 1 << 4;
    pub const BIND_SCANOUT: u32 = 1 << 18;
    pub const TEXTURE_2D: u32 = 2;
    pub const BUFFER: u32 = 0;
    pub const PRIM_TRIANGLES: u32 = 4;
    pub const SHADER_VERTEX: u32 = 0;
    pub const SHADER_FRAGMENT: u32 = 1;
    pub const CLEAR_COLOR0: u32 = 1 << 2;

    pub const CCMD_CREATE_OBJECT: u32 = 1;
    pub const CCMD_BIND_OBJECT: u32 = 2;
    pub const CCMD_SET_VIEWPORT_STATE: u32 = 4;
    pub const CCMD_SET_FRAMEBUFFER_STATE: u32 = 5;
    pub const CCMD_SET_VERTEX_BUFFERS: u32 = 6;
    pub const CCMD_CLEAR: u32 = 7;
    pub const CCMD_DRAW_VBO: u32 = 8;
    pub const CCMD_BIND_SHADER: u32 = 31;
    pub const OBJ_BLEND: u32 = 1;
    pub const OBJ_RASTERIZER: u32 = 2;
    pub const OBJ_DSA: u32 = 3;
    pub const OBJ_SHADER: u32 = 4;
    pub const OBJ_VERTEX_ELEMENTS: u32 = 5;
    pub const OBJ_SURFACE: u32 = 8;

    pub const cmd0 = virglCmd0;
    pub const Box = GpuBox;
};

/// Fill a `Virtio` over `mmio`. Call `start` once it is at its final address.
pub fn bind(mmio: Mmio) Virtio {
    return .{ .mmio = mmio };
}

/// Like `bind`, but requests the VIRTIO_GPU_F_VIRGL feature so `start` brings up
/// the 3D command set (needs a virglrenderer-backed device). After `start`,
/// check `virgl_ok` to confirm the device actually offered the feature.
pub fn bind3d(mmio: Mmio) Virtio {
    return .{ .mmio = mmio, .want_virgl = true };
}

/// A read-only byte view of a packed wire struct, for `submit`.
fn asBytes(ptr: anytype) []const u8 {
    const T = @typeInfo(@TypeOf(ptr)).pointer.child;
    const p: [*]const u8 = @ptrCast(ptr);
    return p[0..@sizeOf(T)];
}

/// A memory barrier. It orders virtqueue writes against the device. The compiler
/// selects it per arch, so conduit compiles for any target.
fn barrier() void {
    switch (builtin.cpu.arch) {
        .riscv64, .riscv32 => asm volatile ("fence" ::: .{ .memory = true }),
        .aarch64 => asm volatile ("dsb sy" ::: .{ .memory = true }),
        .x86_64, .x86 => asm volatile ("mfence" ::: .{ .memory = true }),
        else => asm volatile ("" ::: .{ .memory = true }),
    }
}

const std = @import("std");
const fake = struct {
    // A fake virtio-gpu device backing for the unit test: it answers the MMIO
    // handshake registers, and on each NOTIFY it advances the used ring and
    // writes a canned OK response into the resp descriptor, so `submit`
    // completes against a deterministic stand-in (no real device).
    const Self = @This();

    v: *Virtio = undefined,
    status: u32 = 0,
    queue_ready: u32 = 0,
    last_avail: u16 = 0,
    saw_create: bool = false,
    saw_attach: bool = false,
    saw_scanout: bool = false,
    saw_transfer: bool = false,
    saw_flush: bool = false,

    fn read(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const self: *Self = @ptrCast(@alignCast(ctx));
        return switch (off) {
            R_MAGIC => MAGIC,
            R_VERSION => 2,
            R_DEVICE_ID => DEVICE_ID_GPU,
            R_QUEUE_NUM_MAX => QSIZE,
            R_STATUS => self.status,
            R_INTERRUPT_STATUS => 1,
            else => 0,
        };
    }

    fn write(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const self: *Self = @ptrCast(@alignCast(ctx));
        switch (off) {
            R_STATUS => self.status = @truncate(val),
            R_QUEUE_READY => self.queue_ready = @truncate(val),
            R_QUEUE_NOTIFY => self.serviceQueue(),
            else => {},
        }
    }

    // Drain every avail descriptor the driver posted since the last notify,
    // decode the request, and post an OK response.
    fn serviceQueue(self: *Self) void {
        const v = self.v;
        while (self.last_avail != v.avail.idx) {
            const head = v.avail.ring[self.last_avail % QSIZE];
            const req_desc = v.desc[head];
            const cmd_type = std.mem.readInt(u32, v.req_buf[0..4], .little);
            const resp_desc = v.desc[req_desc.next];

            var resp = GpuHdr{ .type = RESP_OK_NODATA };
            switch (cmd_type) {
                CMD_GET_DISPLAY_INFO => {
                    var info = std.mem.zeroes(RespDisplayInfo);
                    info.hdr.type = RESP_OK_DISPLAY_INFO;
                    info.pmodes[0] = .{ .r = .{ .x = 0, .y = 0, .width = 800, .height = 600 }, .enabled = 1, .flags = 0 };
                    const dst: [*]u8 = @ptrFromInt(@as(usize, @intCast(resp_desc.addr)));
                    @memcpy(dst[0..@sizeOf(RespDisplayInfo)], std.mem.asBytes(&info));
                },
                CMD_RESOURCE_CREATE_2D => self.saw_create = true,
                CMD_RESOURCE_ATTACH_BACKING => self.saw_attach = true,
                CMD_SET_SCANOUT => self.saw_scanout = true,
                CMD_TRANSFER_TO_HOST_2D => self.saw_transfer = true,
                CMD_RESOURCE_FLUSH => self.saw_flush = true,
                else => {},
            }
            if (cmd_type != CMD_GET_DISPLAY_INFO) {
                const dst: [*]u8 = @ptrFromInt(@as(usize, @intCast(resp_desc.addr)));
                @memcpy(dst[0..@sizeOf(GpuHdr)], std.mem.asBytes(&resp));
            }

            v.used.ring[v.used.idx % QSIZE] = .{ .id = head, .len = 0 };
            barrier();
            v.used.idx +%= 1;
            self.last_avail +%= 1;
        }
    }

    fn mmio(self: *Self) Mmio {
        return .{ .ctx = self, .base = 0, .read_fn = read, .write_fn = write };
    }
};

test "virtio_gpu: handshake, display info, setup, present over a fake device" {
    var dev = fake{};
    var v = bind(dev.mmio());
    dev.v = &v;

    try std.testing.expect(v.start());
    try std.testing.expect(dev.status & S_DRIVER_OK != 0);

    const info = v.displayInfo();
    try std.testing.expectEqual(@as(u32, 800), info.width);
    try std.testing.expectEqual(@as(u32, 600), info.height);

    // A small framebuffer (the test asserts the command sequence, not pixels).
    var fb: [16 * 16 * 4]u8 align(16) = undefined;
    try std.testing.expect(v.setup(&fb, 16, 16));
    try std.testing.expect(dev.saw_create);
    try std.testing.expect(dev.saw_attach);
    try std.testing.expect(dev.saw_scanout);

    try std.testing.expect(v.present());
    try std.testing.expect(dev.saw_transfer);
    try std.testing.expect(dev.saw_flush);
}

test "virtio_gpu: setup is rejected before start completes the resource path" {
    // present() must refuse before setup() bound a framebuffer.
    var dev = fake{};
    var v = bind(dev.mmio());
    dev.v = &v;
    try std.testing.expect(v.start());
    try std.testing.expect(!v.present());
}
