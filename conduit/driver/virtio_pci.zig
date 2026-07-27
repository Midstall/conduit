//! virtio over PCI (modern, virtio 1.0+): walk vendor capabilities, map the
//! common/notify/isr/device-cfg regions out of BAR+offset, and expose the
//! register helpers a device driver needs for handshake / queue setup / notify.
//!
//! This is transport only. Device-class drivers (e.g. virtio_gpu) still own
//! their command protocol; they call into `Transport` instead of virtio-mmio
//! register offsets when bound over PCI.

const Mmio = @import("../mmio.zig");
const freestanding_pci = @import("../backend/pci/freestanding.zig");

pub const BarInfo = freestanding_pci.BarInfo;

const PCI_CAP_ID_VNDR: u8 = 0x09;

pub const CFG_COMMON: u8 = 1;
pub const CFG_NOTIFY: u8 = 2;
pub const CFG_ISR: u8 = 3;
pub const CFG_DEVICE: u8 = 4;

/// virtio-pci common configuration field offsets (virtio 1.2 §4.1.4.3).
pub const C_DEVICE_FEATURE_SELECT: usize = 0x00;
pub const C_DEVICE_FEATURE: usize = 0x04;
pub const C_DRIVER_FEATURE_SELECT: usize = 0x08;
pub const C_DRIVER_FEATURE: usize = 0x0c;
pub const C_NUM_QUEUES: usize = 0x12;
pub const C_DEVICE_STATUS: usize = 0x14;
pub const C_CONFIG_GENERATION: usize = 0x15;
pub const C_QUEUE_SELECT: usize = 0x16;
pub const C_QUEUE_SIZE: usize = 0x18;
pub const C_QUEUE_ENABLE: usize = 0x1c;
pub const C_QUEUE_NOTIFY_OFF: usize = 0x1e;
pub const C_QUEUE_DESC: usize = 0x20;
pub const C_QUEUE_DRIVER: usize = 0x28;
pub const C_QUEUE_DEVICE: usize = 0x30;

pub const S_ACKNOWLEDGE: u8 = 1;
pub const S_DRIVER: u8 = 2;
pub const S_DRIVER_OK: u8 = 4;
pub const S_FEATURES_OK: u8 = 8;
pub const S_FAILED: u8 = 128;

/// Red Hat virtio vendor; modern GPU device id = 0x1040 + 16.
pub const VIRTIO_VENDOR: u16 = 0x1AF4;
pub const VIRTIO_GPU_MODERN_DEVICE_ID: u16 = 0x1050;

pub const Transport = struct {
    common: Mmio,
    notify_base: u64,
    notify_off_multiplier: u32,
    isr: Mmio,
    device_cfg: Mmio,

    pub fn status(self: Transport) u8 {
        return self.common.read(u8, C_DEVICE_STATUS);
    }

    pub fn setStatus(self: Transport, s: u8) void {
        self.common.write(u8, C_DEVICE_STATUS, s);
    }

    pub fn reset(self: Transport) void {
        self.setStatus(0);
        // Flush the write; some hosts require a readback after reset.
        _ = self.status();
    }

    pub fn deviceFeatures(self: Transport, select: u32) u32 {
        self.common.write(u32, C_DEVICE_FEATURE_SELECT, select);
        return self.common.read(u32, C_DEVICE_FEATURE);
    }

    pub fn setDriverFeatures(self: Transport, select: u32, features: u32) void {
        self.common.write(u32, C_DRIVER_FEATURE_SELECT, select);
        self.common.write(u32, C_DRIVER_FEATURE, features);
    }

    pub fn setQueue(self: Transport, index: u16, size: u16, desc: u64, driver: u64, device: u64) bool {
        self.common.write(u16, C_QUEUE_SELECT, index);
        const max = self.common.read(u16, C_QUEUE_SIZE);
        if (max < size) return false;
        self.common.write(u16, C_QUEUE_SIZE, size);
        writeU64(self.common, C_QUEUE_DESC, desc);
        writeU64(self.common, C_QUEUE_DRIVER, driver);
        writeU64(self.common, C_QUEUE_DEVICE, device);
        self.common.write(u16, C_QUEUE_ENABLE, 1);
        return true;
    }

    pub fn notify(self: Transport, queue_index: u16) void {
        self.common.write(u16, C_QUEUE_SELECT, queue_index);
        const notify_off = self.common.read(u16, C_QUEUE_NOTIFY_OFF);
        const addr = self.notify_base + @as(u64, notify_off) * @as(u64, self.notify_off_multiplier);
        const ptr: *volatile u16 = @ptrFromInt(addr);
        ptr.* = queue_index;
    }

    pub fn ackInterrupt(self: Transport) void {
        // Reading the ISR clears it.
        _ = self.isr.read(u8, 0);
    }
};

fn writeU64(mmio: Mmio, off: usize, val: u64) void {
    mmio.write(u32, off, @truncate(val));
    mmio.write(u32, off + 4, @truncate(val >> 32));
}

const CapLoc = struct {
    bar: u8,
    offset: u32,
    length: u32,
    notify_off_multiplier: u32 = 0,
};

/// Assign unassigned BARs for this BDF (QEMU direct-kernel boot leaves them at
/// 0). Returns the BAR table so the kernel can `mapExtra` each memory window
/// before `bind` touches them.
pub fn assignBars(ecam_base: u64, bus: u8, device: u8, function: u8) [6]BarInfo {
    const bdf = freestanding_pci.Bdf{ .bus = bus, .device = device, .function = function };
    var alloc = freestanding_pci.BarAllocator{};
    return alloc.assign(ecam_base, bdf);
}

/// Bind a modern virtio-pci transport from ECAM + BDF. Enables Memory Space and
/// Bus Master, walks vendor caps, and returns Mmio views into the BAR regions.
/// The caller must have identity-mapped every memory BAR beforehand (see
/// `assignBars`).
pub fn bind(ecam_base: u64, bus: u8, device: u8, function: u8) ?Transport {
    const bdf = freestanding_pci.Bdf{ .bus = bus, .device = device, .function = function };

    const vendor = freestanding_pci.configRead16(ecam_base, bdf, freestanding_pci.VENDOR_ID);
    if (vendor == 0xFFFF) return null;

    // Assign any still-zero BARs, then enable decoding.
    var alloc = freestanding_pci.BarAllocator{};
    const bars = alloc.assign(ecam_base, bdf);

    var bar_base: [6]u64 = .{0} ** 6;
    for (bars, 0..) |bar, i| {
        if (!bar.is_high_half and !bar.is_io) bar_base[i] = bar.base;
    }

    const status = freestanding_pci.configRead16(ecam_base, bdf, freestanding_pci.STATUS);
    if (status & freestanding_pci.STATUS_CAP_LIST == 0) return null;

    var common: ?CapLoc = null;
    var notify: ?CapLoc = null;
    var isr: ?CapLoc = null;
    var device_cfg: ?CapLoc = null;

    var ptr: u8 = freestanding_pci.configRead8(ecam_base, bdf, freestanding_pci.CAP_PTR) & 0xFC;
    var guard: u8 = 0;
    while (ptr != 0 and guard < 64) : (guard += 1) {
        const cap_id = freestanding_pci.configRead8(ecam_base, bdf, ptr);
        const next = freestanding_pci.configRead8(ecam_base, bdf, ptr + 1);
        if (cap_id == PCI_CAP_ID_VNDR) {
            const cfg_type = freestanding_pci.configRead8(ecam_base, bdf, ptr + 3);
            const bar = freestanding_pci.configRead8(ecam_base, bdf, ptr + 4);
            const offset = freestanding_pci.configRead32(ecam_base, bdf, ptr + 8);
            const length = freestanding_pci.configRead32(ecam_base, bdf, ptr + 12);
            const loc = CapLoc{ .bar = bar, .offset = offset, .length = length };
            switch (cfg_type) {
                CFG_COMMON => common = loc,
                CFG_NOTIFY => {
                    var n = loc;
                    n.notify_off_multiplier = freestanding_pci.configRead32(ecam_base, bdf, ptr + 16);
                    notify = n;
                },
                CFG_ISR => isr = loc,
                CFG_DEVICE => device_cfg = loc,
                else => {},
            }
        }
        ptr = next & 0xFC;
    }

    const c = common orelse return null;
    const n = notify orelse return null;
    const i = isr orelse return null;
    // device_cfg is optional for some devices; gpu needs it for nothing critical
    // (display info goes over the control queue), but keep a view when present.
    const d = device_cfg orelse CapLoc{ .bar = c.bar, .offset = c.offset, .length = 0 };

    if (c.bar >= 6 or n.bar >= 6 or i.bar >= 6 or d.bar >= 6) return null;
    if (bar_base[c.bar] == 0 or bar_base[n.bar] == 0 or bar_base[i.bar] == 0) return null;

    return .{
        .common = Mmio.direct(bar_base[c.bar] + c.offset),
        .notify_base = bar_base[n.bar] + n.offset,
        .notify_off_multiplier = n.notify_off_multiplier,
        .isr = Mmio.direct(bar_base[i.bar] + i.offset),
        .device_cfg = if (d.length != 0 and bar_base[d.bar] != 0)
            Mmio.direct(bar_base[d.bar] + d.offset)
        else
            Mmio.direct(0),
    };
}
