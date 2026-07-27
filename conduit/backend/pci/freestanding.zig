//! Freestanding PCI backend: enumerates devices via ECAM (Enhanced Configuration
//! Access Mechanism). The ECAM base address is provided at init time (discovered
//! from the DTB's PCI host bridge node's `reg` property).
//!
//! PCI config space is memory-mapped at:
//!   ECAM_BASE + (bus << 20 | device << 15 | function << 12 | offset)
//!
//! Each device's vendor/device/class/BARs are read directly from config space.
//! The backend scans buses × 32 devices × 8 functions, yielding a
//! conduit `Node` with `PciInfo` for each present device (vendor_id != 0xFFFF).

const backend = @import("../../backend.zig");
const resource = @import("../../resource.zig");
const pci = @import("../pci.zig");

/// PCI config space register offsets.
pub const VENDOR_ID: u16 = 0x00;
pub const DEVICE_ID: u16 = 0x02;
pub const COMMAND: u16 = 0x04;
pub const STATUS: u16 = 0x06;
pub const REG_CLASS: u16 = 0x08; // class_code[31:24] subclass[23:16] prog_if[15:8] revision[7:0]
pub const HEADER_TYPE: u16 = 0x0E;
pub const BAR0: u16 = 0x10;
pub const CAP_PTR: u16 = 0x34;

pub const COMMAND_MEMORY: u16 = 1 << 1;
pub const COMMAND_BUS_MASTER: u16 = 1 << 2;
pub const STATUS_CAP_LIST: u16 = 1 << 4;

/// Invalid vendor ID (no device present).
const INVALID_VENDOR: u16 = 0xFFFF;

/// Buses to scan. QEMU virt places devices on low bus numbers; keeping this
/// bounded avoids touching unmapped high ECAM pages when the mapped window is
/// smaller than a full 256-bus hierarchy.
const MAX_BUSES: u16 = 16;
/// Devices per bus.
const DEVICES_PER_BUS: u8 = 32;
/// Functions per device.
const FUNCTIONS_PER_DEVICE: u8 = 8;

pub const Bdf = struct {
    bus: u8 = 0,
    device: u8 = 0,
    function: u8 = 0,
};

pub const BarInfo = struct {
    base: u64 = 0,
    size: u64 = 0,
    is_io: bool = false,
    is_64: bool = false,
    /// True when this slot is the high half of a 64-bit BAR (not a distinct region).
    is_high_half: bool = false,
};

pub const FreestandingPciBackend = struct {
    /// ECAM base address (physical, identity-mapped in the kernel).
    ecam_base: u64,
    /// Current scan position.
    bus: u16 = 0,
    device: u8 = 0,
    function: u8 = 0,
    /// The most recently returned node's BDF, so `resources` can read its BARs.
    last_bdf: Bdf = .{},
    /// After yielding a single-function device on function 0, skip functions 1–7.
    skip_remaining_functions: bool = false,

    pub fn init(ecam_base: u64) FreestandingPciBackend {
        return .{ .ecam_base = ecam_base };
    }

    pub fn any(self: *FreestandingPciBackend) backend.Backend {
        return backend.fromImpl(self);
    }

    pub fn reset(self: *FreestandingPciBackend) void {
        self.bus = 0;
        self.device = 0;
        self.function = 0;
        self.skip_remaining_functions = false;
    }

    pub fn next(self: *FreestandingPciBackend) backend.Error!?backend.Node {
        while (self.bus < MAX_BUSES) {
            const bdf = Bdf{
                .bus = @intCast(self.bus),
                .device = self.device,
                .function = self.function,
            };

            const vendor = configRead16(self.ecam_base, bdf, VENDOR_ID);
            if (vendor == INVALID_VENDOR) {
                // Empty function 0 ⇒ no device here; skip the rest of the slot.
                if (self.function == 0) self.function = 7;
                self.advance();
                continue;
            }

            const device_id = configRead16(self.ecam_base, bdf, DEVICE_ID);
            const class_reg = configRead32(self.ecam_base, bdf, REG_CLASS);

            const info = backend.PciInfo{
                .bus = bdf.bus,
                .device = bdf.device,
                .function = bdf.function,
                .vendor_id = vendor,
                .device_id = device_id,
                .class_code = @truncate(class_reg >> 24),
                .subclass = @truncate(class_reg >> 16),
                .prog_if = @truncate(class_reg >> 8),
                .revision = @truncate(class_reg),
            };

            if (self.function == 0) {
                const hdr_type = configRead8(self.ecam_base, bdf, HEADER_TYPE);
                if (hdr_type & 0x80 == 0) self.skip_remaining_functions = true;
            }

            self.last_bdf = bdf;
            // Advance before returning so the next call does not re-yield this BDF.
            self.advance();
            return pci.synthIds(info);
        }
        return null; // scan complete
    }

    pub fn resources(self: *FreestandingPciBackend, node: backend.Node, out: *resource.List) backend.Error!void {
        _ = node;
        out.* = .{};

        const bars = readBars(self.ecam_base, self.last_bdf);
        for (bars) |bar| {
            if (bar.is_high_half or bar.base == 0 or bar.size == 0) continue;
            if (bar.is_io) {
                try out.append(.{ .reg_io = .{ .port = @truncate(bar.base), .size = @truncate(@min(bar.size, 0xffff)) } });
            } else {
                try out.append(.{ .mmio = .{ .base = bar.base, .size = bar.size } });
            }
        }
    }

    /// Advance to the next BDF slot.
    fn advance(self: *FreestandingPciBackend) void {
        if (self.skip_remaining_functions) {
            self.skip_remaining_functions = false;
            self.function = 0;
            self.device += 1;
            if (self.device >= DEVICES_PER_BUS) {
                self.device = 0;
                self.bus += 1;
            }
            return;
        }

        self.function += 1;
        if (self.function >= FUNCTIONS_PER_DEVICE) {
            self.function = 0;
            self.device += 1;
            if (self.device >= DEVICES_PER_BUS) {
                self.device = 0;
                self.bus += 1;
            }
        }
    }
};

/// Enable Memory Space + Bus Master for `bdf` so BARs and DMA work.
pub fn enableMemAndBusMaster(ecam_base: u64, bdf: Bdf) void {
    const cmd = configRead16(ecam_base, bdf, COMMAND);
    configWrite16(ecam_base, bdf, COMMAND, cmd | COMMAND_MEMORY | COMMAND_BUS_MASTER);
}

/// Read BAR0..5 with size probing. Unassigned BARs (current value 0) are still
/// probed for size/type so the caller can allocate addresses. 64-bit BAR high
/// halves are marked `is_high_half`.
pub fn readBars(ecam_base: u64, bdf: Bdf) [6]BarInfo {
    var out: [6]BarInfo = [_]BarInfo{.{}} ** 6;
    var i: u8 = 0;
    while (i < 6) {
        const bar_off: u16 = BAR0 + @as(u16, @intCast(i * 4));
        const orig = configRead32(ecam_base, bdf, bar_off);

        // Size-probe even when the BAR is currently 0 (unassigned). An
        // unimplemented slot reads back 0 (or all-ones) after the write.
        configWrite32(ecam_base, bdf, bar_off, 0xFFFFFFFF);
        const mask = configRead32(ecam_base, bdf, bar_off);
        configWrite32(ecam_base, bdf, bar_off, orig);

        if (mask == 0 or mask == 0xFFFFFFFF) {
            i += 1;
            continue;
        }

        if (mask & 1 != 0) {
            // I/O BAR.
            const size: u64 = (~(mask & 0xFFFFFFFC) +% 1);
            out[i] = .{
                .base = orig & 0xFFFFFFFC,
                .size = size,
                .is_io = true,
            };
            i += 1;
            continue;
        }

        const mem_type = (mask >> 1) & 0x3;
        if (mem_type == 2 and i + 1 < 6) {
            // 64-bit memory BAR.
            const orig_hi = configRead32(ecam_base, bdf, bar_off + 4);
            configWrite32(ecam_base, bdf, bar_off + 4, 0xFFFFFFFF);
            const mask_hi = configRead32(ecam_base, bdf, bar_off + 4);
            configWrite32(ecam_base, bdf, bar_off + 4, orig_hi);

            const mask64 = (@as(u64, mask_hi) << 32) | (@as(u64, mask) & 0xFFFFFFF0);
            const size: u64 = if (mask64 == 0) 0 else (~mask64 +% 1);
            out[i] = .{
                .base = (@as(u64, orig_hi) << 32) | (@as(u64, orig) & 0xFFFFFFF0),
                .size = size,
                .is_64 = true,
            };
            out[i + 1] = .{ .is_high_half = true, .is_64 = true };
            i += 2;
        } else {
            const size: u64 = (~(mask & 0xFFFFFFF0) +% 1);
            out[i] = .{
                .base = orig & 0xFFFFFFF0,
                .size = size,
            };
            i += 1;
        }
    }
    return out;
}

/// QEMU virt PCI MMIO windows used when firmware has not assigned BARs
/// (common with direct `-kernel` boot). 32-bit window at 0x10000000;
/// 64-bit high window at 0x8000000000.
pub const BarAllocator = struct {
    mmio32: u64 = 0x1000_0000,
    mmio32_end: u64 = 0x3eff_0000,
    mmio64: u64 = 0x80_0000_0000,
    mmio64_end: u64 = 0x100_0000_0000,
    io: u64 = 0x3eff_0000,
    io_end: u64 = 0x3f00_0000,

    fn alignUp(addr: u64, size: u64) u64 {
        if (size <= 1) return addr;
        return (addr + size - 1) & ~(size - 1);
    }

    /// Assign addresses to any BAR that has a non-zero size but base 0, write
    /// them into config space, and return the updated BAR table.
    pub fn assign(self: *BarAllocator, ecam_base: u64, bdf: Bdf) [6]BarInfo {
        var bars = readBars(ecam_base, bdf);
        var i: u8 = 0;
        while (i < 6) {
            if (bars[i].is_high_half) {
                i += 1;
                continue;
            }
            if (bars[i].size == 0 or bars[i].base != 0) {
                i += if (bars[i].is_64) @as(u8, 2) else 1;
                continue;
            }

            const bar_off: u16 = BAR0 + @as(u16, @intCast(i * 4));
            if (bars[i].is_io) {
                const addr = alignUp(self.io, bars[i].size);
                if (addr + bars[i].size > self.io_end) break;
                self.io = addr + bars[i].size;
                configWrite32(ecam_base, bdf, bar_off, @as(u32, @truncate(addr)) | 1);
                bars[i].base = addr;
                i += 1;
            } else if (bars[i].is_64) {
                // Prefer the 32-bit MMIO window when the BAR fits (keeps us off
                // the high PCI window, which is easy to mis-size against DTB
                // ranges under direct -kernel boot).
                const use_low = bars[i].size <= (self.mmio32_end - self.mmio32);
                if (use_low) {
                    const addr = alignUp(self.mmio32, bars[i].size);
                    if (addr + bars[i].size > self.mmio32_end) break;
                    self.mmio32 = addr + bars[i].size;
                    configWrite32(ecam_base, bdf, bar_off, @as(u32, @truncate(addr & 0xFFFFFFF0)) | 0x4);
                    configWrite32(ecam_base, bdf, bar_off + 4, 0);
                    bars[i].base = addr;
                } else {
                    const addr = alignUp(self.mmio64, bars[i].size);
                    if (addr + bars[i].size > self.mmio64_end) break;
                    self.mmio64 = addr + bars[i].size;
                    configWrite32(ecam_base, bdf, bar_off, @as(u32, @truncate(addr & 0xFFFFFFF0)) | 0x4);
                    configWrite32(ecam_base, bdf, bar_off + 4, @truncate(addr >> 32));
                    bars[i].base = addr;
                }
                i += 2;
            } else {
                const addr = alignUp(self.mmio32, bars[i].size);
                if (addr + bars[i].size > self.mmio32_end) break;
                self.mmio32 = addr + bars[i].size;
                configWrite32(ecam_base, bdf, bar_off, @as(u32, @truncate(addr & 0xFFFFFFF0)));
                bars[i].base = addr;
                i += 1;
            }
        }
        enableMemAndBusMaster(ecam_base, bdf);
        return bars;
    }
};

pub fn configRead8(ecam_base: u64, bdf: Bdf, offset: u16) u8 {
    const ptr: *const volatile u8 = @ptrFromInt(ecamAddr(ecam_base, bdf, offset));
    return ptr.*;
}

pub fn configRead16(ecam_base: u64, bdf: Bdf, offset: u16) u16 {
    const lo = configRead8(ecam_base, bdf, offset);
    const hi = configRead8(ecam_base, bdf, offset + 1);
    return @as(u16, lo) | (@as(u16, hi) << 8);
}

pub fn configRead32(ecam_base: u64, bdf: Bdf, offset: u16) u32 {
    const b0 = configRead8(ecam_base, bdf, offset);
    const b1 = configRead8(ecam_base, bdf, offset + 1);
    const b2 = configRead8(ecam_base, bdf, offset + 2);
    const b3 = configRead8(ecam_base, bdf, offset + 3);
    return @as(u32, b0) | (@as(u32, b1) << 8) | (@as(u32, b2) << 16) | (@as(u32, b3) << 24);
}

pub fn configWrite8(ecam_base: u64, bdf: Bdf, offset: u16, val: u8) void {
    const ptr: *volatile u8 = @ptrFromInt(ecamAddr(ecam_base, bdf, offset));
    ptr.* = val;
}

pub fn configWrite16(ecam_base: u64, bdf: Bdf, offset: u16, val: u16) void {
    configWrite8(ecam_base, bdf, offset, @truncate(val));
    configWrite8(ecam_base, bdf, offset + 1, @truncate(val >> 8));
}

pub fn configWrite32(ecam_base: u64, bdf: Bdf, offset: u16, val: u32) void {
    configWrite8(ecam_base, bdf, offset, @truncate(val));
    configWrite8(ecam_base, bdf, offset + 1, @truncate(val >> 8));
    configWrite8(ecam_base, bdf, offset + 2, @truncate(val >> 16));
    configWrite8(ecam_base, bdf, offset + 3, @truncate(val >> 24));
}

fn ecamAddr(ecam_base: u64, bdf: Bdf, offset: u16) u64 {
    return ecam_base |
        (@as(u64, bdf.bus) << 20) |
        (@as(u64, bdf.device) << 15) |
        (@as(u64, bdf.function) << 12) |
        @as(u64, offset);
}
