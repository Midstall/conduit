//! UEFI PCI backend: EFI_PCI_IO_PROTOCOL via boot services.
//!
//! Enumerates handles with LocateHandleBuffer(ByProtocol, EFI_PCI_IO GUID). For
//! each handle it opens the protocol, reads GetLocation (seg/bus/dev/fn) and
//! config-space (vendor/device/class), and resolves BARs. BAR bases come STRAIGHT
//! from config space (offset 0x10 + 0x14/0x18..., masking the low type bits and
//! folding the high dword of a 64-bit BAR). This is the proven approach, immune
//! to the ACPI QWORD-descriptor parse bug. conduit uses GetBarAttributes only to
//! refine each BAR's size (informational).
//!
//! The hand-rolled EFI_PCI_IO_PROTOCOL wrapper (GUID 4cf5b200-..., the Mem/Io/Pci
//! access groups plus GetLocation and GetBarAttributes fn-table) lives in
//! pci_io.zig beside this file, recovered from the proven Prism UEFI probe.

const std = @import("std");
const uefi = std.os.uefi;
const backend = @import("../../backend.zig");
const resource = @import("../../resource.zig");
const pci = @import("../pci.zig");
const pci_io = @import("pci_io.zig");

pub const PciIo = pci_io.PciIo;

/// Max BARs we resolve (BAR0..5).
const max_bars = 6;

pub const UefiPciBackend = struct {
    handles: []uefi.Handle = &.{},
    idx: usize = 0,
    /// The protocol of the node most recently returned, for `resources`.
    cur: ?*const PciIo = null,

    /// Build a backend over the live UEFI boot services (std.os.uefi.system_table).
    pub fn init() UefiPciBackend {
        var self = UefiPciBackend{};
        self.locate();
        return self;
    }

    pub fn any(self: *UefiPciBackend) backend.Backend {
        return backend.fromImpl(self);
    }

    pub fn reset(self: *UefiPciBackend) void {
        self.idx = 0;
        self.cur = null;
    }

    fn locate(self: *UefiPciBackend) void {
        const st = uefi.system_table;
        const bs = st.boot_services orelse return;
        const found = bs.locateHandleBuffer(.{ .by_protocol = &PciIo.guid }) catch return;
        self.handles = found orelse &.{};
    }

    pub fn next(self: *UefiPciBackend) backend.Error!?backend.Node {
        const st = uefi.system_table;
        const bs = st.boot_services orelse return null;

        while (self.idx < self.handles.len) {
            const handle = self.handles[self.idx];
            self.idx += 1;

            const proto = (bs.openProtocol(PciIo, handle, .{
                .get_protocol = .{ .agent = uefi.handle },
            }) catch continue) orelse continue;

            const loc = proto.getLocation() catch continue;
            const vendor = proto.configRead(u16, 0x00) catch continue;
            const device = proto.configRead(u16, 0x02) catch continue;
            const revision = proto.configRead(u8, 0x08) catch 0;
            const prog_if = proto.configRead(u8, 0x09) catch 0;
            const subclass = proto.configRead(u8, 0x0A) catch 0;
            const class_code = proto.configRead(u8, 0x0B) catch 0;

            const info = backend.PciInfo{
                .segment = @truncate(loc.segment),
                .bus = @truncate(loc.bus),
                .device = @truncate(loc.device),
                .function = @truncate(loc.function),
                .vendor_id = vendor,
                .device_id = device,
                .class_code = class_code,
                .subclass = subclass,
                .prog_if = prog_if,
                .revision = revision,
            };

            self.cur = proto;
            return pci.synthIds(info);
        }
        return null;
    }

    pub fn resources(self: *UefiPciBackend, node: backend.Node, out: *resource.List) backend.Error!void {
        _ = node;
        out.* = .{};
        const proto = self.cur orelse return;

        var bar: u8 = 0;
        while (bar < max_bars) : (bar += 1) {
            const off: u32 = 0x10 + @as(u32, bar) * 4;
            const raw = proto.configRead(u32, off) catch continue;
            if (raw == 0) continue;

            if (raw & 0x1 != 0) {
                // IO-space BAR: base in [31:2].
                const port: u16 = @truncate(raw & 0xFFFF_FFFC);
                try out.append(.{ .reg_io = .{ .port = port, .size = 0 } });
                continue;
            }

            // Memory BAR. Type bits [2:1]: 0b10 == 64-bit (consumes the next slot).
            var base: u64 = raw & 0xFFFF_FFF0;
            const is64 = (raw & 0x6) == 0x4;
            if (is64) {
                const hi = proto.configRead(u32, off + 4) catch 0;
                base |= @as(u64, hi) << 32;
            }
            if (base == 0) {
                if (is64) bar += 1;
                continue;
            }

            // Refine the size via GetBarAttributes (informational), default 0.
            var size: u64 = 0;
            if (proto.barInfo(bar) catch null) |bi| size = bi.size;

            try out.append(.{ .mmio = .{ .base = base, .size = size } });
            if (is64) bar += 1; // a 64-bit BAR occupies two config slots.
        }
    }
};

test {
    _ = pci_io;
}
