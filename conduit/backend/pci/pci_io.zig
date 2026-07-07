//! Minimal hand-rolled EFI_PCI_IO_PROTOCOL wrapper. Zig 0.16's std.os.uefi has
//! no PCI_IO protocol, so we define the GUID + the function-table layout from the
//! UEFI 2.10 spec (matched to EDK2 MdePkg/Include/Protocol/PciIo.h) ourselves.
//!
//! The struct field ORDER and the calling convention (uefi.cc) are the ABI, so
//! they must match the firmware's table exactly. We only wrap the calls the probe
//! needs: GetLocation, config-space read (Pci.Read), and GetBarAttributes.
//!
//! Recovered from the proven Prism UEFI GPU probe (the bug-fixed descriptor
//! parser is the version that reads the ACPI fields at exact unaligned offsets).

const std = @import("std");
const uefi = std.os.uefi;
const Guid = uefi.Guid;
const Status = uefi.Status;
const cc = uefi.cc;

pub const Width = enum(u32) {
    uint8 = 0,
    uint16 = 1,
    uint32 = 2,
    uint64 = 3,
    _,
};

/// A read/write pair, used for the Mem, Io and Pci access groups.
const Access = extern struct {
    read: *const fn (
        *const PciIo,
        Width,
        offset: u32,
        count: usize,
        buffer: *anyopaque,
    ) callconv(cc) Status,
    write: *const fn (
        *const PciIo,
        Width,
        offset: u32,
        count: usize,
        buffer: *anyopaque,
    ) callconv(cc) Status,
};

pub const PciIo = extern struct {
    _poll_mem: *const anyopaque,
    _poll_io: *const anyopaque,
    mem: Access,
    io: Access,
    pci: Access,
    _copy_mem: *const anyopaque,
    _map: *const anyopaque,
    _unmap: *const anyopaque,
    _allocate_buffer: *const anyopaque,
    _free_buffer: *const anyopaque,
    _flush: *const anyopaque,
    _get_location: *const fn (
        *const PciIo,
        segment: *usize,
        bus: *usize,
        device: *usize,
        function: *usize,
    ) callconv(cc) Status,
    _attributes: *const anyopaque,
    _get_bar_attributes: *const fn (
        *const PciIo,
        bar_index: u8,
        supports: ?*u64,
        resources: ?*?*anyopaque,
    ) callconv(cc) Status,
    _set_bar_attributes: *const anyopaque,
    rom_size: u64,
    rom_image: ?*anyopaque,

    pub const guid align(8) = Guid{
        .time_low = 0x4cf5b200,
        .time_mid = 0x68b8,
        .time_high_and_version = 0x4ca5,
        .clock_seq_high_and_reserved = 0x9e,
        .clock_seq_low = 0xec,
        .node = [_]u8{ 0xb2, 0x3e, 0x3f, 0x50, 0x02, 0x9a },
    };

    pub const Location = struct {
        segment: usize,
        bus: usize,
        device: usize,
        function: usize,
    };

    pub fn getLocation(self: *const PciIo) error{DeviceError}!Location {
        var loc: Location = undefined;
        switch (self._get_location(self, &loc.segment, &loc.bus, &loc.device, &loc.function)) {
            .success => return loc,
            else => return error.DeviceError,
        }
    }

    /// Read a single value of type `T` from PCI configuration space at `offset`.
    pub fn configRead(self: *const PciIo, comptime T: type, offset: u32) error{DeviceError}!T {
        const width: Width = switch (T) {
            u8 => .uint8,
            u16 => .uint16,
            u32 => .uint32,
            else => @compileError("unsupported config width"),
        };
        var value: T = 0;
        switch (self.pci.read(self, width, offset, 1, &value)) {
            .success => return value,
            else => return error.DeviceError,
        }
    }

    /// Resolve a BAR's base address and length via GetBarAttributes, which hands
    /// back an ACPI 2.0 QWORD Address Space Descriptor we parse. Returns null if
    /// the firmware reports no resource for the BAR.
    pub fn barInfo(self: *const PciIo, bar_index: u8) error{DeviceError}!?BarInfo {
        var resources: ?*anyopaque = null;
        switch (self._get_bar_attributes(self, bar_index, null, &resources)) {
            .success => {},
            else => return error.DeviceError,
        }
        const res = resources orelse return null;
        return parseQwordDescriptor(res);
    }
};

pub const BarInfo = struct {
    base: u64,
    size: u64,
};

/// ACPI 2.0 QWORD Address Space Descriptor (type 0x8A). UEFI GetBarAttributes
/// returns one (or an End tag 0x79 if the BAR is unused). The descriptor is
/// BYTE-PACKED with no alignment padding, so its u64 fields must be read at
/// their exact unaligned byte offsets. An `extern struct` is WRONG here: u16/u64
/// natural alignment shifts AddrRangeMin to byte 16 and AddrLen to byte 40
/// instead of the spec's 14 and 38, reading every address 2 bytes off and
/// returning garbage. Spec offsets: tag@0, len@1, restype@3, genflags@4,
/// typeflags@5, granularity@6, AddrRangeMin@14 (base), AddrRangeMax@22,
/// translation@30, AddrLen@38 (size).
const ACPI_QWORD_TAG: u8 = 0x8A;

fn parseQwordDescriptor(res: *anyopaque) ?BarInfo {
    const bytes: [*]const u8 = @ptrCast(res);
    if (bytes[0] != ACPI_QWORD_TAG) return null; // not a QWORD descriptor (e.g. end tag 0x79)
    const base = std.mem.readInt(u64, bytes[14..22], .little);
    const size = std.mem.readInt(u64, bytes[38..46], .little);
    return .{ .base = base, .size = size };
}
