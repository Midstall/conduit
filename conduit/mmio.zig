//! `Mmio`: the register-access seam that lets one driver source run in both
//! Weir (raw physical pointer, M-mode, identity-mapped) and Ferrite (a virtual
//! mapping minted by SYS_MMIO_CREATE + mmap, or x86 port-IO).
//!
//! A driver never holds a raw `usize` base. It takes an `Mmio` and pokes
//! registers through `read`/`write`. The consumer supplies the implementation:
//! `Mmio.direct(base)` for an identity-mapped volatile pointer, or a custom
//! `ctx`/`read_fn`/`write_fn` for anything else.
//!
//! The shape (ctx + read/write fn) deliberately matches almanac's
//! `RegionHandler`, so an `Mmio` adapts into an AML `OperationRegion` handler
//! with a trivial wrapper.

const Mmio = @This();

pub const Width = enum { byte, half, word, dword };

ctx: ?*anyopaque,
/// Bus/virtual base the ctx understands. Informational for most impls; the
/// `direct` impl carries the base inside `ctx` so register offsets resolve
/// without a separate allocation.
base: u64,
read_fn: *const fn (ctx: ?*anyopaque, off: usize, width: Width) u64,
write_fn: *const fn (ctx: ?*anyopaque, off: usize, width: Width, val: u64) void,

pub inline fn read(self: Mmio, comptime T: type, off: usize) T {
    return @truncate(self.read_fn(self.ctx, off, widthOf(T)));
}

pub inline fn write(self: Mmio, comptime T: type, off: usize, v: T) void {
    self.write_fn(self.ctx, off, widthOf(T), v);
}

/// An identity-mapped, volatile-pointer accessor. The base is stashed in `ctx`
/// so reads/writes resolve to `base + off` with zero overhead. This is Weir's
/// M-mode path.
pub fn direct(base: u64) Mmio {
    return .{
        .ctx = @ptrFromInt(base),
        .base = base,
        .read_fn = directRead,
        .write_fn = directWrite,
    };
}

fn directRead(ctx: ?*anyopaque, off: usize, width: Width) u64 {
    const addr = @intFromPtr(ctx) + off;
    return switch (width) {
        .byte => @as(*volatile u8, @ptrFromInt(addr)).*,
        .half => @as(*volatile u16, @ptrFromInt(addr)).*,
        .word => @as(*volatile u32, @ptrFromInt(addr)).*,
        .dword => @as(*volatile u64, @ptrFromInt(addr)).*,
    };
}

fn directWrite(ctx: ?*anyopaque, off: usize, width: Width, val: u64) void {
    const addr = @intFromPtr(ctx) + off;
    switch (width) {
        .byte => @as(*volatile u8, @ptrFromInt(addr)).* = @truncate(val),
        .half => @as(*volatile u16, @ptrFromInt(addr)).* = @truncate(val),
        .word => @as(*volatile u32, @ptrFromInt(addr)).* = @truncate(val),
        .dword => @as(*volatile u64, @ptrFromInt(addr)).* = val,
    }
}

fn widthOf(comptime T: type) Width {
    return switch (@sizeOf(T)) {
        1 => .byte,
        2 => .half,
        4 => .word,
        8 => .dword,
        else => @compileError("Mmio access type must be 1/2/4/8 bytes, got " ++ @typeName(T)),
    };
}
