//! The block-device contract. Deliberately the same shape as Weir's existing
//! `block.Device` (so GPT/FAT/virtio keep working when Weir re-exports this),
//! with optional `writeBlocks` added.

const Block = @This();

ctx: ?*anyopaque,
block_size: u32,
num_blocks: u64,
read_blocks: *const fn (ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]u8) bool,
write_blocks: ?*const fn (ctx: ?*anyopaque, lba: u64, count: u32, buf: [*]const u8) bool = null,

/// Read `count` blocks at `lba` into `buf` (>= count*block_size bytes).
pub fn readBlocks(self: *const Block, lba: u64, count: u32, buf: []u8) bool {
    return self.read_blocks(self.ctx, lba, count, buf.ptr);
}

/// Write `count` blocks at `lba` from `buf`. Returns false if the device is
/// read-only (no `write_blocks`).
pub fn writeBlocks(self: *const Block, lba: u64, count: u32, buf: []const u8) bool {
    const f = self.write_blocks orelse return false;
    return f(self.ctx, lba, count, buf.ptr);
}

pub fn writable(self: *const Block) bool {
    return self.write_blocks != null;
}
