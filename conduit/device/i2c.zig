//! The I2C controller device-class contract.

const I2c = @This();

ctx: ?*anyopaque,
/// Write `wr` then read into `rd` (either may be empty) for 7/10-bit `addr`.
/// Returns false on NAK / bus error.
transfer_fn: *const fn (ctx: ?*anyopaque, addr: u16, wr: []const u8, rd: []u8) bool,

pub fn transfer(self: I2c, addr: u16, wr: []const u8, rd: []u8) bool {
    return self.transfer_fn(self.ctx, addr, wr, rd);
}

pub fn from(impl: anytype) I2c {
    const P = @TypeOf(impl);
    const gen = struct {
        fn transfer(ctx: ?*anyopaque, addr: u16, wr: []const u8, rd: []u8) bool {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.transfer(addr, wr, rd);
        }
    };
    return .{ .ctx = impl, .transfer_fn = gen.transfer };
}
