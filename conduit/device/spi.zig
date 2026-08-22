//! The SPI controller device-class contract.

const Spi = @This();

ctx: ?*anyopaque,
/// Full-duplex transfer: clock out `tx` while clocking in to `rd`. The LONGER
/// of the two bounds the exchange, so a caller reads more than it writes by
/// passing a short `tx`. Past the end of `tx` the driver clocks its idle byte,
/// and it fills `rd` only within `rd`'s bounds.
transfer_fn: *const fn (ctx: ?*anyopaque, tx: []const u8, rd: []u8) void,

pub fn transfer(self: Spi, tx: []const u8, rd: []u8) void {
    self.transfer_fn(self.ctx, tx, rd);
}

pub fn from(impl: anytype) Spi {
    const P = @TypeOf(impl);
    const gen = struct {
        fn transfer(ctx: ?*anyopaque, tx: []const u8, rd: []u8) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.transfer(tx, rd);
        }
    };
    return .{ .ctx = impl, .transfer_fn = gen.transfer };
}
