//! The GPIO device-class contract.

const Gpio = @This();

pub const Dir = enum { input, output };

ctx: ?*anyopaque,
set_fn: *const fn (ctx: ?*anyopaque, pin: u32, value: bool) void,
get_fn: *const fn (ctx: ?*anyopaque, pin: u32) bool,
dir_fn: *const fn (ctx: ?*anyopaque, pin: u32, dir: Dir) void,

pub fn set(self: Gpio, pin: u32, value: bool) void {
    self.set_fn(self.ctx, pin, value);
}

pub fn get(self: Gpio, pin: u32) bool {
    return self.get_fn(self.ctx, pin);
}

pub fn direction(self: Gpio, pin: u32, dir: Dir) void {
    self.dir_fn(self.ctx, pin, dir);
}

pub fn from(impl: anytype) Gpio {
    const P = @TypeOf(impl);
    const gen = struct {
        fn set(ctx: ?*anyopaque, pin: u32, value: bool) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.set(pin, value);
        }
        fn get(ctx: ?*anyopaque, pin: u32) bool {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.get(pin);
        }
        fn dir(ctx: ?*anyopaque, pin: u32, d: Dir) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.direction(pin, d);
        }
    };
    return .{ .ctx = impl, .set_fn = gen.set, .get_fn = gen.get, .dir_fn = gen.dir };
}
