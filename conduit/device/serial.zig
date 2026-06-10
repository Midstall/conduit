//! The serial / UART device-class contract. A vtable over any byte-stream UART.
//! Drivers (driver/ns16550a.zig, driver/pl011.zig) implement the small method
//! set and wrap themselves with `Serial.from`.

const Serial = @This();

ctx: ?*anyopaque,
write_fn: *const fn (ctx: ?*anyopaque, bytes: []const u8) void,
read_fn: *const fn (ctx: ?*anyopaque, buf: []u8) usize,
tx_ready_fn: *const fn (ctx: ?*anyopaque) bool,
rx_ready_fn: *const fn (ctx: ?*anyopaque) bool,

/// Write all bytes (blocking until each is accepted).
pub fn write(self: Serial, bytes: []const u8) void {
    self.write_fn(self.ctx, bytes);
}

/// Read currently-available bytes into `buf`; returns the count (may be 0).
pub fn read(self: Serial, buf: []u8) usize {
    return self.read_fn(self.ctx, buf);
}

/// True if the transmitter can accept a byte now.
pub fn txReady(self: Serial) bool {
    return self.tx_ready_fn(self.ctx);
}

/// True if a received byte is waiting.
pub fn rxReady(self: Serial) bool {
    return self.rx_ready_fn(self.ctx);
}

/// Wrap a driver pointer (with `write`/`read`/`txReady`/`rxReady` methods) into
/// a `Serial`.
pub fn from(impl: anytype) Serial {
    const P = @TypeOf(impl);
    const gen = struct {
        fn write(ctx: ?*anyopaque, bytes: []const u8) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.write(bytes);
        }
        fn read(ctx: ?*anyopaque, buf: []u8) usize {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.read(buf);
        }
        fn txReady(ctx: ?*anyopaque) bool {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.txReady();
        }
        fn rxReady(ctx: ?*anyopaque) bool {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.rxReady();
        }
    };
    return .{
        .ctx = impl,
        .write_fn = gen.write,
        .read_fn = gen.read,
        .tx_ready_fn = gen.txReady,
        .rx_ready_fn = gen.rxReady,
    };
}
