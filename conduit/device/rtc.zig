//! The real-time-clock device-class contract.

const Rtc = @This();

pub const DateTime = struct {
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
};

ctx: ?*anyopaque,
now_fn: *const fn (ctx: ?*anyopaque) DateTime,
set_fn: ?*const fn (ctx: ?*anyopaque, dt: DateTime) void = null,

pub fn now(self: Rtc) DateTime {
    return self.now_fn(self.ctx);
}

/// Set the clock. Returns false if the RTC is read-only.
pub fn set(self: Rtc, dt: DateTime) bool {
    const f = self.set_fn orelse return false;
    f(self.ctx, dt);
    return true;
}

/// Convert Unix time (seconds since 1970-01-01 UTC) to a UTC `DateTime`.
/// Uses Howard Hinnant's civil-from-days algorithm.
pub fn fromUnix(secs: i64) DateTime {
    const days = @divFloor(secs, 86400);
    var rem = @mod(secs, 86400);
    const hour = @divTrunc(rem, 3600);
    rem -= hour * 3600;
    const minute = @divTrunc(rem, 60);
    const second = rem - minute * 60;

    const z = days + 719468;
    const era = @divFloor(if (z >= 0) z else z - 146096, 146097);
    const doe = z - era * 146097; // [0, 146096]
    const yoe = @divTrunc(doe - @divTrunc(doe, 1460) + @divTrunc(doe, 36524) - @divTrunc(doe, 146096), 365);
    var y = yoe + era * 400;
    const doy = doe - (365 * yoe + @divTrunc(yoe, 4) - @divTrunc(yoe, 100));
    const mp = @divTrunc(5 * doy + 2, 153); // [0, 11]
    const d = doy - @divTrunc(153 * mp + 2, 5) + 1; // [1, 31]
    const m = if (mp < 10) mp + 3 else mp - 9; // [1, 12]
    if (m <= 2) y += 1;

    return .{
        .year = @intCast(y),
        .month = @intCast(m),
        .day = @intCast(d),
        .hour = @intCast(hour),
        .minute = @intCast(minute),
        .second = @intCast(second),
    };
}

/// Convert a UTC `DateTime` to Unix time (seconds since 1970-01-01 UTC).
pub fn toUnix(dt: DateTime) i64 {
    const y0: i64 = @as(i64, dt.year) - @as(i64, if (dt.month <= 2) 1 else 0);
    const era = @divFloor(if (y0 >= 0) y0 else y0 - 399, 400);
    const yoe = y0 - era * 400;
    const mp: i64 = if (dt.month > 2) @as(i64, dt.month) - 3 else @as(i64, dt.month) + 9;
    const doy = @divTrunc(153 * mp + 2, 5) + @as(i64, dt.day) - 1;
    const doe = yoe * 365 + @divTrunc(yoe, 4) - @divTrunc(yoe, 100) + doy;
    const days = era * 146097 + doe - 719468;
    return days * 86400 + @as(i64, dt.hour) * 3600 + @as(i64, dt.minute) * 60 + @as(i64, dt.second);
}

test "epoch conversion round-trips a known timestamp" {
    const std = @import("std");
    const dt = fromUnix(1609459200); // 2021-01-01 00:00:00 UTC
    try std.testing.expectEqual(@as(u16, 2021), dt.year);
    try std.testing.expectEqual(@as(u8, 1), dt.month);
    try std.testing.expectEqual(@as(u8, 1), dt.day);
    try std.testing.expectEqual(@as(u8, 0), dt.hour);
    try std.testing.expectEqual(@as(i64, 1609459200), toUnix(dt));
}

pub fn from(impl: anytype) Rtc {
    const P = @TypeOf(impl);
    const gen = struct {
        fn now(ctx: ?*anyopaque) DateTime {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.now();
        }
        fn set(ctx: ?*anyopaque, dt: DateTime) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.set(dt);
        }
    };
    return .{ .ctx = impl, .now_fn = gen.now, .set_fn = gen.set };
}
