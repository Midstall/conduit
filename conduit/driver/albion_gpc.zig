//! Albion GPC driver: drives the granule-protection reference monitor's control
//! ops over an `Mmio`. The RMM firmware uses this to run the realm lifecycle
//! (create/assign/program-stream/destroy) and to turn enforcement on.
//!
//! Register map (8-byte strided, matches lib/src/gpc/gpc.dart):
//!   0x00 ARG_REALM (W)   0x08 ARG_GRAN (W)   0x10 ARG_STREAM (W)
//!   0x18 ARG_WORLD (W)   0x20 CMD (W)        0x28 RESULT (R: bit0 busy, [7:4] code)
//!   0x30 FAULT (R)       0x38 FAULT_ADDR (R) 0x40 ENABLE (RW: bit0)

const Mmio = @import("../mmio.zig");

pub const Gpc = struct {
    mmio: Mmio,

    const ARG_REALM = 0x00;
    const ARG_GRAN = 0x08;
    const ARG_STREAM = 0x10;
    const ARG_WORLD = 0x18;
    const CMD = 0x20;
    const RESULT = 0x28;
    const FAULT = 0x30;
    const FAULT_ADDR = 0x38;
    const ENABLE = 0x40;

    const OP_PROGRAM_STREAM = 1;
    const OP_CREATE_REALM = 2;
    const OP_ASSIGN_GRANULE = 3;
    const OP_SHARE = 4;
    const OP_RECLAIM = 5;
    const OP_DESTROY = 6;

    /// Worlds (mirror the hardware).
    pub const world_normal: u8 = 0;
    pub const world_realm: u8 = 1;
    pub const world_root: u8 = 2;

    /// Result code returned in RESULT[7:4].
    pub const Result = enum(u8) {
        ok = 0,
        busy = 1,
        range = 2,
        not_live = 3,
        not_free = 4,
        bad_state = 5,
        _,
    };

    /// Submit one op. Stage the args, write CMD, then wait for the busy bit to
    /// clear. Reclaim and destroy run a bounded scrub. Return the result code.
    fn submit(self: Gpc, op: u32, realm: u32, gran: u32, stream: u32, world: u32) Result {
        self.mmio.write(u32, ARG_REALM, realm);
        self.mmio.write(u32, ARG_GRAN, gran);
        self.mmio.write(u32, ARG_STREAM, stream);
        self.mmio.write(u32, ARG_WORLD, world);
        self.mmio.write(u32, CMD, op);
        while (self.mmio.read(u32, RESULT) & 0x1 != 0) {}
        return @enumFromInt(@as(u8, @truncate((self.mmio.read(u32, RESULT) >> 4) & 0xf)));
    }

    pub fn createRealm(self: Gpc, realm: u8) Result {
        return self.submit(OP_CREATE_REALM, realm, 0, 0, 0);
    }
    pub fn assignGranule(self: Gpc, granule: u32, realm: u8) Result {
        return self.submit(OP_ASSIGN_GRANULE, realm, granule, 0, 0);
    }
    pub fn programStream(self: Gpc, stream: u32, realm: u8, world: u8) Result {
        return self.submit(OP_PROGRAM_STREAM, realm, 0, stream, world);
    }
    pub fn shareGranule(self: Gpc, granule: u32) Result {
        return self.submit(OP_SHARE, 0, granule, 0, 0);
    }
    pub fn reclaimGranule(self: Gpc, granule: u32) Result {
        return self.submit(OP_RECLAIM, 0, granule, 0, 0);
    }
    pub fn destroyRealm(self: Gpc, realm: u8) Result {
        return self.submit(OP_DESTROY, realm, 0, 0, 0);
    }

    /// Turn enforcement on/off (0 = bypass, the reset default).
    pub fn setEnable(self: Gpc, on: bool) void {
        self.mmio.write(u32, ENABLE, if (on) @as(u32, 1) else 0);
    }

    /// Read the latched fault descriptor (valid in bit0).
    pub fn fault(self: Gpc) u32 {
        return self.mmio.read(u32, FAULT);
    }
};
