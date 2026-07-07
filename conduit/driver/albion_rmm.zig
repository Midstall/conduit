//! Albion RMM command-channel driver. Two views over one channel:
//!   - `Rmm`  : the Root/SEP side, where the RMM firmware drains commands the AP
//!     submitted and pushes responses back (SEP/Root window 0x40-0x7F).
//!   - `Host` : the AP side, where a VMM submits typed realm-management commands
//!     and reads responses (the firewall-whitelisted AP window 0x00-0x3F). This
//!     is the AP-side Realm Management Interface.
//!
//! SEP/Root window (8-byte strided, matches lib/src/rmm/rmm.dart):
//!   0x40 CMD_HEAD (R: bit0 valid, [..:1] {op,realm})  0x48 CMD_A0 (R)
//!   0x50 CMD_A1 (R)   0x58 CMD_POP (W)   0x60 RSP_ST (W)   0x68 RSP_D (W)
//!   0x70 RSP_PUSH (W) 0x78 SEP_STAT (R)
//! AP window:
//!   0x00 CMD_OP (W: {op|realm<<8})  0x08 ARG0 (W)  0x10 ARG1 (W)  0x18 SUBMIT (W)
//!   0x20 AP_STAT (R: bit0 full, bit1 rsp)  0x28 RSP_HEAD (R: bit0 valid, [8:1] st)
//!   0x30 RSP_DATA (R)  0x38 RSP_POP (W)

const std = @import("std");
const Mmio = @import("../mmio.zig");

const CMD_OP = 0x00;
const CMD_ARG0 = 0x08;
const CMD_ARG1 = 0x10;
const SUBMIT = 0x18;
const AP_STAT = 0x20;
const RSP_HEAD = 0x28;
const RSP_DATA = 0x30;
const RSP_POP = 0x38;

const OP_CREATE = 1;
const OP_DELEGATE = 2;
const OP_ACTIVATE = 3;
const OP_RUN = 4;
const OP_EXIT = 5;
const OP_DESTROY = 6;
const OP_ATTEST = 7;
const OP_GET_TOKEN_WORD = 8;
const OP_SHARE_GRANULE = 9;
const OP_UNSHARE_GRANULE = 10;
const OP_EXTEND_REM = 11;
const OP_ALLOW_IRQ = 12;
const OP_DENY_IRQ = 13;

pub const Status = enum(u8) {
    ok = 0,
    err_realm = 1,
    err_state = 2,
    err_gpc = 3,
    err_op = 4,
    err_denied = 5,
    _,
};

pub const Rmm = struct {
    mmio: Mmio,

    const CMD_HEAD = 0x40;
    const CMD_A0 = 0x48;
    const CMD_A1 = 0x50;
    const CMD_POP = 0x58;
    const RSP_ST = 0x60;
    const RSP_D = 0x68;
    const RSP_PUSH = 0x70;
    const SEP_STAT = 0x78;

    /// A drained command (op-code field is 8 bits, realm 4 bits, as packed).
    pub const Command = struct {
        op: u8,
        realm: u8,
        arg0: u32,
        arg1: u32,
    };

    /// Returns the head command if one is pending, else null. Does NOT pop.
    pub fn peek(self: Rmm) ?Command {
        const head = self.mmio.read(u32, CMD_HEAD);
        if (head & 0x1 == 0) return null;
        const opr = head >> 1;
        return .{
            .op = @truncate(opr & 0xff),
            .realm = @truncate((opr >> 8) & 0xf),
            .arg0 = self.mmio.read(u32, CMD_A0),
            .arg1 = self.mmio.read(u32, CMD_A1),
        };
    }

    /// Pop the head command (advance the command FIFO).
    pub fn pop(self: Rmm) void {
        self.mmio.write(u32, CMD_POP, 1);
    }

    /// Stage and enqueue a response for the AP.
    pub fn respond(self: Rmm, status: u8, data: u32) void {
        self.mmio.write(u32, RSP_ST, status);
        self.mmio.write(u32, RSP_D, data);
        self.mmio.write(u32, RSP_PUSH, 1);
    }
};

/// The AP-side Realm Management Interface: a host VMM submits typed realm
/// commands over the firewall-whitelisted AP window and reads the RMM's responses.
/// The VMM never touches realm memory, since the GPC/RMM enforce isolation in
/// hardware. It only orchestrates lifecycle here.
pub const Host = struct {
    mmio: Mmio,

    /// Bound on the busy-wait so a dead/absent channel cannot wedge the caller.
    const spin_limit: u32 = 1_000_000;

    pub const Error = error{ Full, Timeout };

    pub const Response = struct {
        status: Status,
        data: u32,
    };

    /// Submit one command and return its response.
    fn submit(self: Host, op: u32, realm: u8, arg0: u32, arg1: u32) Error!Response {
        var spins: u32 = 0;
        while (self.mmio.read(u32, AP_STAT) & 0x1 != 0) {
            spins += 1;
            if (spins >= spin_limit) return Error.Full;
        }
        self.mmio.write(u32, CMD_OP, op | (@as(u32, realm) << 8));
        self.mmio.write(u32, CMD_ARG0, arg0);
        self.mmio.write(u32, CMD_ARG1, arg1);
        self.mmio.write(u32, SUBMIT, 1);
        spins = 0;
        while (self.mmio.read(u32, AP_STAT) & 0x2 == 0) {
            spins += 1;
            if (spins >= spin_limit) return Error.Timeout;
        }
        const head = self.mmio.read(u32, RSP_HEAD);
        const data = self.mmio.read(u32, RSP_DATA);
        self.mmio.write(u32, RSP_POP, 1);
        return .{
            .status = @enumFromInt(@as(u8, @truncate((head >> 1) & 0xff))),
            .data = data,
        };
    }

    pub fn createRealm(self: Host, realm: u8) Error!Status {
        return (try self.submit(OP_CREATE, realm, 0, 0)).status;
    }
    pub fn delegateGranule(self: Host, realm: u8, granule: u32) Error!Status {
        return (try self.submit(OP_DELEGATE, realm, granule, 0)).status;
    }
    pub fn activateRealm(self: Host, realm: u8) Error!Status {
        return (try self.submit(OP_ACTIVATE, realm, 0, 0)).status;
    }
    pub fn runRealm(self: Host, realm: u8, stream: u32) Error!Status {
        return (try self.submit(OP_RUN, realm, stream, 0)).status;
    }
    pub fn exitNotify(self: Host, stream: u32) Error!Status {
        return (try self.submit(OP_EXIT, 0, stream, 0)).status;
    }
    pub fn destroyRealm(self: Host, realm: u8) Error!Status {
        return (try self.submit(OP_DESTROY, realm, 0, 0)).status;
    }
    /// Request an attestation token for an ACTIVE realm. The response data is the
    /// produced token length. Follow with [readToken] to pull the bytes.
    pub fn getAttestationToken(self: Host, realm: u8, challenge: u32) Error!Response {
        return self.submit(OP_ATTEST, realm, challenge, 0);
    }

    /// Read the `word`-th 32-bit word of the last produced token (data plane).
    pub fn getTokenWord(self: Host, word: u32) Error!Response {
        return self.submit(OP_GET_TOKEN_WORD, 0, word, 0);
    }

    /// Mark an ACTIVE realm's granule shared with the host (CoVE COVG share).
    pub fn shareGranule(self: Host, realm: u8, granule: u32) Error!Status {
        return (try self.submit(OP_SHARE_GRANULE, realm, granule, 0)).status;
    }

    /// Return a shared granule to the realm (private again).
    pub fn unshareGranule(self: Host, realm: u8, granule: u32) Error!Status {
        return (try self.submit(OP_UNSHARE_GRANULE, realm, granule, 0)).status;
    }

    /// Extend an ACTIVE realm's REM[rem] with a runtime measurement value.
    pub fn extendRem(self: Host, realm: u8, rem: u8, value: u32) Error!Status {
        return (try self.submit(OP_EXTEND_REM, realm, rem, value)).status;
    }

    /// Realm interrupt allow-list: the realm permits/revokes which interrupts the
    /// host may inject. The RMM mirrors this into the hardware COVI gate, which
    /// enforces injection inline (via the AP-facing injection aperture).
    pub fn allowInterrupt(self: Host, realm: u8, irq: u32) Error!Status {
        return (try self.submit(OP_ALLOW_IRQ, realm, irq, 0)).status;
    }
    pub fn denyInterrupt(self: Host, realm: u8, irq: u32) Error!Status {
        return (try self.submit(OP_DENY_IRQ, realm, irq, 0)).status;
    }

    /// Pull `out.len` bytes of the last produced token (call after
    /// [getAttestationToken] told you the length). Returns bytes read.
    pub fn readToken(self: Host, out: []u8) Error!usize {
        var i: usize = 0;
        var word: u32 = 0;
        while (i < out.len) : (word += 1) {
            const w = (try self.getTokenWord(word)).data;
            var b: usize = 0;
            const n = @min(@as(usize, 4), out.len - i);
            while (b < n) : (b += 1) {
                out[i + b] = @truncate(w >> @intCast(8 * b));
            }
            i += n;
        }
        return i;
    }
};

// A software model of the channel plus a minimal RMM lifecycle, so the AP-side
// Host round-trips against it. This mirrors the SEP-side RMM gating.
const MockRmm = struct {
    op: u32 = 0,
    arg0: u32 = 0,
    arg1: u32 = 0,
    rsp_valid: bool = false,
    rsp_status: u8 = 0,
    rsp_data: u32 = 0,
    states: [16]u8 = [_]u8{0} ** 16, // 0 null, 1 configuring, 2 active

    fn process(m: *MockRmm) void {
        const op = m.op & 0xff;
        const realm = (m.op >> 8) & 0xf;
        const st = &m.states[realm];
        var status: u8 = 4;
        var data: u32 = 0;
        switch (op) {
            OP_CREATE => {
                if (realm == 0) status = 1 else if (st.* != 0) status = 2 else {
                    st.* = 1;
                    status = 0;
                }
            },
            OP_DELEGATE => status = if (st.* != 1) 2 else 0,
            OP_ACTIVATE => if (st.* != 1) {
                status = 2;
            } else {
                st.* = 2;
                status = 0;
            },
            OP_RUN => status = if (st.* != 2) 2 else 0,
            OP_EXIT => status = 0,
            OP_DESTROY => if (st.* == 0) {
                status = 2;
            } else {
                st.* = 0;
                status = 0;
            },
            OP_ATTEST => if (st.* != 2) {
                status = 2;
            } else {
                status = 0;
                data = 250;
            },
            OP_GET_TOKEN_WORD => {
                status = 0;
                data = 0xBB00_0000 | (m.arg0 & 0xffff); // deterministic per word.
            },
            OP_SHARE_GRANULE, OP_UNSHARE_GRANULE => status = if (st.* != 2) 2 else 0,
            OP_EXTEND_REM => status = if (st.* != 2) 2 else if (m.arg0 >= 4) 4 else 0,
            OP_ALLOW_IRQ, OP_DENY_IRQ => status = if (st.* != 2) 2 else if (m.arg0 >= 32) 4 else 0,
            else => status = 4,
        }
        m.rsp_status = status;
        m.rsp_data = data;
        m.rsp_valid = true;
    }

    fn rd(ctx: ?*anyopaque, off: usize, width: Mmio.Width) u64 {
        _ = width;
        const m: *MockRmm = @ptrCast(@alignCast(ctx.?));
        return switch (off) {
            AP_STAT => if (m.rsp_valid) 0x2 else 0,
            RSP_HEAD => if (m.rsp_valid) (@as(u32, m.rsp_status) << 1) | 1 else 0,
            RSP_DATA => m.rsp_data,
            else => 0,
        };
    }
    fn wr(ctx: ?*anyopaque, off: usize, width: Mmio.Width, val: u64) void {
        _ = width;
        const m: *MockRmm = @ptrCast(@alignCast(ctx.?));
        switch (off) {
            CMD_OP => m.op = @truncate(val),
            CMD_ARG0 => m.arg0 = @truncate(val),
            CMD_ARG1 => m.arg1 = @truncate(val),
            SUBMIT => m.process(),
            RSP_POP => m.rsp_valid = false,
            else => {},
        }
    }
    fn mmio(self: *MockRmm) Mmio {
        return .{ .ctx = self, .base = 0, .read_fn = rd, .write_fn = wr };
    }
};

test "AP-side Host round-trips the full realm lifecycle" {
    var mock = MockRmm{};
    const host = Host{ .mmio = mock.mmio() };

    try std.testing.expectEqual(Status.ok, try host.createRealm(3));
    try std.testing.expectEqual(Status.ok, try host.delegateGranule(3, 0));
    try std.testing.expectEqual(Status.ok, try host.delegateGranule(3, 1));
    try std.testing.expectEqual(Status.ok, try host.activateRealm(3));
    try std.testing.expectEqual(Status.ok, try host.runRealm(3, 1));
    const tok = try host.getAttestationToken(3, 0xC0FFEE);
    try std.testing.expectEqual(Status.ok, tok.status);
    try std.testing.expectEqual(@as(u32, 250), tok.data); // token length.
    // active realm shares a granule with the host, then reclaims it.
    try std.testing.expectEqual(Status.ok, try host.shareGranule(3, 0));
    try std.testing.expectEqual(Status.ok, try host.unshareGranule(3, 0));
    // and records a runtime measurement into REM2.
    try std.testing.expectEqual(Status.ok, try host.extendRem(3, 2, 0xABCD));
    try std.testing.expectEqual(Status.err_op, try host.extendRem(3, 9, 0)); // bad REM index
    try std.testing.expectEqual(Status.ok, try host.exitNotify(1));
    try std.testing.expectEqual(Status.ok, try host.destroyRealm(3));
}

test "AP-side Host pulls the attestation token over the data plane" {
    var mock = MockRmm{};
    const host = Host{ .mmio = mock.mmio() };

    try std.testing.expectEqual(Status.ok, try host.createRealm(2));
    try std.testing.expectEqual(Status.ok, try host.activateRealm(2));
    const tok = try host.getAttestationToken(2, 0);
    try std.testing.expectEqual(@as(u32, 250), tok.data); // token length.

    // Individual words are addressable by offset.
    try std.testing.expectEqual(@as(u32, 0xBB00_0000), (try host.getTokenWord(0)).data);
    try std.testing.expectEqual(@as(u32, 0xBB00_0005), (try host.getTokenWord(5)).data);

    // readToken reconstructs the bytes (little-endian per word).
    var buf: [8]u8 = undefined;
    try std.testing.expectEqual(@as(usize, 8), try host.readToken(&buf));
    try std.testing.expectEqualSlices(
        u8,
        &[_]u8{ 0x00, 0x00, 0x00, 0xBB, 0x01, 0x00, 0x00, 0xBB },
        &buf,
    );
}

test "AP-side Host programs a realm's interrupt allow-list" {
    var mock = MockRmm{};
    const host = Host{ .mmio = mock.mmio() };
    try std.testing.expectEqual(Status.ok, try host.createRealm(2));
    try std.testing.expectEqual(Status.ok, try host.activateRealm(2));

    // the realm permits + revokes interrupts (the RMM mirrors this into the gate).
    try std.testing.expectEqual(Status.ok, try host.allowInterrupt(2, 7));
    try std.testing.expectEqual(Status.ok, try host.denyInterrupt(2, 7));
    // allow on a non-active realm is rejected.
    try std.testing.expectEqual(Status.err_state, try host.allowInterrupt(5, 1));
}

test "AP-side Host sees the RMM enforce lifecycle gating" {
    var mock = MockRmm{};
    const host = Host{ .mmio = mock.mmio() };

    // run / attest before active, and destroy of nothing, are rejected.
    try std.testing.expectEqual(Status.err_state, try host.runRealm(5, 2));
    try std.testing.expectEqual(Status.err_state, (try host.getAttestationToken(5, 1)).status);
    try std.testing.expectEqual(Status.err_state, try host.destroyRealm(5));

    try std.testing.expectEqual(Status.ok, try host.createRealm(5));
    try std.testing.expectEqual(Status.err_state, try host.createRealm(5)); // double create
    try std.testing.expectEqual(Status.ok, try host.activateRealm(5));
    try std.testing.expectEqual(Status.err_state, try host.delegateGranule(5, 0)); // locked
    try std.testing.expectEqual(Status.err_realm, try host.createRealm(0)); // reserved
}
