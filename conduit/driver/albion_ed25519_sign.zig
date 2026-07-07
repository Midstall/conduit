//! Albion Ed25519-sign offload driver, over an `Mmio`. Drives the hardened
//! hardware sign engine so the RMM attestation firmware produces a realm
//! signature with the side-channel-protected secret scalar, instead of a
//! software Ed25519 sign. Operands are 32-byte LITTLE-ENDIAN, and word i (8-byte
//! strided) carries bytes[4*i..4*i+4].
//!
//! Register map (matches lib/src/crypto/ed25519_sign_mmio.dart):
//!   0x00 CTRL (W bit0 START)  0x08 STATUS (R: bit0 busy, bit1 done)
//!   0x40 SEED  0x80 A  0xC0 M  0x100 Z  (RW inputs, 8 words @ +8)
//!   0x140 R  0x180 S  (RO outputs, 8 words @ +8)

const std = @import("std");
const Mmio = @import("../mmio.zig");

pub const Ed25519Sign = struct {
    mmio: Mmio,

    const CTRL = 0x00;
    const STATUS = 0x08;
    const SEED = 0x40;
    const A = 0x80;
    const M = 0xC0;
    const Z = 0x100;
    const R = 0x140;
    const S = 0x180;

    fn writeOperand(self: Ed25519Sign, base: usize, val: [32]u8) void {
        var i: usize = 0;
        while (i < 8) : (i += 1) {
            const w = std.mem.readInt(u32, val[4 * i ..][0..4], .little);
            self.mmio.write(u32, base + 8 * i, w);
        }
    }

    fn readOperand(self: Ed25519Sign, base: usize, out: *[32]u8) void {
        var i: usize = 0;
        while (i < 8) : (i += 1) {
            const w = self.mmio.read(u32, base + 8 * i);
            std.mem.writeInt(u32, out[4 * i ..][0..4], w, .little);
        }
    }

    /// Sign a 32-byte message with the realm key, returning `(R, S)` (the 64-byte
    /// Ed25519 signature is `R || S`). `seed` is the private seed, `pubkey` the
    /// public-key encoding, `z` fresh hedge entropy.
    pub fn sign(
        self: Ed25519Sign,
        seed: [32]u8,
        pubkey: [32]u8,
        msg: [32]u8,
        z: [32]u8,
        r_out: *[32]u8,
        s_out: *[32]u8,
    ) void {
        self.writeOperand(SEED, seed);
        self.writeOperand(A, pubkey);
        self.writeOperand(M, msg);
        self.writeOperand(Z, z);
        self.mmio.write(u32, CTRL, 1); // START
        while (self.mmio.read(u32, STATUS) & 0x1 != 0) {} // wait for !busy
        self.readOperand(R, r_out);
        self.readOperand(S, s_out);
    }
};
