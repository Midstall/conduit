//! Albion Ed25519-verify offload driver, over an `Mmio`.
//!
//! The hardware signature-verify engine (`AlbionEd25519Verify`, default window
//! 0x4000_C000): a full RFC 8032 §5.1.7 Ed25519 verifier in silicon. The SEP
//! firmware (Ferrite's signature path) and the secure-boot ROM drive it to
//! verify a signature in hardware instead of running the Curve25519 double-
//! scalar multiply in software (hundreds of millions of cycles on the in-order
//! SEP). The engine fixes the message to a 32-byte digest (the SHA-256 prehash
//! Ferrite signs) and computes `SHA-512(R || A || M)` itself.
//!
//! 32-bit word accesses. Registers are 8-BYTE-STRIDED (each on the low data
//! lanes) because the Albion SEP's 64-bit master places a 32-bit store on the
//! lane chosen by the address, so an 8-aligned offset keeps every register on the
//! low lanes and distinct by ADR. Each 256-bit operand is eight LITTLE-ENDIAN
//! 32-bit words: word `i` holds operand bytes `[4i .. 4i+4]` (byte `4i` = the
//! word's low byte), the on-wire Ed25519 LE encoding read four bytes at a time.
//!   CTRL   0x00 (W):       bit0 = START (ignored unless the engine is idle).
//!   STATUS 0x08 (RO):      bit0 = busy, bit1 = done (sticky), bit2 = accept.
//!   A[i]   0x40 + 8*i (RW, i in 0..7): public key (32-byte LE encoding).
//!   R[i]   0x80 + 8*i (RW, i in 0..7): signature point R (32-byte LE).
//!   S[i]   0xC0 + 8*i (RW, i in 0..7): signature scalar S (32-byte LE).
//!   M[i]   0x100 + 8*i (RW, i in 0..7): message digest M (32 bytes).

const std = @import("std");
const Mmio = @import("../mmio.zig");

pub const Ed25519 = struct {
    mmio: Mmio,

    const CTRL = 0x00;
    const STATUS = 0x08;
    const STRIDE = 0x08; // 8-byte register stride (low-lane convention).
    const A = 0x40;
    const R = 0x80;
    const S = 0xC0;
    const M = 0x100;

    const STATUS_BUSY: u32 = 1 << 0;
    const STATUS_DONE: u32 = 1 << 1;
    const STATUS_ACCEPT: u32 = 1 << 2;

    /// True while a verify is in flight.
    pub fn busy(self: Ed25519) bool {
        return (self.mmio.read(u32, STATUS) & STATUS_BUSY) != 0;
    }

    /// Load a 32-byte operand (eight LE words) into the window at `base`.
    fn loadOperand(self: Ed25519, base: usize, operand: [32]u8) void {
        for (0..8) |i| {
            const w = std.mem.readInt(u32, operand[4 * i ..][0..4], .little);
            self.mmio.write(u32, base + STRIDE * i, w);
        }
    }

    /// Verify an Ed25519 signature in hardware. `pubkey`/`r`/`s` are the on-wire
    /// 32-byte LITTLE-ENDIAN encodings (the signature is `r || s`), and `msg` is the
    /// 32-byte digest that was signed. Loads the operands, launches the engine,
    /// blocks until it completes, and returns true iff the signature verifies.
    pub fn verify(self: Ed25519, pubkey: [32]u8, r: [32]u8, s: [32]u8, msg: [32]u8) bool {
        self.loadOperand(A, pubkey);
        self.loadOperand(R, r);
        self.loadOperand(S, s);
        self.loadOperand(M, msg);
        self.mmio.write(u32, CTRL, 1);
        var st = self.mmio.read(u32, STATUS);
        while ((st & STATUS_DONE) == 0) st = self.mmio.read(u32, STATUS);
        return (st & STATUS_ACCEPT) != 0;
    }

    /// Convenience wrapper taking a 64-byte signature (`r || s`).
    pub fn verifySig(self: Ed25519, pubkey: [32]u8, sig: [64]u8, msg: [32]u8) bool {
        return self.verify(pubkey, sig[0..32].*, sig[32..64].*, msg);
    }
};

pub fn bind(mmio: Mmio) Ed25519 {
    return .{ .mmio = mmio };
}
