//! Albion RoT-KDF driver, over an `Mmio`. It derives keys from the FUSED ROOT in
//! hardware. The firmware writes only a 256-bit `info` domain label. It reads the
//! 512-bit derived key (`OKM = HKDF-SHA512(root, info)`). So the raw root never
//! transits software. The attestation firmware derives its sealing key and key
//! seeds here. It then uses them with the seal engine.
//!
//! Register map (8-byte strided, matches lib/src/crypto/rot_kdf_mmio.dart):
//!   0x00 CTRL (W bit0 START)  0x08 STATUS (R: bit0 busy, bit1 done)
//!   0x40 INFO (RW, 8 words @ +8)  0x80 OKM (RO, 16 words @ +8)

const std = @import("std");
const Mmio = @import("../mmio.zig");

pub const RotKdf = struct {
    mmio: Mmio,

    const CTRL = 0x00;
    const STATUS = 0x08;
    const INFO = 0x40;
    const OKM = 0x80;

    /// Derive the full 512-bit key for a 256-bit `info` label.
    pub fn derive(self: RotKdf, info: [32]u8, out: *[64]u8) void {
        var i: usize = 0;
        while (i < 8) : (i += 1) {
            const w = std.mem.readInt(u32, info[4 * i ..][0..4], .little);
            self.mmio.write(u32, INFO + 8 * i, w);
        }
        self.mmio.write(u32, CTRL, 1); // START
        while (self.mmio.read(u32, STATUS) & 0x1 != 0) {} // wait until not busy
        i = 0;
        while (i < 16) : (i += 1) {
            const w = self.mmio.read(u32, OKM + 8 * i);
            std.mem.writeInt(u32, out[4 * i ..][0..4], w, .little);
        }
    }

    /// Derive a 32-byte key (the first 256 bits of the OKM) for a label.
    pub fn deriveKey(self: RotKdf, info: [32]u8, out: *[32]u8) void {
        var full: [64]u8 = undefined;
        self.derive(info, &full);
        @memcpy(out, full[0..32]);
    }
};
