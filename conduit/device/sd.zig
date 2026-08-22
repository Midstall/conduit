//! SD/MMC card identity. The CID (Card IDentification) register holds the
//! manufacturer, product, serial number, and manufacture date. Every SD/MMC host
//! reads the same register, so the field layout lives here once. A driver
//! captures the CID at bring-up and hands it out as a `Cid`.

const std = @import("std");

/// The 16-byte CID in standard order: raw[0] is the most significant byte
/// (CID[127:120], the manufacturer ID) and raw[15] is the least significant byte
/// (the CRC). An SPI host reads these 16 bytes directly. A native host reads a
/// 4-word R2 response instead, so build the Cid with `fromR2`.
pub const Cid = struct {
    pub const Date = struct { year: u16, month: u8 };
    pub const Revision = struct { major: u8, minor: u8 };

    raw: [16]u8,

    /// Manufacturer ID (MID, CID byte 0). Well-known values: 0x01 Panasonic,
    /// 0x02 Toshiba/Kioxia, 0x03 SanDisk, 0x1b Samsung, 0x41 Kingston.
    pub fn manufacturerId(self: Cid) u8 {
        return self.raw[0];
    }

    /// OEM/application ID (OID, 2 ASCII characters, CID bytes 1..2).
    pub fn oemId(self: Cid) [2]u8 {
        return .{ self.raw[1], self.raw[2] };
    }

    /// Product name (PNM, 5 ASCII characters, CID bytes 3..7).
    pub fn productName(self: Cid) [5]u8 {
        return .{ self.raw[3], self.raw[4], self.raw[5], self.raw[6], self.raw[7] };
    }

    /// Product revision (PRV, CID byte 8): a major and a minor nibble.
    pub fn revision(self: Cid) Revision {
        return .{ .major = self.raw[8] >> 4, .minor = self.raw[8] & 0x0f };
    }

    /// Product serial number (PSN, CID bytes 9..12), a 32-bit value.
    pub fn serialNumber(self: Cid) u32 {
        return std.mem.readInt(u32, self.raw[9..13], .big);
    }

    /// Manufacture date (MDT, CID bits [19:8]) as (year, month).
    pub fn manufactureDate(self: Cid) Date {
        const mdt: u16 = (@as(u16, self.raw[13] & 0x0f) << 8) | self.raw[14];
        return .{ .year = 2000 + (mdt >> 4), .month = @intCast(mdt & 0x0f) };
    }

    /// Whether the register holds an identity. A read that returns all zeros
    /// (no card, or a host that does not surface the CID) has a 0 manufacturer.
    pub fn known(self: Cid) bool {
        return self.raw[0] != 0;
    }

    /// Build a Cid from a controller R2 response with Harbor's direct alignment:
    /// resp[i] holds CID bits [32*i+31 : 32*i], so resp[3] is CID[127:96]. The
    /// four words pack most-significant-first into the 16-byte raw.
    pub fn fromR2(resp: [4]u32) Cid {
        var raw: [16]u8 = undefined;
        std.mem.writeInt(u32, raw[0..4], resp[3], .big);
        std.mem.writeInt(u32, raw[4..8], resp[2], .big);
        std.mem.writeInt(u32, raw[8..12], resp[1], .big);
        std.mem.writeInt(u32, raw[12..16], resp[0], .big);
        return .{ .raw = raw };
    }
};

test "cid decodes fields from raw bytes" {
    // MID=0x03 (SanDisk), OID="SD", PNM="SU08G", PRV=8.0, PSN=0x12345678,
    // MDT year 2019 (0x13) month 8 -> mdt=0x138, bytes[13]=0x01 bytes[14]=0x38.
    const cid = Cid{ .raw = .{
        0x03, 'S',  'D',  'S',  'U',  '0',  '8',  'G',
        0x80, 0x12, 0x34, 0x56, 0x78, 0x01, 0x38, 0x00,
    } };
    try std.testing.expectEqual(@as(u8, 0x03), cid.manufacturerId());
    try std.testing.expectEqualSlices(u8, "SD", &cid.oemId());
    try std.testing.expectEqualSlices(u8, "SU08G", &cid.productName());
    try std.testing.expectEqual(@as(u8, 8), cid.revision().major);
    try std.testing.expectEqual(@as(u8, 0), cid.revision().minor);
    try std.testing.expectEqual(@as(u32, 0x12345678), cid.serialNumber());
    try std.testing.expectEqual(@as(u16, 2019), cid.manufactureDate().year);
    try std.testing.expectEqual(@as(u8, 8), cid.manufactureDate().month);
    try std.testing.expect(cid.known());
}

test "fromR2 direct alignment round-trips the raw CID" {
    const raw = [16]u8{
        0x03, 'S',  'D',  'S',  'U',  '0',  '8',  'G',
        0x80, 0x12, 0x34, 0x56, 0x78, 0x01, 0x38, 0x00,
    };
    // Pack the raw into four R2 words the way the controller stores them:
    // resp[3] is the top four bytes, resp[0] the bottom four.
    const resp = [4]u32{
        std.mem.readInt(u32, raw[12..16], .big),
        std.mem.readInt(u32, raw[8..12], .big),
        std.mem.readInt(u32, raw[4..8], .big),
        std.mem.readInt(u32, raw[0..4], .big),
    };
    const cid = Cid.fromR2(resp);
    try std.testing.expectEqualSlices(u8, &raw, &cid.raw);
    try std.testing.expectEqual(@as(u8, 0x03), cid.manufacturerId());
    try std.testing.expectEqual(@as(u32, 0x12345678), cid.serialNumber());
    try std.testing.expectEqual(@as(u16, 2019), cid.manufactureDate().year);
}
