//! ARM PL011 UART driver, over an `Mmio`. The serial console on QEMU's
//! aarch64 `virt` machine and many ARM SoCs Ferrite targets.

const Mmio = @import("../mmio.zig");
const Serial = @import("../device/serial.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .uart,
    .dt_compatible = &.{ "arm,pl011", "arm,primecell" },
    .acpi_hid = &.{ "ARMH0011", "arm,sbsa-uart" },
    .driver = "pl011",
};

pub const Pl011 = struct {
    mmio: Mmio,

    const DR = 0x00; // data register
    const FR = 0x18; // flag register
    const LCRH = 0x2c; // line control
    const CR = 0x30; // control register

    const FR_RXFE = 0x10; // receive FIFO empty
    const FR_TXFF = 0x20; // transmit FIFO full

    pub fn init(self: Pl011) void {
        self.mmio.write(u32, CR, 0); // disable while configuring
        self.mmio.write(u32, LCRH, 0x70); // 8N1, FIFOs enabled
        self.mmio.write(u32, CR, 0x301); // UARTEN | TXE | RXE
    }

    pub fn txReady(self: Pl011) bool {
        return self.mmio.read(u32, FR) & FR_TXFF == 0;
    }

    pub fn rxReady(self: Pl011) bool {
        return self.mmio.read(u32, FR) & FR_RXFE == 0;
    }

    pub fn putc(self: Pl011, c: u8) void {
        while (!self.txReady()) {}
        self.mmio.write(u32, DR, c);
    }

    pub fn write(self: Pl011, s: []const u8) void {
        for (s) |c| {
            if (c == '\n') self.putc('\r');
            self.putc(c);
        }
    }

    pub fn read(self: Pl011, buf: []u8) usize {
        var n: usize = 0;
        while (n < buf.len and self.rxReady()) : (n += 1) {
            buf[n] = @truncate(self.mmio.read(u32, DR));
        }
        return n;
    }

    pub fn serial(self: *Pl011) Serial {
        return Serial.from(self);
    }
};

pub fn bind(mmio: Mmio) Pl011 {
    const u = Pl011{ .mmio = mmio };
    u.init();
    return u;
}
