//! NS16550A (8250-family) UART driver, over an `Mmio`. Ported from Weir's
//! console/uart.zig. It replaces the raw `@ptrFromInt` pokes with `mmio`, so the
//! same source runs in Weir (M-mode pointer) and Ferrite (mapped region).
//!
//! Byte-wide register stride (reg-shift 0), as on QEMU virt and River.

const Mmio = @import("../mmio.zig");
const Serial = @import("../device/serial.zig");
const match = @import("../match.zig");

pub const matcher = match.Matcher{
    .class = .uart,
    .dt_compatible = &.{ "ns16550a", "ns16550", "snps,dw-apb-uart" },
    .acpi_hid = &.{ "PNP0501", "PNP0500" },
    .driver = "ns16550a",
};

pub const Ns16550a = struct {
    mmio: Mmio,
    divisor: u16 = 0,
    /// Program only LCR and the DLL/DLM divisor. Skip the IER, FCR, and MCR
    /// writes. See `init` for why River needs this.
    minimal_init: bool = false,

    const THR = 0; // transmit holding (write) / receive buffer (read)
    const IER = 1; // interrupt enable
    const FCR = 2; // FIFO control (write)
    const LCR = 3; // line control
    const MCR = 4; // modem control
    const LSR = 5; // line status

    const LSR_DR = 0x01; // data ready
    const LSR_THRE = 0x20; // transmit holding register empty

    pub fn init(self: Ns16550a) void {
        // River's minimal UART does not ack writes to FCR (offset 2) or MCR
        // (offset 4). A CPU store to an unacked register stalls forever. So
        // `minimal_init` programs only LCR and the divisor. HW-localised
        // 2026-07-02.
        if (!self.minimal_init) self.mmio.write(u8, IER, 0x00); // no interrupts, we poll

        if (self.divisor != 0) {
            // Program the baud. DLAB=1 exposes DLL and DLM at offsets 0 and 1.
            self.mmio.write(u8, LCR, 0x83); // DLAB=1, 8N1
            self.mmio.write(u8, THR, @truncate(self.divisor & 0xff)); // DLL
            self.mmio.write(u8, IER, @truncate((self.divisor >> 8) & 0xff)); // DLM
        }

        self.mmio.write(u8, LCR, 0x03); // DLAB=0, 8N1 (latches the divisor)
        if (!self.minimal_init) {
            self.mmio.write(u8, FCR, 0x07); // enable + clear RX/TX FIFOs
            self.mmio.write(u8, MCR, 0x03); // DTR + RTS
        }
    }

    pub fn txReady(self: Ns16550a) bool {
        return self.mmio.read(u8, LSR) & LSR_THRE != 0;
    }

    pub fn rxReady(self: Ns16550a) bool {
        return self.mmio.read(u8, LSR) & LSR_DR != 0;
    }

    pub fn putc(self: Ns16550a, c: u8) void {
        while (!self.txReady()) {}
        self.mmio.write(u8, THR, c);
    }

    /// Write bytes, translating LF to CRLF (firmware-console convention).
    pub fn write(self: Ns16550a, s: []const u8) void {
        for (s) |c| {
            if (c == '\n') self.putc('\r');
            self.putc(c);
        }
    }

    pub fn read(self: Ns16550a, buf: []u8) usize {
        var n: usize = 0;
        while (n < buf.len and self.rxReady()) : (n += 1) {
            buf[n] = self.mmio.read(u8, THR);
        }
        return n;
    }

    pub fn serial(self: *Ns16550a) Serial {
        return Serial.from(self);
    }
};

pub const Options = struct {
    divisor: u16 = 0,
    /// Skip the IER, FCR, and MCR writes (see `Ns16550a.init`). Set it for
    /// River's minimal UART.
    minimal_init: bool = false,
};

/// Bind a 16550 over `mmio` and initialize it.
pub fn bind(mmio: Mmio, opts: Options) Ns16550a {
    const u = Ns16550a{ .mmio = mmio, .divisor = opts.divisor, .minimal_init = opts.minimal_init };
    u.init();
    return u;
}
