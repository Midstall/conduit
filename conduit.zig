//! conduit: a hardware abstraction layer for Zig.
//!
//! Shared by Weir (RISC-V firmware) and Ferrite (microkernel). Two decoupled
//! halves bridged by `Mmio` + `Backend` + `Resource`:
//!
//!   * Discovery: a pluggable `Backend` (device tree, ACPI, or a host's own)
//!     feeding a match-rule `Registry` (runtime) or `Builder` (comptime) that
//!     iterates every device of a class.
//!   * Driving: device-class contracts (`device.*`) implemented by shipped
//!     drivers (`driver.*`), each binding over an `Mmio`.
//!
//! See docs/superpowers/specs/2026-06-09-conduit-hal-design.md for the design.

pub const config = @import("conduit/config.zig");

pub const dtree = if (config.have_dtree) @import("dtree") else struct {};
pub const almanac = if (config.have_almanac) @import("almanac") else struct {};

pub const Mmio = @import("conduit/mmio.zig");

pub const resource = @import("conduit/resource.zig");
pub const Resource = resource.Resource;
pub const ResourceList = resource.List;

pub const match = @import("conduit/match.zig");
pub const Class = match.Class;
pub const Matcher = match.Matcher;
pub const IdList = match.IdList;

const builtin = @import("builtin");

const backend_iface = @import("conduit/backend.zig");
pub const Backend = backend_iface.Backend;

/// The PCI backend builds only for the two OS targets it has an impl for (Linux
/// /sys and UEFI EFI_PCI_IO). On any other target (e.g. the freestanding
/// `conduit_bare` module) it stays an empty struct so the host-OS code never
/// compiles, even with `-Dpci` on.
const pci_supported = builtin.os.tag == .linux or builtin.os.tag == .uefi;

pub const backend = struct {
    pub const Backend = backend_iface.Backend;
    pub const Node = backend_iface.Node;
    pub const PciInfo = backend_iface.PciInfo;
    pub const Error = backend_iface.Error;
    pub const fromImpl = backend_iface.fromImpl;
    pub const dtree = if (config.have_dtree) @import("conduit/backend/dtree.zig") else struct {};
    pub const almanac = if (config.have_almanac) @import("conduit/backend/almanac.zig") else struct {};
    pub const pci = if (config.have_pci and pci_supported) @import("conduit/backend/pci.zig") else struct {};
};

pub const discover = @import("conduit/discover.zig");
pub const Registry = discover.Registry;
pub const Builder = discover.Builder;
pub const Match = discover.Match;
pub const ofClass = discover.ofClass;

/// Device-class contracts (vtables).
pub const device = struct {
    pub const Serial = @import("conduit/device/serial.zig");
    pub const Block = @import("conduit/device/block.zig");
    pub const Gpio = @import("conduit/device/gpio.zig");
    pub const I2c = @import("conduit/device/i2c.zig");
    pub const Spi = @import("conduit/device/spi.zig");
    pub const Rtc = @import("conduit/device/rtc.zig");
    pub const Intc = @import("conduit/device/intc.zig");
};

/// Shipped driver implementations. Each exports `bind(Mmio, ...)`, and the
/// DT/ACPI-discoverable ones also export a `matcher`. Statically-mapped devices
/// (bound directly over an `Mmio`, e.g. the Albion secure-element blocks) have
/// no matcher.
pub const driver = struct {
    pub const ns16550a = @import("conduit/driver/ns16550a.zig");
    pub const pl011 = @import("conduit/driver/pl011.zig");
    pub const harbor_gpio = @import("conduit/driver/harbor_gpio.zig");
    pub const harbor_i2c = @import("conduit/driver/harbor_i2c.zig");
    pub const harbor_spi = @import("conduit/driver/harbor_spi.zig");
    pub const plic = @import("conduit/driver/plic.zig");
    pub const clint = @import("conduit/driver/clint.zig");
    pub const gicv2 = @import("conduit/driver/gicv2.zig");
    pub const gicv3 = @import("conduit/driver/gicv3.zig");
    pub const sdhci = @import("conduit/driver/sdhci.zig");
    pub const virtio_blk = @import("conduit/driver/virtio_blk.zig");
    pub const virtio_gpu = @import("conduit/driver/virtio_gpu.zig");
    pub const goldfish_rtc = @import("conduit/driver/goldfish_rtc.zig");
    pub const pl031 = @import("conduit/driver/pl031.zig");
    pub const albion_ftpm = @import("conduit/driver/albion_ftpm.zig");
    pub const albion_pcr = @import("conduit/driver/albion_pcr.zig");
    pub const albion_ed25519 = @import("conduit/driver/albion_ed25519.zig");
    pub const albion_integrity = @import("conduit/driver/albion_integrity.zig");
    pub const albion_gpc = @import("conduit/driver/albion_gpc.zig");
    pub const albion_rmm = @import("conduit/driver/albion_rmm.zig");
    pub const albion_measure = @import("conduit/driver/albion_measure.zig");
    pub const albion_sealver = @import("conduit/driver/albion_sealver.zig");
    pub const albion_covi = @import("conduit/driver/albion_covi.zig");
    pub const albion_covi_inject = @import("conduit/driver/albion_covi_inject.zig");
    pub const albion_ed25519_sign = @import("conduit/driver/albion_ed25519_sign.zig");
    pub const albion_rotkdf = @import("conduit/driver/albion_rotkdf.zig");
    pub const harbor_trng = @import("conduit/driver/harbor_trng.zig");
};

/// A matcher table covering every shipped driver. A consumer can pass this
/// straight to a Registry/Builder, or assemble a narrower table by hand.
pub const all_matchers = [_]Matcher{
    driver.ns16550a.matcher,
    driver.pl011.matcher,
    driver.harbor_gpio.matcher,
    driver.harbor_i2c.matcher,
    driver.harbor_spi.matcher,
    driver.plic.matcher,
    driver.clint.matcher,
    driver.gicv2.matcher,
    driver.gicv3.matcher,
    driver.sdhci.matcher,
    driver.virtio_blk.matcher,
    driver.goldfish_rtc.matcher,
    driver.pl031.matcher,
};

// Unit tests: the inline `test` blocks live in the modules below (match, the
// device/driver register logic, the dtree decode/ranges helpers, etc).
// Integration tests live under test/ and run through the public module.
test {
    _ = match;
    _ = resource;
    _ = Mmio;
    _ = discover;
    inline for (.{ device.Serial, device.Block, device.Gpio, device.I2c, device.Spi, device.Rtc, device.Intc }) |d| _ = d;
    inline for (.{
        driver.ns16550a,           driver.pl011,               driver.harbor_gpio,      driver.harbor_i2c,
        driver.harbor_spi,         driver.plic,                driver.clint,            driver.gicv2,
        driver.gicv3,              driver.sdhci,               driver.virtio_blk,       driver.virtio_gpu,
        driver.goldfish_rtc,       driver.pl031,               driver.albion_ftpm,      driver.albion_pcr,
        driver.harbor_trng,        driver.albion_ed25519,      driver.albion_integrity, driver.albion_gpc,
        driver.albion_rmm,         driver.albion_measure,      driver.albion_sealver,   driver.albion_covi,
        driver.albion_covi_inject, driver.albion_ed25519_sign, driver.albion_rotkdf,
    }) |d| _ = d;
    if (config.have_dtree) _ = backend.dtree;
    if (config.have_almanac) _ = backend.almanac;
    if (config.have_pci and pci_supported) _ = backend.pci;
}

/// A matcher that claims every PCI device (any vendor/class), so the string
/// `Registry`/`Builder` path enumerates PCI nodes, and a consumer then refines on
/// `Match.pci` (the numeric vendor/device/class). Pair it with `Class.pci`.
pub const pci_matcher = Matcher{ .class = .pci, .dt_compatible = &.{"pci"} };
