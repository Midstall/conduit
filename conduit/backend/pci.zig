//! PCI bus-probing backend. Enumerates PCI devices as conduit `Node`s carrying
//! their numeric identity (segment/bus/device/function + vendor/device/class)
//! and BAR resources, so the ordinary `Registry`/`Builder` discovery path works
//! over PCI exactly as it does over device tree.
//!
//! The build target selects the implementation at comptime:
//!   * Linux  -> read-only `/sys/bus/pci/devices/*`, world-readable with no root.
//!     This yields enumeration and BAR addresses only. It does NOT touch BAR
//!     register contents (that needs root/mmap and is the consumer's bare-metal
//!     concern).
//!   * UEFI   -> EFI_PCI_IO_PROTOCOL via boot services. Reads config-space for
//!     vendor/device/class and BAR0 base from config offset 0x10 (the proven,
//!     descriptor-parse-immune approach).
//!
//! Each impl exposes the same `PciBackend` surface (`init`, `next`, `resources`,
//! `reset`, `any`) so it plugs straight into `Registry`/`Builder` via
//! `backend.fromImpl`. Both impls build for their own OS only. The comptime
//! selection gates the OS-specific `std.os.linux` / `std.os.uefi` code, so the
//! wrong-OS half never compiles.

const builtin = @import("builtin");

pub const PciBackend = switch (builtin.os.tag) {
    .linux => @import("pci/linux.zig").LinuxPciBackend,
    .uefi => @import("pci/uefi.zig").UefiPciBackend,
    else => @compileError("conduit: the PCI backend supports only linux (/sys) and uefi (EFI_PCI_IO) targets"),
};

/// The synthetic id every PCI node carries so the string `Matcher`/`Registry`
/// path can claim it. A matcher listing "pci" (see `conduit.pci_matcher`) claims
/// every PCI device, and the consumer then refines on `Node.pci` / `Match.pci`,
/// the NUMERIC vendor/device/class identity. conduit uses a single STABLE string
/// literal (not formatted vendor:device/class ids) because `Match` stores `ids`
/// as slices that must outlive iteration, and a per-node formatted buffer would
/// be reused and dangle. The numeric `pci` field is the durable, exact identity.
pub const id_any = "pci";

const backend = @import("../backend.zig");

/// Build a `Node` for a PCI device: its numeric identity on `.pci` plus the
/// stable synthetic `id_any` so string matchers claim it.
pub fn synthIds(info: backend.PciInfo) backend.Node {
    var node = backend.Node{ .pci = info };
    node.ids.append(id_any);
    return node;
}

test {
    // Pull in the OS-selected impl's tests.
    _ = PciBackend;
}
