//! Build-time configuration, resolved from conduit's build options.
//!
//! `resource_cap` is the number of resources each discovered device node holds
//! inline (no allocator: a fixed buffer). Set it with `-Dresource-cap`, default
//! 8. `have_dtree`/`have_almanac`/`have_pci` gate the bundled backends.

const options = @import("build_options");

pub const resource_cap: usize = options.resource_cap;
pub const have_dtree: bool = options.have_dtree;
pub const have_almanac: bool = options.have_almanac;
pub const have_pci: bool = options.have_pci;
