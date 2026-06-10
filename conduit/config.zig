//! Build-time configuration, resolved from conduit's build options.
//!
//! `resource_cap` is the number of resources held inline per discovered device
//! node (no allocator: a fixed buffer). It is set with `-Dresource-cap`,
//! default 8. `have_dtree`/`have_almanac` gate the two bundled backends.

const options = @import("build_options");

pub const resource_cap: usize = options.resource_cap;
pub const have_dtree: bool = options.have_dtree;
pub const have_almanac: bool = options.have_almanac;
