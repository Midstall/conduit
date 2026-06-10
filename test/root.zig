//! Integration test aggregator. These tests drive conduit through its public
//! module (`@import("conduit")`), the same way a consumer would. Backend-
//! specific suites are gated on the build options so the matrix
//! (-Ddtree/-Dalmanac) stays green.

const conduit = @import("conduit");

test {
    _ = @import("driver.zig");
    _ = @import("drivers.zig");
    _ = @import("custom_backend.zig");
    if (conduit.config.have_dtree) {
        _ = @import("discovery.zig");
    }
    if (conduit.config.have_almanac) {
        _ = @import("acpi.zig");
        _ = @import("acpi_full.zig");
    }
}
