const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const no_tests = b.option(bool, "no-tests", "skip building tests") orelse false;
    const no_docs = b.option(bool, "no-docs", "skip installing documentation") orelse false;
    const resource_cap = b.option(usize, "resource-cap", "max resources held inline per device node") orelse 8;
    const want_dtree = b.option(bool, "dtree", "build the device-tree backend") orelse true;
    const want_almanac = b.option(bool, "almanac", "build the ACPI (almanac) backend") orelse true;

    const options = b.addOptions();
    options.addOption(usize, "resource_cap", resource_cap);
    options.addOption(bool, "have_dtree", want_dtree);
    options.addOption(bool, "have_almanac", want_almanac);

    const dtree = b.dependency("dtree", .{ .target = target, .optimize = optimize });
    const almanac = b.dependency("almanac", .{ .target = target, .optimize = optimize });

    const conduit = b.addModule("conduit", .{
        .root_source_file = b.path("conduit.zig"),
        .target = target,
        .optimize = optimize,
    });
    conduit.addOptions("build_options", options);
    if (want_dtree) conduit.addImport("dtree", dtree.module("dtree"));
    if (want_almanac) conduit.addImport("almanac", almanac.module("almanac"));

    if (!no_tests) {
        const step_test = b.step("test", "Run all unit tests");

        const unit_tests = b.addTest(.{
            .root_module = b.createModule(.{
                .root_source_file = b.path("conduit.zig"),
                .target = target,
                .optimize = optimize,
            }),
        });
        unit_tests.root_module.addOptions("build_options", options);
        if (want_dtree) unit_tests.root_module.addImport("dtree", dtree.module("dtree"));
        if (want_almanac) unit_tests.root_module.addImport("almanac", almanac.module("almanac"));

        const run_unit_tests = b.addRunArtifact(unit_tests);
        step_test.dependOn(&run_unit_tests.step);

        // Integration tests drive conduit through its public module.
        const integration_tests = b.addTest(.{
            .name = "integration-test",
            .root_module = b.createModule(.{
                .root_source_file = b.path("test/root.zig"),
                .target = target,
                .optimize = optimize,
            }),
        });
        integration_tests.root_module.addImport("conduit", conduit);
        if (want_dtree) integration_tests.root_module.addImport("dtree", dtree.module("dtree"));
        if (want_almanac) integration_tests.root_module.addImport("almanac", almanac.module("almanac"));

        const run_integration_tests = b.addRunArtifact(integration_tests);
        step_test.dependOn(&run_integration_tests.step);

        if (!no_docs) {
            const docs = b.addInstallDirectory(.{
                .source_dir = unit_tests.getEmittedDocs(),
                .install_dir = .prefix,
                .install_subdir = "docs",
            });
            b.getInstallStep().dependOn(&docs.step);
        }
    }

    // The discovery example needs the device-tree backend.
    if (want_dtree) {
        const exe_example = b.addExecutable(.{
            .name = "example",
            .root_module = b.createModule(.{
                .root_source_file = b.path("example.zig"),
                .target = target,
                .optimize = optimize,
                .imports = &.{
                    .{ .name = "conduit", .module = conduit },
                    .{ .name = "dtree", .module = dtree.module("dtree") },
                },
            }),
        });
        b.installArtifact(exe_example);

        const run_example = b.addRunArtifact(exe_example);
        const run_step = b.step("run", "Run the discovery example");
        run_step.dependOn(&run_example.step);
    }
}
