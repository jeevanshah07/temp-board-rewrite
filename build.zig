const std = @import("std");
const microzig = @import("microzig");

const MicroBuild = microzig.MicroBuild(.{
    .stm32 = true,
});

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});

    const mz_dep = b.dependency("microzig", .{});
    const mb = MicroBuild.init(b, mz_dep) orelse return;
    const stm32 = mb.ports.stm32;

    const fw = mb.add_firmware(.{
        .name = "rfr-27-temp-board",
        .target = stm32.chips.STM32F103T8,
        .optimize = optimize,
        .root_source_file = b.path("src/main.zig"),
    });

    mb.install_firmware(fw, .{});

    // For debugging, we also always install the firmware as an ELF file
    mb.install_firmware(fw, .{ .format = .elf });
    mb.install_firmware(fw, .{ .format = .bin });
}
