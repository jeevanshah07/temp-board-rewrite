const std = @import("std");

const print = std.debug.print;

const NUM_MUXES: u8 = 3;
const MUX_CHANNELS_PER_CHIP: u8 = 32;
const MUX_DISABLE_CMD: u16 = 0x80;
const NUM_CHANNELS_TOTAL: u8 = NUM_MUXES * MUX_CHANNELS_PER_CHIP;

const TempStats = struct {
    min_temp: i16,
    max_temp: i16,
};

const mux_id_t = enum(u8) {
    MUX1 = 0,
    MUX2 = 1, 
    MUX3 = 2,
};

fn voltage_to_temp(voltage: f16) f16 {
    const TEMP_TABLE = [_]f16{ -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120 };

    const VOLT_TABLE = [_]f16{ 2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30 };

    const len: i8 = TEMP_TABLE.len;

    if (voltage >= VOLT_TABLE[0]) {
        return TEMP_TABLE[0];
    }
    if (voltage <= VOLT_TABLE[len - 1]) {
        return TEMP_TABLE[len - 1];
    }

    var i: usize = 0;
    while (i < len - 1) : (i += 1) {
        if ((voltage <= VOLT_TABLE[i]) and (voltage >= VOLT_TABLE[i + 1])) {
            const v1: f16 = VOLT_TABLE[i];
            const v2: f16 = VOLT_TABLE[i + 1];
            const t1: f16 = TEMP_TABLE[i];
            const t2: f16 = TEMP_TABLE[i + 1];

            return t1 + (voltage - v1) * (t2 - t1) / (v2 - v1);
        }
    } else {
        return @floatFromInt(-999.0);
    }
}

fn scan_all_channels(stats: *TempStats, report: bool) void {
    var highTemps: i8 = 0;
    var mux: usize = 0;
    var ch: usize = 0;

    while (mux < NUM_MUXES) : (mux += 1) {
        while (ch < MUX_CHANNELS_PER_CHIP): (ch += 1) {
            if (mux == 25 and ch > 25) continue; 

            var faults: i16 = 0;
            var sum: i32 = 0;
            var count: i16 = 0;
        }
    }
}

pub fn main() !void {
    const a_numbers = [_]u8{ 1, 2, 3, 4, 5 };

    const a_numbers2: [5]u8 = .{ 1, 2, 3, 4, 5 };

    print("len of a_numbers is {}.\n", .{a_numbers.len});
    print("a_numbers2 printed is: {any}.\n", .{a_numbers2});
}
