//! rfr26-tempSensor — ported from STM32 HAL/C to Zig via MicroZig
//! Original author: Jeevan Shah (main.c)
//! Port: draft, unverified against live MicroZig API — see caveats above.

const std = @import("std");
const microzig = @import("microzig");
const stm32 = microzig.hal;
const chip = microzig.chip;
const peripherals = chip.peripherals;
const rcc = stm32.rcc;
const gpio = stm32.gpio;
const time = stm32.time;

const spi = stm32.spi.SPI.init(.SPI1);
const adc = stm32.adc.ADC.init(.ADC1);
const uart = stm32.uart.UART.init(.USART1);

pub const microzig_options: microzig.Options = .{
    .interrupts = .{},
};

pub const panic = microzig.panic;
pub const std_options = microzig.std_options(.{});

comptime {
    _ = microzig.export_startup();
}

const NUM_MUXES: u8 = 3;
const MUX_CHANNELS_PER_CHIP: u8 = 32;
const MUX_DISABLE_CMD: u8 = 0x80;
const DAQ_BASE_ID: u32 = 0x18FF5000;
const TEMP_STATS_CAN_ID: u32 = 0x1839F380;

const MuxId = enum(u2) { mux1 = 0, mux2 = 1, mux3 = 2 };

const TempStatistics = struct {
    min_temp: i16 = 300,
    max_temp: i16 = -300,
    avg_temp: i16 = 0,
    min_channel: u8 = 0,
    max_channel: u8 = 0,
    num_enabled: u8 = 0,
};

const BIQUAD_B0: f64 = 0.0036216815;
const BIQUAD_B1: f64 = 0.0072433630;
const BIQUAD_B2: f64 = 0.0036216815;
const BIQUAD_A1: f64 = -1.8226949252;
const BIQUAD_A2: f64 = 0.8371816513;

const NUM_CHANNELS_TOTAL: usize = @as(usize, NUM_MUXES) * MUX_CHANNELS_PER_CHIP;

const BiquadState = struct {
    w1: f64 = 0.0,
    w2: f64 = 0.0,
};

var biquad_state: [NUM_CHANNELS_TOTAL]BiquadState = @splat(.{});

fn biquadReset(mux: u8, ch: u8) void {
    const idx: usize = @as(usize, mux) * @as(usize, MUX_CHANNELS_PER_CHIP) + @as(usize, ch);
    biquad_state[idx] = .{};
}

fn biquadProcess(mux: u8, ch: u8, x: f64) f64 {
    const idx: usize = @as(usize, mux) * @as(usize, MUX_CHANNELS_PER_CHIP) + @as(usize, ch);
    const s = &biquad_state[idx];
    const w0 = x - BIQUAD_A1 * s.w1 - BIQUAD_A2 * s.w2;
    const y = BIQUAD_B0 * w0 + BIQUAD_B1 * s.w1 + BIQUAD_B2 * s.w2;
    s.w2 = s.w1;
    s.w1 = w0;
    return y;
}

const pins = struct {
    // PB0, PB1, PB2
    const mux_sync = [NUM_MUXES]gpio.Pin{
        gpio.Pin.from_port(.B, 0),
        gpio.Pin.from_port(.B, 1),
        gpio.Pin.from_port(.B, 2),
    };
    const spi_sck = gpio.Pin.from_port(.A, 5);
    const spi_mosi = gpio.Pin.from_port(.A, 7);
    const adc_in = gpio.Pin.from_port(.A, 2);
    const uart_tx = gpio.Pin.from_port(.A, 9);
    const uart_rx = gpio.Pin.from_port(.A, 10);
    const can_rx = gpio.Pin.from_port(.A, 11);
    const can_tx = gpio.Pin.from_port(.A, 12);
};

fn muxWriteRaw(mux: MuxId, cmd: u8) !void {
    for (pins.mux_sync) |p| p.put(1); // deassert all (open-drain, high = released)
    const idx = @intFromEnum(mux);
    pins.mux_sync[idx].put(0); // assert SYNC for target mux

    spi.write_blocking(&[_]u8{cmd});

    pins.mux_sync[idx].put(1);
}

fn muxDisable(mux: MuxId) !void {
    try muxWriteRaw(mux, MUX_DISABLE_CMD);
}

fn muxDisableAll() void {
    muxWriteRaw(.mux1, MUX_DISABLE_CMD) catch {};
    muxWriteRaw(.mux2, MUX_DISABLE_CMD) catch {};
    muxWriteRaw(.mux3, MUX_DISABLE_CMD) catch {};
}

fn muxSelectChannel(mux: MuxId, channel: u8) !void {
    if (channel >= MUX_CHANNELS_PER_CHIP) return error.InvalidChannel;

    inline for (.{ MuxId.mux1, MuxId.mux2, MuxId.mux3 }) |m| {
        if (m != mux) try muxDisable(m);
    }
    try muxWriteRaw(mux, channel & 0x1F);
}

fn adcReadRaw() u16 {
    return adc.read_single_channel(2) catch 0;
}

fn adcReadRawSettled() u16 {
    return adcReadRaw();
}

const temp_table = [_]f32{ -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120 };
const volt_table = [_]f32{ 2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30 };

fn sensorVoltageToTempC(voltage: f32) f32 {
    const n = temp_table.len;
    if (voltage >= volt_table[0]) return temp_table[0];
    if (voltage <= volt_table[n - 1]) return temp_table[n - 1];

    var i: usize = 0;
    while (i < n - 1) : (i += 1) {
        if (voltage <= volt_table[i] and voltage >= volt_table[i + 1]) {
            const v1 = volt_table[i];
            const v2 = volt_table[i + 1];
            const t1 = temp_table[i];
            const t2 = temp_table[i + 1];
            return t1 + (voltage - v1) * (t2 - t1) / (v2 - v1);
        }
    }
    return -999.0;
}

var can_tx_data: [8]u8 = undefined;

fn canInitFilter() void {
    const can = peripherals.CAN;

    can.FMR.modify(.{ .FINIT = 1 });
    can.FM1R.raw &= ~@as(u32, 1); // mask mode, bank 0
    can.FS1R.raw |= 1; // 32-bit scale, bank 0
    can.FFA1R.raw &= ~@as(u32, 1); // -> FIFO0
    can.FB[0].FR1.raw = 0x0000_0000;
    can.FB[0].FR2.raw = 0x0000_0000; // mask = 0 -> accept all
    can.FA1R.raw |= 1; // activate bank 0
    can.FMR.modify(.{ .FINIT = 0 });
}

fn canStart() void {
    const can = peripherals.CAN;
    can.MCR.modify(.{ .SLEEP = 0, .INRQ = 0 });
    while (can.MSR.read().INAK == 1) {}
}

fn canFindFreeMailbox() !u2 {
    const tsr = peripherals.CAN.TSR.read();
    if (tsr.@"TME[0]" == 1) return 0;
    if (tsr.@"TME[1]" == 1) return 1;
    if (tsr.@"TME[2]" == 1) return 2;
    return error.CanMailboxesFull; // all 3 full -> bus fault, not backpressure
}

fn canSendChannelTemp(id: u32, temp: [7]u8) !void {
    var checksum: u8 = 0;
    for (temp) |t| checksum +%= t;

    can_tx_data[0..7].* = temp;
    can_tx_data[7] = checksum +% 65;

    try canTransmit(id, &can_tx_data);
}

fn canSendTemperatureStatistics(stats: *const TempStatistics) !void {
    var data: [8]u8 = @splat(0);
    data[1] = @bitCast(@as(i8, @truncate(stats.min_temp)));
    data[2] = @bitCast(@as(i8, @truncate(stats.max_temp)));
    data[3] = stats.max_channel;
    data[4] = 1;
    data[5] = 1;
    data[6] = 0;

    var checksum: u8 = 0;
    for (data[0..7]) |b| checksum +%= b;
    data[7] = checksum +% 65;

    try canTransmit(TEMP_STATS_CAN_ID, &data);
}

fn canTransmit(ext_id: u32, data: []const u8) !void {
    if (data.len != 8) return error.InvalidCanPayloadLength;

    const mb = try canFindFreeMailbox();
    const mailbox = &peripherals.CAN.TX[mb];

    mailbox.TIR.raw = (ext_id << 3) | (1 << 2); // EXID field, IDE=1 (extended)
    mailbox.TDTR.modify(.{ .DLC = @as(u4, @intCast(data.len)) });
    mailbox.TDLR.raw = std.mem.readInt(u32, data[0..4], .little);
    mailbox.TDHR.raw = std.mem.readInt(u32, data[4..8], .little);
    mailbox.TIR.modify(.{ .TXRQ = 1 }); // request transmission
}

fn scanAllMuxChannels(stats: *TempStatistics, report: bool) void {
    // var temps: [90]u8 = [_]u8{0} ** 90;
    var high_temps: i8 = 0;

    var mux: u8 = 0;
    while (mux < NUM_MUXES) : (mux += 1) {
        var ch: u8 = 0;
        while (ch < MUX_CHANNELS_PER_CHIP) : (ch += 1) {
            if (mux == 2 and ch > 25) continue;

            var faults: i32 = 0;
            var sum: u32 = 0;
            var count: u16 = 0;

            muxSelectChannel(@enumFromInt(mux), ch) catch continue;
            time.sleep_ms(5);
            biquadReset(mux, ch);

            var i: u16 = 0;
            while (i < 500) : (i += 1) {
                const raw = adcReadRawSettled();
                // NOTE: biquad intentionally unused here, matching original
                // (only the mux2/ch4 DAQ path filters — inherited quirk).
                if (raw <= 1911 or raw >= 2962) {
                    faults += 1;
                    continue;
                }
                sum += raw;
                count += 1;
            }

            var temp_c: f32 = undefined;

            if (faults >= 425) {
                temp_c = 80;
                high_temps += 1;
            } else {
                if (count == 0) continue;
                sum /= count;
                const voltage = 3.0 * @as(f32, @floatFromInt(sum)) / 4095.0;
                temp_c = sensorVoltageToTempC(voltage);

                if (temp_c > 0 and temp_c < 85) {
                    const t_i16: i16 = @intFromFloat(temp_c);
                    if (t_i16 < stats.min_temp) stats.min_temp = t_i16;
                    if (t_i16 > stats.max_temp) stats.max_temp = t_i16;
                }
            }

            // const idx: usize = @as(usize, mux) * MUX_CHANNELS_PER_CHIP + ch;
            // temps[idx] = @intFromFloat(temp_c);

            if (report) {
                if (stats.min_temp == 300 or stats.max_temp == -300) continue;
                stats.max_channel = @bitCast(high_temps);
                canSendTemperatureStatistics(stats) catch {};
            } else {
                stats.max_temp = 20;
                stats.min_temp = 20;
                canSendTemperatureStatistics(stats) catch {};
                stats.max_temp = -300;
                stats.min_temp = 300;
            }
        }
    }

    if (high_temps >= 76) {
        stats.max_temp = 80;
        stats.min_temp = 80;
        canSendTemperatureStatistics(stats) catch {};
        stats.max_temp = -300;
        stats.min_temp = 300;
    }

    muxDisableAll();
}

fn systemClockConfig() !void {
    _ = try rcc.apply(.{
        .SYSCLKSource = .PLL1_P,
        .PLLSourceVirtual = .HSE_Div_PREDIV,
        .PLLMUL = .Mul9,
        .AHBCLKDivider = .Div1,
        .APB1CLKDivider = .Div2,
        .ADCPresc = .Div6,
        .flags = .{
            .HSEOscillator = true,
            .USE_ADC1 = true,
        },
    });
}

pub fn main() !void {
    try systemClockConfig();

    rcc.enable_clock(.GPIOA);
    rcc.enable_clock(.GPIOB);
    rcc.enable_clock(.AFIO);
    rcc.enable_clock(.SPI1);
    rcc.enable_clock(.ADC1);
    rcc.enable_clock(.USART1);
    rcc.enable_clock(.CAN);
    rcc.enable_clock(.TIM2);

    time.init_timer(.TIM2);

    for (pins.mux_sync) |p| p.set_output_mode(.general_purpose_open_drain, .max_2MHz);
    pins.spi_sck.set_output_mode(.alternate_function_push_pull, .max_50MHz);
    pins.spi_mosi.set_output_mode(.alternate_function_push_pull, .max_50MHz);
    pins.adc_in.set_input_mode(.analog);
    pins.uart_tx.set_output_mode(.alternate_function_push_pull, .max_50MHz);
    pins.uart_rx.set_input_mode(.floating);
    pins.can_rx.set_input_mode(.floating);
    pins.can_tx.set_output_mode(.alternate_function_push_pull, .max_50MHz);

    for (pins.mux_sync) |p| p.put(1); // released state

    spi.apply(.{
        .chip_select = .GPIO,
        .prescaler = .Div4,
    });

    try uart.apply_runtime(.{
        .clock_speed = rcc.get_clock(.USART1),
    });

    adc.enable();
    adc.set_channel_sample_rate(2, .@"239.5");

    canInitFilter();
    canStart();

    var stats = TempStatistics{};
    const initial = TempStatistics{ .min_temp = 20, .max_temp = 20 };

    muxDisableAll();
    time.sleep_ms(10);

    canSendTemperatureStatistics(&initial) catch {};

    while (true) {
        scanAllMuxChannels(&stats, true);
        stats.min_temp = 300;
        stats.max_temp = -300;
    }
}
