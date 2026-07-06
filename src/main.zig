//! rfr26-tempSensor — ported from STM32 HAL/C to Zig via MicroZig
//! Original author: Jeevan Shah (main.c)
//! Port: draft, unverified against live MicroZig API — see caveats above.

const std = @import("std");
const microzig = @import("microzig");
const hal = microzig.hal;
const chip = microzig.chip;
const peripherals = chip.peripherals;

pub const microzig_options: microzig.Options = .{
    .interrupts = .{},
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Biquad IIR filter — 2nd-order Butterworth lowpass @ 5kHz, fs = 250kHz
// scipy.signal.butter(2, 5000, fs=250000, output='sos')
// ---------------------------------------------------------------------------
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

var biquad_state: [NUM_CHANNELS_TOTAL]BiquadState = [_]BiquadState{.{}} ** NUM_CHANNELS_TOTAL;

fn biquadReset(mux: u8, ch: u8) void {
    const idx: usize = @as(usize, mux) * MUX_CHANNELS_PER_CHIP + ch;
    biquad_state[idx] = .{};
}

fn biquadProcess(mux: u8, ch: u8, x: f64) f64 {
    const idx: usize = @as(usize, mux) * MUX_CHANNELS_PER_CHIP + ch;
    const s = &biquad_state[idx];
    const w0 = x - BIQUAD_A1 * s.w1 - BIQUAD_A2 * s.w2;
    const y = BIQUAD_B0 * w0 + BIQUAD_B1 * s.w1 + BIQUAD_B2 * s.w2;
    s.w2 = s.w1;
    s.w1 = w0;
    return y;
}

// ---------------------------------------------------------------------------
// Pins — SHAKY: verify against actual microzig.hal pin API
// ---------------------------------------------------------------------------
const pins = struct {
    const mux_sync = [NUM_MUXES]hal.gpio.Pin{
        hal.gpio.Pin.init(.B, 0),
        hal.gpio.Pin.init(.B, 1),
        hal.gpio.Pin.init(.B, 2),
    };
    const spi_sck = hal.gpio.Pin.init(.A, 5);
    const spi_mosi = hal.gpio.Pin.init(.A, 7);
    const adc_in = hal.gpio.Pin.init(.A, 2);
    const uart_tx = hal.gpio.Pin.init(.A, 9);
    const uart_rx = hal.gpio.Pin.init(.A, 10);
    const can_rx = hal.gpio.Pin.init(.A, 11);
    const can_tx = hal.gpio.Pin.init(.A, 12);
};

var spi_dev: hal.spi.SPI = undefined;
var adc_dev: hal.adc.ADC = undefined;
var uart_dev: hal.uart.UART = undefined;

// ---------------------------------------------------------------------------
// DWT-based delay_us — direct register access, stable across all Cortex-M3 parts
// ---------------------------------------------------------------------------
fn delayUs(us: u32) void {
    const dwt = peripherals.DWT;
    const start = dwt.CYCCNT.raw;
    const ticks = us * (hal.clocks.system_clock_hz() / 1_000_000);
    while ((dwt.CYCCNT.raw -% start) < ticks) {}
}

fn enableDwtCycleCounter() void {
    peripherals.CoreDebug.DEMCR.modify(.{ .TRCENA = 1 });
    peripherals.DWT.CYCCNT.raw = 0;
    peripherals.DWT.CTRL.modify(.{ .CYCCNTENA = 1 });
}

// ---------------------------------------------------------------------------
// Mux control (bit-banged SYNC + SPI 1-line write, same as original)
// ---------------------------------------------------------------------------
fn muxWriteRaw(mux: MuxId, cmd: u8) !void {
    for (pins.mux_sync) |p| p.set(); // deassert all (open-drain, high = released)
    const idx = @intFromEnum(mux);
    pins.mux_sync[idx].clear(); // assert SYNC for target mux

    try spi_dev.writeBlocking(&[_]u8{cmd}, .{});

    pins.mux_sync[idx].set();
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

// ---------------------------------------------------------------------------
// ADC — single conversion, blocking poll
// ---------------------------------------------------------------------------
fn adcReadRaw() u16 {
    adc_dev.startConversion();
    adc_dev.waitForConversion();
    return adc_dev.readResult();
}

fn adcReadRawSettled() u16 {
    return adcReadRaw();
}

// ---------------------------------------------------------------------------
// Voltage -> temp, linear interpolation over datasheet table
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// CAN — raw bxCAN register access (RM0008 ch. 24). Field names below assume
// regz produced CMSIS-matching names; adjust casing if your generated SVD
// module differs.
// ---------------------------------------------------------------------------
var can_tx_data: [8]u8 = undefined;

fn canInitFilter() void {
    const can = peripherals.CAN1;

    can.FMR.modify(.{ .FINIT = 1 });
    can.FM1R.modify(.{ .FBM0 = 0 }); // mask mode, bank 0
    can.FS1R.modify(.{ .FSC0 = 1 }); // 32-bit scale, bank 0
    can.FFA1R.modify(.{ .FFA0 = 0 }); // -> FIFO0
    can.sFilterRegister[0].FR1.raw = 0x0000_0000;
    can.sFilterRegister[0].FR2.raw = 0x0000_0000; // mask = 0 -> accept all
    can.FA1R.modify(.{ .FACT0 = 1 }); // activate bank 0
    can.FMR.modify(.{ .FINIT = 0 });
}

fn canStart() void {
    const can = peripherals.CAN1;
    can.MCR.modify(.{ .SLEEP = 0, .INRQ = 0 });
    while (can.MSR.read().INAK == 1) {}
}

/// checksum + no-ACK/mailbox-full visibility (fixes the silent-drop bug from
/// the C version — returns error instead of swallowing HAL_BUSY)
fn canFindFreeMailbox() !u2 {
    const tsr = peripherals.CAN1.TSR.read();
    if (tsr.TME0 == 1) return 0;
    if (tsr.TME1 == 1) return 1;
    if (tsr.TME2 == 1) return 2;
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
    var data = [_]u8{0} ** 8;
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
    const mb = try canFindFreeMailbox();
    const box = &peripherals.CAN1.sTxMailBox[mb];

    box.TIR.raw = (ext_id << 3) | (1 << 2); // EXID field, IDE=1 (extended)
    box.TDTR.modify(.{ .DLC = @as(u4, @intCast(data.len)) });
    box.TDLR.raw = std.mem.readInt(u32, data[0..4], .little);
    box.TDHR.raw = std.mem.readInt(u32, data[4..8], .little);
    box.TIR.modify(.{ .TXRQ = 1 }); // request transmission
}

// ---------------------------------------------------------------------------
// Main scan loop
// ---------------------------------------------------------------------------
fn scanAllMuxChannels(stats: *TempStatistics, report: bool) void {
    var temps: [90]u8 = [_]u8{0} ** 90;
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
            hal.time.sleep_ms(5);
            biquadReset(mux, ch);

            if (mux == 2 and ch == 4) {
                var daq_samples = [_]u8{0} ** 7;
                var daq_idx: u8 = 0;
                var daq_packet_seq: u32 = 0;

                var i: u16 = 0;
                while (i < 500) : (i += 1) {
                    const raw = adcReadRawSettled();
                    const filtered = biquadProcess(mux, ch, @floatFromInt(raw));
                    const raw_f: u16 = @intFromFloat(filtered + 0.5);

                    daq_samples[daq_idx] = @intFromFloat(100.0 * (3.0 * @as(f32, @floatFromInt(raw_f)) / 4095.0));
                    daq_idx += 1;

                    if (daq_idx == 7) {
                        const daq_id = DAQ_BASE_ID + daq_packet_seq;
                        canSendChannelTemp(daq_id, daq_samples) catch {};
                        daq_packet_seq += 1;
                        daq_idx = 0;
                        daq_samples = [_]u8{0} ** 7;
                    }

                    if (raw_f <= 1911 or raw_f >= 2962) {
                        faults += 1;
                        continue;
                    }
                    sum += raw_f;
                    count += 1;
                }
            } else {
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
            }

            var temp_c: f32 = undefined;

            if (faults >= 425) {
                temp_c = 80;
                high_temps += 1;
            } else {
                sum /= count;
                const voltage = 3.0 * @as(f32, @floatFromInt(sum)) / 4095.0;
                temp_c = sensorVoltageToTempC(voltage);

                if (temp_c > 0 and temp_c < 85) {
                    const t_i16: i16 = @intFromFloat(temp_c);
                    if (t_i16 < stats.min_temp) stats.min_temp = t_i16;
                    if (t_i16 > stats.max_temp) stats.max_temp = t_i16;
                }
            }

            const idx: usize = @as(usize, mux) * MUX_CHANNELS_PER_CHIP + ch;
            temps[idx] = @intFromFloat(temp_c);

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

// ---------------------------------------------------------------------------
// Clock config — HSE 8MHz -> PLLx9 = 72MHz, ADC /6.
// NOTE: AHB divider is /2 here, matching the original (looks like an inherited
// bug — halves HCLK to 36MHz instead of running at full 72MHz — not fixed,
// ported as-is per your original comment structure).
// ---------------------------------------------------------------------------
fn systemClockConfig() void {
    hal.rcc.configure(.{
        .hse_hz = 8_000_000,
        .pll = .{ .source = .hse, .mul = 9 },
        .ahb_div = 2,
        .apb1_div = 2,
        .apb2_div = 1,
        .adc_div = 6,
    });
    hal.rcc.enableCss();
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
pub fn main() !void {
    systemClockConfig();

    for (pins.mux_sync) |p| p.setMode(.output_open_drain);
    pins.spi_sck.setMode(.alt_push_pull);
    pins.spi_mosi.setMode(.alt_push_pull);
    pins.adc_in.setMode(.analog);
    pins.uart_tx.setMode(.alt_push_pull);
    pins.uart_rx.setMode(.input_floating);
    pins.can_rx.setMode(.input_floating);
    pins.can_tx.setMode(.alt_push_pull);

    for (pins.mux_sync) |p| p.set(); // released state

    spi_dev = try hal.spi.SPI.init(.SPI1, .{
        .mode = .master,
        .direction = .one_line,
        .data_size = .bits_8,
        .cpol = .low,
        .cpha = .first_edge,
        .baud_div = 4,
        .first_bit = .msb,
    });

    adc_dev = try hal.adc.ADC.init(.ADC1, .{
        .channel = 2,
        .sample_time = .cycles_239_5,
    });

    uart_dev = try hal.uart.UART.init(.USART1, .{ .baud_rate = 115_200 });

    enableDwtCycleCounter();

    canInitFilter();
    canStart();

    adc_dev.calibrate();

    var stats = TempStatistics{};
    const initial = TempStatistics{ .min_temp = 20, .max_temp = 20 };

    muxDisableAll();
    hal.time.sleep_ms(10);

    canSendTemperatureStatistics(&initial) catch {};

    while (true) {
        scanAllMuxChannels(&stats, true);
        stats.min_temp = 300;
        stats.max_temp = -300;
    }
}
