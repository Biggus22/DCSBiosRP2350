#ifndef WEACT_RP2350B_CORE
#define WEACT_RP2350B_CORE
#endif
#include "pico/time.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/rs485.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

uart_inst_t *rs485_uart = uart0;

namespace {

constexpr uint MOTOR_IN1_PIN = 17;
constexpr uint MOTOR_IN2_PIN = 16;
constexpr uint MOTOR_ENCODER_A_PIN = 34;
constexpr uint MOTOR_ENCODER_B_PIN = 35;

constexpr uint16_t MOTOR_PWM_WRAP = 1000;
constexpr float MOTOR_PWM_CLKDIV = 125.0f;
constexpr uint16_t MOTOR_PWM_DRIVE_LEVEL = 1000;
constexpr uint16_t MOTOR_PWM_APPROACH_LEVEL = 550;

constexpr int32_t MOTOR_POSITION_TOLERANCE_COUNTS = 5;

constexpr uint LED_PIN_EXT = 15;

struct N20MotorState {
    bool ready = false;
    bool calibrated = false;
    bool encoder_inverted = false;
    uint pwm_slice = 0;
    uint8_t encoder_prev = 0;
    int32_t encoder_count = 0;
    int32_t target_count = 0;
    int32_t positive_stop_count = 0;
    int32_t negative_stop_count = 0;
    int32_t soft_pos_limit = 0;
    int32_t soft_neg_limit = 0;
    int32_t usable_half_range = 0;
};

N20MotorState g_motor = {};

constexpr int8_t QUADRATURE_DECODE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

// Reads the state from encoder pins A and B.
uint8_t readEncoderState() {
    return (uint8_t)((gpio_get(MOTOR_ENCODER_A_PIN) ? 0x2u : 0u) |
                     (gpio_get(MOTOR_ENCODER_B_PIN) ? 0x1u : 0u));
}

// Returns the absolute value of the input integer.
int32_t motorAbs(int32_t value) {
    return value >= 0 ? value : -value;
}

// Sets the motor control pins to zero level.
void stopMotor() {
    pwm_set_gpio_level(MOTOR_IN1_PIN, 0);
    pwm_set_gpio_level(MOTOR_IN2_PIN, 0);
}

// Sets the motor drive level and direction. If direction is positive, IN1 receives the drive level.
void setMotorDrive(int direction, uint16_t drive_level) {
    if (direction > 0) {
        pwm_set_gpio_level(MOTOR_IN1_PIN, drive_level);
        pwm_set_gpio_level(MOTOR_IN2_PIN, 0);
    } else if (direction < 0) {
        pwm_set_gpio_level(MOTOR_IN1_PIN, 0);
        pwm_set_gpio_level(MOTOR_IN2_PIN, drive_level);
    } else {
        stopMotor();
    }
}

uint16_t chooseDriveLevel(int32_t error) {
    int32_t window = g_motor.usable_half_range / 4;
    if (window < 5) window = 5;
    if (motorAbs(error) <= window) {
        return MOTOR_PWM_APPROACH_LEVEL;
    }
    return MOTOR_PWM_DRIVE_LEVEL;
}

bool initMotor() {
    gpio_set_function(MOTOR_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_IN2_PIN, GPIO_FUNC_PWM);

    const uint slice_in1 = pwm_gpio_to_slice_num(MOTOR_IN1_PIN);
    const uint slice_in2 = pwm_gpio_to_slice_num(MOTOR_IN2_PIN);
    if (slice_in1 != slice_in2) {
        printf("N20 PWM slice mismatch: GPIO%u -> %u, GPIO%u -> %u\n",
               MOTOR_IN1_PIN, slice_in1, MOTOR_IN2_PIN, slice_in2);
        return false;
    }

    g_motor.pwm_slice = slice_in1;
    pwm_set_wrap(g_motor.pwm_slice, MOTOR_PWM_WRAP);
    pwm_set_clkdiv(g_motor.pwm_slice, MOTOR_PWM_CLKDIV);
    stopMotor();
    pwm_set_enabled(g_motor.pwm_slice, true);

    gpio_init(MOTOR_ENCODER_A_PIN);
    gpio_set_dir(MOTOR_ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(MOTOR_ENCODER_A_PIN);

    gpio_init(MOTOR_ENCODER_B_PIN);
    gpio_set_dir(MOTOR_ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(MOTOR_ENCODER_B_PIN);

    g_motor.encoder_prev = readEncoderState();
    g_motor.encoder_count = 0;
    g_motor.target_count = 0;
    g_motor.ready = true;

    printf("N20 motor ready: PWM GPIO%u/%u encoder GPIO%u/%u\n",
        MOTOR_IN1_PIN, MOTOR_IN2_PIN,
        MOTOR_ENCODER_A_PIN, MOTOR_ENCODER_B_PIN);
    return true;
}

void updateEncoder() {
    if (!g_motor.ready) return;

    const uint8_t current = readEncoderState();
    const uint8_t transition = (uint8_t)((g_motor.encoder_prev << 2) | current);
    int8_t delta = QUADRATURE_DECODE_TABLE[transition];
    g_motor.encoder_count += g_motor.encoder_inverted ? -delta : delta;
    g_motor.encoder_prev = current;
}

void testEncoderCPR() {
    printf("=== ENCODER CPR TEST ===\n");
    printf("Driving motor for 5 seconds. Count the physical shaft rotations visually,\n");
    printf("or mark the output shaft and count.\n");
    printf("Starting in 2 seconds...\n");
    sleep_ms(2000);

    g_motor.encoder_count = 0;
    g_motor.encoder_prev = readEncoderState();

    absolute_time_t start = get_absolute_time();
    absolute_time_t deadline = make_timeout_time_ms(5000);
    absolute_time_t next_log = make_timeout_time_ms(500);

    int32_t prev_log_count = 0;

    while (!time_reached(deadline)) {
        updateEncoder();
        setMotorDrive(1, 600);

        if (time_reached(next_log)) {
            int32_t delta = g_motor.encoder_count - prev_log_count;
            printf("  t=%lldms  encoder=%ld  (+%ld in last 500ms)\n",
                absolute_time_diff_us(start, get_absolute_time()) / 1000,
                (long)g_motor.encoder_count,
                (long)delta);
            prev_log_count = g_motor.encoder_count;
            next_log = make_timeout_time_ms(500);
        }
        sleep_ms(1);
    }

    stopMotor();
    printf("=== RESULT ===\n");
    printf("  Total counts in 5s: %ld\n", (long)g_motor.encoder_count);
    printf("  If you counted N output shaft rotations, CPR = %ld / N\n",
        (long)g_motor.encoder_count);
    printf("  Counts/second: %ld\n", (long)(g_motor.encoder_count / 5));

    stopMotor();
    while (true) { sleep_ms(1000); }
}

void calibrateWithHardStops() {
    constexpr uint32_t STALL_MS = 1500;
    constexpr uint32_t PHASE_TIMEOUT_MS = 20000;
    constexpr uint16_t CAL_PWM = 800;
    constexpr int32_t MIN_COUNTS_BEFORE_STALL = 20;
    constexpr uint32_t INITIAL_MOVE_TIMEOUT_MS = 3000;

    printf("=== STARTUP CALIBRATION ===\n");

    // Phase 0: unconditionally drive to stop A with no stall minimum
    // This guarantees we always start from the same known position
    printf("Phase 0: seeking stop A...\n");
    g_motor.encoder_count = 0;
    g_motor.encoder_prev = readEncoderState();

    int32_t last_count = g_motor.encoder_count;
    int32_t start_count = 0;
    absolute_time_t last_change = get_absolute_time();
    absolute_time_t start_time = get_absolute_time();
    absolute_time_t phase_deadline = make_timeout_time_ms(PHASE_TIMEOUT_MS);
    bool moved_at_all = false;

    while (!time_reached(phase_deadline)) {
        updateEncoder();
        if (g_motor.encoder_count != last_count) {
            last_count = g_motor.encoder_count;
            last_change = get_absolute_time();
            moved_at_all = true;
        }
        setMotorDrive(-1, CAL_PWM);

        if (!moved_at_all) {
            if (absolute_time_diff_us(start_time, get_absolute_time()) > (int64_t)INITIAL_MOVE_TIMEOUT_MS * 1000) {
                printf("  Already at stop A\n");
                break;
            }
        } else {
            if (absolute_time_diff_us(last_change, get_absolute_time()) > (int64_t)STALL_MS * 1000) {
                printf("  Reached stop A at encoder=%ld\n", (long)g_motor.encoder_count);
                break;
            }
        }
        sleep_ms(1);
    }
    stopMotor();
    sleep_ms(500);
    g_motor.encoder_count = 0;
    g_motor.encoder_prev = readEncoderState();
    printf("  Stop A (rebased to 0)\n");
    sleep_ms(300);

    // Phase 1: now sweep the FULL range to stop B from known stop A
    printf("Phase 1: full sweep to stop B...\n");
    int32_t stop_b = 0;
    last_count = g_motor.encoder_count;
    start_count = g_motor.encoder_count;
    last_change = get_absolute_time();
    moved_at_all = false;
    start_time = get_absolute_time();
    phase_deadline = make_timeout_time_ms(PHASE_TIMEOUT_MS);
    absolute_time_t next_log = make_timeout_time_ms(500);

    while (!time_reached(phase_deadline)) {
        updateEncoder();
        if (g_motor.encoder_count != last_count) {
            last_count = g_motor.encoder_count;
            last_change = get_absolute_time();
            moved_at_all = true;
        }
        setMotorDrive(1, CAL_PWM);

        if (!moved_at_all) {
            if (absolute_time_diff_us(start_time, get_absolute_time()) > (int64_t)INITIAL_MOVE_TIMEOUT_MS * 1000) {
                printf("  ERROR: no movement from stop A toward stop B\n");
                break;
            }
        } else {
            bool moved_enough = motorAbs(g_motor.encoder_count - start_count) >= MIN_COUNTS_BEFORE_STALL;
            if (moved_enough &&
                absolute_time_diff_us(last_change, get_absolute_time()) > (int64_t)STALL_MS * 1000) {
                printf("  Reached stop B at encoder=%ld\n", (long)g_motor.encoder_count);
                break;
            }
        }
        if (time_reached(next_log)) {
            printf("  ... encoder=%ld  no-mvmt-for=%lldms\n",
                (long)g_motor.encoder_count,
                absolute_time_diff_us(last_change, get_absolute_time()) / 1000);
            next_log = make_timeout_time_ms(500);
        }
        sleep_ms(1);
    }
    stopMotor();
    stop_b = g_motor.encoder_count;
    printf("  Stop B: encoder=%ld\n", (long)stop_b);
    sleep_ms(800);

    // If stop_b is negative, the encoder counts backward relative to motor direction.
    // Negate the entire encoder count so stop_b is always positive.
    if (stop_b < 0) {
        g_motor.encoder_inverted = true;
        g_motor.encoder_count = -stop_b;
        stop_b = -stop_b;
        printf("  Encoder inverted, stop_b normalised to %ld\n", (long)stop_b);
    } else {
        g_motor.encoder_inverted = false;
        g_motor.encoder_count = stop_b;
    }
    g_motor.encoder_prev = readEncoderState();
    printf("  DEBUG: immediately after force-reset, encoder=%ld, GPIO A=%u B=%u\n",
        (long)g_motor.encoder_count,
        gpio_get(MOTOR_ENCODER_A_PIN),
        gpio_get(MOTOR_ENCODER_B_PIN));

    // Total range and center
    int32_t total_range = motorAbs(stop_b);
    int32_t center = stop_b / 2;

    printf("  Total range: %ld counts\n", (long)total_range);
    printf("  Center target: %ld\n", (long)center);

    if (total_range < 10) {
        printf("  ERROR: range too small, calibration failed\n");
        return;
    }

    int32_t counts_per_degree = total_range / 180;
    if (counts_per_degree < 1) counts_per_degree = 1;
    int32_t margin = counts_per_degree * 1;

    int32_t soft_pos_raw, soft_neg_raw;
    if (stop_b > 0) {
        soft_neg_raw = 0 + margin;
        soft_pos_raw = stop_b - margin;
    } else {
        soft_pos_raw = 0 - margin;
        soft_neg_raw = stop_b + margin;
    }
    g_motor.usable_half_range = (soft_pos_raw - soft_neg_raw) / 2;
    if (g_motor.usable_half_range < 0)
        g_motor.usable_half_range = -g_motor.usable_half_range;

    printf("  Soft limits: pos=%ld neg=%ld\n",
        (long)soft_pos_raw, (long)soft_neg_raw);
    printf("  Usable half-range: %ld counts\n", (long)g_motor.usable_half_range);

    // Phase 2: drive to center
    printf("Phase 2: returning to center (target=%ld)...\n", (long)center);
    g_motor.encoder_prev = readEncoderState();
    phase_deadline = make_timeout_time_ms(PHASE_TIMEOUT_MS);
    constexpr int32_t RETURN_TOLERANCE = 3;
    int32_t prev_error_sign = (center - g_motor.encoder_count) > 0 ? 1 : -1;
    absolute_time_t next_phase2_log = make_timeout_time_ms(200);

    while (!time_reached(phase_deadline)) {
        updateEncoder();
        int32_t error = center - g_motor.encoder_count;

        if (time_reached(next_phase2_log)) {
            printf("  P2: encoder=%ld error=%ld dir=%d\n",
                (long)g_motor.encoder_count,
                (long)error,
                error > 0 ? 1 : -1);
            next_phase2_log = make_timeout_time_ms(200);
        }

        if (motorAbs(error) <= RETURN_TOLERANCE) {
            printf("  Reached center. encoder=%ld\n", (long)g_motor.encoder_count);
            break;
        }

        int32_t current_error_sign = error > 0 ? 1 : -1;
        if (current_error_sign != prev_error_sign) {
            printf("  Overshot, stopping. encoder=%ld\n", (long)g_motor.encoder_count);
            break;
        }

        // When encoder is inverted, positive error requires negative drive direction
        int drive_dir = error > 0 ? 1 : -1;
        if (g_motor.encoder_inverted) drive_dir = -drive_dir;

        uint16_t pwm = (motorAbs(error) < 20) ? 400 : CAL_PWM;
        setMotorDrive(drive_dir, pwm);
        sleep_ms(1);
    }
    stopMotor();
    sleep_ms(500);
    g_motor.encoder_prev = readEncoderState();

    // Use actual position reached, not theoretical center
    int32_t actual_center = g_motor.encoder_count;
    printf("  Actual center position: encoder=%ld (target was %ld, error=%ld)\n",
        (long)actual_center, (long)center, (long)(actual_center - center));

    // Adjust soft limits relative to where we actually are
    g_motor.soft_pos_limit = soft_pos_raw - actual_center;
    g_motor.soft_neg_limit = soft_neg_raw - actual_center;
    g_motor.usable_half_range = (g_motor.soft_pos_limit - g_motor.soft_neg_limit) / 2;
    g_motor.encoder_count = 0;
    g_motor.target_count = 0;
    g_motor.encoder_prev = readEncoderState();

    g_motor.calibrated = true;
    printf("Calibration complete. encoder rebased to 0 at actual center.\n");
    printf("  Soft limits (final): pos=%ld neg=%ld\n",
        (long)g_motor.soft_pos_limit, (long)g_motor.soft_neg_limit);
    printf("  Usable half-range: %ld\n", (long)g_motor.usable_half_range);
}

int32_t vviToEncoderCount(int16_t vvi) {
    if (vvi < 0) return -vviToEncoderCount((int16_t)-vvi);

    static const struct { int16_t vvi; int32_t frac_num; int32_t frac_den; } pts[] = {
        {0,     0,   1},
        {500,   13,  105},
        {1000,  26,  105},
        {1500,  39,  105},
        {2000,  53,  105},
        {2500,  59,  105},
        {3000,  66,  105},
        {4000,  79,  105},
        {6000,  1,   1},
    };
    constexpr int n = sizeof(pts) / sizeof(pts[0]);

    int32_t max_count = g_motor.usable_half_range;
    if (max_count <= 0) return 0;

    if (vvi >= pts[n-1].vvi) return max_count;

    for (int i = 0; i < n-1; i++) {
        if (vvi <= pts[i+1].vvi) {
            int32_t dv = vvi - pts[i].vvi;
            int32_t dr = pts[i+1].vvi - pts[i].vvi;
            int32_t count_lo = (pts[i].frac_num * max_count) / pts[i].frac_den;
            int32_t count_hi = (pts[i+1].frac_num * max_count) / pts[i+1].frac_den;
            return count_lo + (dv * (count_hi - count_lo)) / dr;
        }
    }
    return max_count;
}

void updatePosition() {
    if (!g_motor.ready || !g_motor.calibrated) return;

    if (g_motor.target_count > g_motor.soft_pos_limit) g_motor.target_count = g_motor.soft_pos_limit;
    if (g_motor.target_count < g_motor.soft_neg_limit) g_motor.target_count = g_motor.soft_neg_limit;

    const int32_t error = g_motor.target_count - g_motor.encoder_count;
    if (motorAbs(error) <= MOTOR_POSITION_TOLERANCE_COUNTS) {
        stopMotor();
        return;
    }

    int drive_dir = error > 0 ? 1 : -1;
    if (g_motor.encoder_inverted) drive_dir = -drive_dir;

    setMotorDrive(drive_dir, chooseDriveLevel(error));
}

void motorSleep(uint32_t ms) {
    absolute_time_t deadline = make_timeout_time_ms(ms);
    while (!time_reached(deadline)) {
        updateEncoder();
        updatePosition();
        sleep_us(500);
    }
}

} // namespace

void onPltVviNeedleChange(unsigned int newValue) {
    if (!g_motor.ready || !g_motor.calibrated) return;
    int32_t raw = (int32_t)newValue - 32767;
    int16_t vvi = (int16_t)((raw * 6000) / 32767);
    int32_t pos_count = vviToEncoderCount(vvi >= 0 ? vvi : (int16_t)-vvi);
    if (vvi >= 0) {
        g_motor.target_count = pos_count;
    } else {
        g_motor.target_count = -pos_count;
    }
}
DcsBios::IntegerBuffer pltVviNeedleBuffer(0x2bc4, 0xffff, 0, onPltVviNeedleChange);

void onPltIntLightVviChange(unsigned int newValue) {
    uint8_t level = (uint8_t)((newValue * 255u) / 65535u);
    pwm_set_gpio_level(LED_PIN_EXT, level);
}
DcsBios::IntegerBuffer pltIntLightVviBuffer(0x2d7a, 0xffff, 0, onPltIntLightVviChange);

int main()
{
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);

    gpio_set_function(LED_PIN_EXT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LED_PIN_EXT);
    pwm_set_wrap(slice, 255);
    pwm_set_clkdiv(slice, 64.0f);
    pwm_set_enabled(slice, true);
    pwm_set_gpio_level(LED_PIN_EXT, 0);

    uint8_t boardAddress = USB_MODE;
    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    DcsBios::currentBoardMode = board;
    if (board.mode != DcsBios::BoardModeType::USB_ONLY) {
        DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);
        printf("DCS-BIOS mode: RS485\n");
    } else {
        printf("DCS-BIOS mode: USB_ONLY\n");
    }

    multicore_launch_core1(DcsBios::core1_task);
    DcsBios::setup();

    initMotor();
    calibrateWithHardStops();

    motorSleep(2000);
    printf("Test: commanding +half_range (%ld)...\n", (long)g_motor.usable_half_range);
    g_motor.target_count = g_motor.usable_half_range;
    motorSleep(3000);

    printf("Test: commanding -half_range (%ld)...\n", (long)-g_motor.usable_half_range);
    g_motor.target_count = -g_motor.usable_half_range;
    motorSleep(3000);

    printf("Test: commanding 0...\n");
    g_motor.target_count = 0;
    motorSleep(3000);
    printf("Test complete. encoder_count at rest: %ld\n", (long)g_motor.encoder_count);

    while (true)
    {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        updateEncoder();
        updatePosition();
        sleep_us(10);
    }
}
