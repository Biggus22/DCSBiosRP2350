#include <stdint.h>
#include <stdio.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

namespace {

constexpr uint MOTOR_C_UNUSED_PINS[] = {14u, 15u, 18u, 19u, 26u, 27u};
constexpr uint MOTOR_C_IN1_PIN = 16u;
constexpr uint MOTOR_C_IN2_PIN = 17u;
constexpr uint MOTOR_C_ENCODER_A_PIN = 34u;
constexpr uint MOTOR_C_ENCODER_B_PIN = 35u;

constexpr uint32_t STARTUP_DELAY_MS = 1500u;
constexpr uint16_t MOTOR_C_PWM_WRAP = 1000u;
constexpr float MOTOR_C_PWM_CLKDIV = 125.0f;
constexpr uint16_t MOTOR_C_PWM_DRIVE_LEVEL = 1000u;
constexpr uint16_t MOTOR_C_PWM_APPROACH_LEVEL = 550u;
constexpr float MOTOR_C_ENCODER_COUNTS_PER_MOTOR_REV = 7.0f;
constexpr float MOTOR_C_GEAR_RATIO = 30.0f;
constexpr int32_t MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV =
    (int32_t)(MOTOR_C_ENCODER_COUNTS_PER_MOTOR_REV * MOTOR_C_GEAR_RATIO + 0.5f);
constexpr int32_t MOTOR_C_FORWARD_STEP_DEG = 30;
constexpr int32_t MOTOR_C_FORWARD_MAX_ANGLE_DEG = 360;
constexpr int32_t MOTOR_C_REVERSE_STEP_DEG = 90;
constexpr int32_t MOTOR_C_POSITION_TOLERANCE_COUNTS = 6;
constexpr int32_t MOTOR_C_SLOWDOWN_WINDOW_COUNTS =
    (MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV + 7) / 12;
constexpr uint32_t MOTOR_C_STOP_DWELL_MS = 500u;
constexpr uint32_t MOTOR_C_ENCODER_LOG_MS = 250u;
constexpr uint32_t LOOP_DELAY_US = 20u;

constexpr int8_t QUADRATURE_DECODE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

enum class MotorPhase {
    Seek,
    Dwell,
};

enum class SweepPhase {
    ForwardFine,
    ReverseCoarse,
};

struct MotorState {
    uint pwm_slice = 0;
    uint8_t encoder_prev = 0;
    int32_t encoder_count = 0;
    int32_t last_reported_encoder_count = 0;
    int32_t target_count = 0;
    int32_t segment_start_count = 0;
    uint32_t segment_index = 0;
    int32_t motion_direction = 1;
    int32_t target_angle_deg = 0;
    MotorPhase phase = MotorPhase::Dwell;
    SweepPhase sweep_phase = SweepPhase::ForwardFine;
    absolute_time_t phase_deadline = {};
    absolute_time_t next_encoder_log = {};
};

MotorState g_motor = {};

// Returns the absolute value of the input integer.
int32_t motorAbs(int32_t value) {
    return value >= 0 ? value : -value;
}

// Converts the input angle in degrees to encoder counts.
int32_t angleToCounts(int32_t angle_deg) {
    return (angle_deg * MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV + 180) / 360;
}

// Reads the state of the encoder pins.
uint8_t readEncoderState() {
    return (uint8_t)((gpio_get(MOTOR_C_ENCODER_A_PIN) ? 0x2u : 0u) |
                     (gpio_get(MOTOR_C_ENCODER_B_PIN) ? 0x1u : 0u));
}

// Sets the output level of both motor control pins to zero.
void stopMotor() {
    pwm_set_gpio_level(MOTOR_C_IN1_PIN, 0);
    pwm_set_gpio_level(MOTOR_C_IN2_PIN, 0);
}

// Sets the motor drive level based on direction and level. If direction is positive, it sets IN1 level and IN2 level to zero.
void setMotorDrive(int direction, uint16_t drive_level) {
    if (direction > 0) {
        pwm_set_gpio_level(MOTOR_C_IN1_PIN, drive_level);
        pwm_set_gpio_level(MOTOR_C_IN2_PIN, 0);
    } else if (direction < 0) {
        pwm_set_gpio_level(MOTOR_C_IN1_PIN, 0);
        pwm_set_gpio_level(MOTOR_C_IN2_PIN, drive_level);
    } else {
        stopMotor();
    }
}

uint16_t chooseDriveLevel(int32_t error) {
    if (motorAbs(error) <= MOTOR_C_SLOWDOWN_WINDOW_COUNTS) {
        return MOTOR_C_PWM_APPROACH_LEVEL;
    }

    return MOTOR_C_PWM_DRIVE_LEVEL;
}

void logTarget(const char* prefix) {
    const char* sweep_name =
        g_motor.sweep_phase == SweepPhase::ForwardFine ? "forward-30" : "reverse-90";
    printf("%s [%s] target=%ld delta=%ld angle=%ld deg\n",
           prefix,
           sweep_name,
           (long)g_motor.target_count,
           (long)(g_motor.target_count - g_motor.segment_start_count),
           (long)g_motor.target_angle_deg);
}

void advanceTarget() {
    int32_t step_angle_deg = 0;

    if (g_motor.sweep_phase == SweepPhase::ForwardFine) {
        if (g_motor.target_angle_deg < MOTOR_C_FORWARD_MAX_ANGLE_DEG) {
            step_angle_deg = MOTOR_C_FORWARD_STEP_DEG;
            g_motor.target_angle_deg += step_angle_deg;
        } else {
            g_motor.sweep_phase = SweepPhase::ReverseCoarse;
            step_angle_deg = -MOTOR_C_REVERSE_STEP_DEG;
            g_motor.target_angle_deg += step_angle_deg;
        }
    } else {
        if (g_motor.target_angle_deg > 0) {
            step_angle_deg = -MOTOR_C_REVERSE_STEP_DEG;
            g_motor.target_angle_deg += step_angle_deg;
        }
        if (g_motor.target_angle_deg <= 0) {
            g_motor.target_angle_deg = 0;
        }
    }

    g_motor.segment_start_count = g_motor.encoder_count;
    ++g_motor.segment_index;
    g_motor.target_count = g_motor.segment_start_count + angleToCounts(step_angle_deg);
    const int32_t delta = g_motor.target_count - g_motor.segment_start_count;
    g_motor.motion_direction = (delta > 0) ? 1 : -1;
    logTarget("Seek");
}

void updateEncoder() {
    const uint8_t current = readEncoderState();
    const uint8_t transition = (uint8_t)((g_motor.encoder_prev << 2) | current);
    g_motor.encoder_count += QUADRATURE_DECODE_TABLE[transition];
    g_motor.encoder_prev = current;
}

void maybeLogEncoder() {
    if (!time_reached(g_motor.next_encoder_log)) return;

    const int32_t delta = g_motor.encoder_count - g_motor.last_reported_encoder_count;
    const int32_t counts_per_second = (delta * 1000) / (int32_t)MOTOR_C_ENCODER_LOG_MS;
    const int32_t rpm_x10 =
        (counts_per_second * 600) / MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV;
    const int32_t error = g_motor.target_count - g_motor.encoder_count;

    printf("Encoder=%ld delta=%ld cps=%ld rpm=%ld.%ld target=%ld error=%ld\n",
           (long)g_motor.encoder_count,
           (long)delta,
           (long)counts_per_second,
           (long)(rpm_x10 / 10),
           (long)motorAbs(rpm_x10 % 10),
           (long)g_motor.target_count,
           (long)error);

    g_motor.last_reported_encoder_count = g_motor.encoder_count;
    g_motor.next_encoder_log = make_timeout_time_ms(MOTOR_C_ENCODER_LOG_MS);
}

void updateSweep() {
    if (g_motor.phase == MotorPhase::Dwell) {
        stopMotor();
        if (!time_reached(g_motor.phase_deadline)) return;

        if (g_motor.sweep_phase == SweepPhase::ReverseCoarse && g_motor.target_angle_deg == 0) {
            g_motor.sweep_phase = SweepPhase::ForwardFine;
        }

        advanceTarget();
        g_motor.phase = MotorPhase::Seek;
        return;
    }

    const int32_t error = g_motor.target_count - g_motor.encoder_count;
    if (motorAbs(error) <= MOTOR_C_POSITION_TOLERANCE_COUNTS) {
        stopMotor();
        g_motor.phase = MotorPhase::Dwell;
        g_motor.phase_deadline = make_timeout_time_ms(MOTOR_C_STOP_DWELL_MS);
         const int32_t delta = g_motor.encoder_count - g_motor.segment_start_count;
         printf("Stop reached: segment=%lu target=%ld actual=%ld error=%ld delta=%ld implied_cpr_if_180=%ld\n",
             (unsigned long)g_motor.segment_index,
               (long)g_motor.target_count,
               (long)g_motor.encoder_count,
             (long)error,
             (long)delta,
             (long)(motorAbs(delta) * 2));
        return;
    }

    setMotorDrive(error > 0 ? 1 : -1, chooseDriveLevel(error));
}

}  // namespace

int main() {
    stdio_init_all();

    for (uint pin : MOTOR_C_UNUSED_PINS) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }

    gpio_set_function(MOTOR_C_IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_C_IN2_PIN, GPIO_FUNC_PWM);
    const uint slice_in1 = pwm_gpio_to_slice_num(MOTOR_C_IN1_PIN);
    const uint slice_in2 = pwm_gpio_to_slice_num(MOTOR_C_IN2_PIN);
    if (slice_in1 != slice_in2) {
        printf("PWM slice mismatch: GPIO%u -> %u, GPIO%u -> %u\n",
               MOTOR_C_IN1_PIN,
               slice_in1,
               MOTOR_C_IN2_PIN,
               slice_in2);
        while (true) {
            sleep_ms(1000);
        }
    }

    g_motor.pwm_slice = slice_in1;
    pwm_set_wrap(g_motor.pwm_slice, MOTOR_C_PWM_WRAP);
    pwm_set_clkdiv(g_motor.pwm_slice, MOTOR_C_PWM_CLKDIV);
    stopMotor();
    pwm_set_enabled(g_motor.pwm_slice, true);

    gpio_init(MOTOR_C_ENCODER_A_PIN);
    gpio_set_dir(MOTOR_C_ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(MOTOR_C_ENCODER_A_PIN);
    gpio_init(MOTOR_C_ENCODER_B_PIN);
    gpio_set_dir(MOTOR_C_ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(MOTOR_C_ENCODER_B_PIN);

    g_motor.encoder_prev = readEncoderState();
    g_motor.phase_deadline = make_timeout_time_ms(MOTOR_C_STOP_DWELL_MS);
    g_motor.next_encoder_log = make_timeout_time_ms(MOTOR_C_ENCODER_LOG_MS);

    sleep_ms(STARTUP_DELAY_MS);
    printf("Motor C MX1508 30-degree forward / 90-degree reverse sweep test\n");
    printf("PWM pins: IN1=%u IN2=%u encoder: A=%u B=%u\n",
           MOTOR_C_IN1_PIN,
           MOTOR_C_IN2_PIN,
           MOTOR_C_ENCODER_A_PIN,
           MOTOR_C_ENCODER_B_PIN);
    printf("Counts/rev=%ld 30deg=%ld 90deg=%ld forward_max=%ld dwell=%lu ms\n",
           (long)MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV,
        (long)angleToCounts(30),
        (long)angleToCounts(90),
        (long)MOTOR_C_FORWARD_MAX_ANGLE_DEG,
        (unsigned long)MOTOR_C_STOP_DWELL_MS);
    printf("Drive=%u approach=%u tolerance=%ld dwell=%lu ms\n",
           MOTOR_C_PWM_DRIVE_LEVEL,
           MOTOR_C_PWM_APPROACH_LEVEL,
           (long)MOTOR_C_POSITION_TOLERANCE_COUNTS,
           (unsigned long)MOTOR_C_STOP_DWELL_MS);
    logTarget("Start");

    while (true) {
        updateEncoder();
        updateSweep();
        maybeLogEncoder();
        sleep_us(LOOP_DELAY_US);
    }

    return 0;
}