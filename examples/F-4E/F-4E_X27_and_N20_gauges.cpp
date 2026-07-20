#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdint.h>
#include <stdio.h>

#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "DcsBios.h"
#include "internal/BoardMode.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/rs485.h"
#include "internal/X27_stepper.h"

namespace {

constexpr uint32_t RS485_BAUD = 250000;
constexpr unsigned char RS485_SLAVE_ADDRESS = 0xF;

// Board pin assumptions from the current 3-driver gauge board:
//   Motor A: X27 gauge on driver A
//   Motor B: X27 gauge on driver B
//   Motor C: N20 DC motor on the first two inputs of driver C
constexpr x27_gpio_config_t MOTOR_A_CFG = {
    .pin_coil1_a = 3,
    .pin_coil1_b = 4,
    .pin_coil2_a = 5,
    .pin_coil2_b = 6,
};

constexpr x27_gpio_config_t MOTOR_B_CFG = {
    .pin_coil1_a = 9,
    .pin_coil1_b = 10,
    .pin_coil2_a = 11,
    .pin_coil2_b = 12,
};

constexpr uint MOTOR_C_IN1_PIN = 16;
constexpr uint MOTOR_C_IN2_PIN = 17;
constexpr uint MOTOR_C_UNUSED_PINS[] = {14, 15, 18, 19, 26, 27};
constexpr uint MOTOR_C_ENCODER_A_PIN = 34;
constexpr uint MOTOR_C_ENCODER_B_PIN = 35;
constexpr uint16_t MOTOR_C_PWM_WRAP = 1000;
constexpr float MOTOR_C_PWM_CLKDIV = 125.0f;
constexpr uint16_t MOTOR_C_PWM_DRIVE_LEVEL = 1000;
constexpr uint16_t MOTOR_C_PWM_APPROACH_LEVEL = 550;
constexpr float MOTOR_C_ENCODER_COUNTS_PER_MOTOR_REV = 7.0f;
constexpr float MOTOR_C_GEAR_RATIO = 30.0f;
constexpr int32_t MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV =
    (int32_t)(MOTOR_C_ENCODER_COUNTS_PER_MOTOR_REV * MOTOR_C_GEAR_RATIO + 0.5f);
constexpr float MOTOR_C_COUNTS_PER_QUADRANT =
    MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV / 4.0f;
constexpr int32_t MOTOR_C_STOPS_PER_REV = 4;
constexpr int32_t MOTOR_C_TOTAL_SWEEP_ROTATIONS = 5;
constexpr int32_t MOTOR_C_TOTAL_STOPS =
    MOTOR_C_TOTAL_SWEEP_ROTATIONS * MOTOR_C_STOPS_PER_REV;
constexpr int32_t MOTOR_C_MAX_TARGET_COUNT =
    (int32_t)(MOTOR_C_TOTAL_STOPS * MOTOR_C_COUNTS_PER_QUADRANT + 0.5f);
constexpr int32_t MOTOR_C_POSITION_TOLERANCE_COUNTS = 6;
constexpr int32_t MOTOR_C_SLOWDOWN_WINDOW_COUNTS =
    (int32_t)(MOTOR_C_COUNTS_PER_QUADRANT / 2.0f + 0.5f);
constexpr uint32_t MOTOR_C_ENCODER_LOG_MS = 250;
constexpr uint32_t MOTOR_C_STOP_DWELL_MS = 1000;

constexpr x27_step_mode_t MOTOR_STEP_MODE = X27_MODE_HALF_STEP;
constexpr uint32_t MOTOR_STEP_DELAY_US = 2200;
constexpr bool RUN_MECHANICAL_HOME_ON_BOOT = true;
constexpr signed char HOME_DIRECTION = -1;
constexpr uint32_t HOME_MAX_STEPS = 1300;

// Flip per-motor direction here if a gauge runs backward.
constexpr bool MOTOR_A_DIRECTION_INVERTED = false;
constexpr bool MOTOR_B_DIRECTION_INVERTED = false;

x27_motor_t g_motorA = {};
x27_motor_t g_motorB = {};
bool g_motorAReady = false;
bool g_motorBReady = false;

enum class MotorCTestPhase {
    Seek,
    Dwell,
};

struct N20MotorState {
    bool ready = false;
    uint pwm_slice = 0;
    MotorCTestPhase phase = MotorCTestPhase::Dwell;
    absolute_time_t phase_deadline = {};
    absolute_time_t next_encoder_log = {};
    uint8_t encoder_prev = 0;
    int32_t encoder_count = 0;
    int32_t last_reported_encoder_count = 0;
    int32_t segment_start_count = 0;
    int32_t target_stop_index = 0;
    int32_t target_count = 0;
    int32_t motion_direction = 1;
};

N20MotorState g_motorC = {};
uart_inst_t* rs485_uart = uart0;

constexpr int8_t QUADRATURE_DECODE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

int32_t rawToX27Steps(unsigned int raw) {
    return (int32_t)(((uint32_t)raw * (uint32_t)X27_MAX_POSITION) / 65535u);
}

uint8_t readMotorCEncoderState() {
    return (uint8_t)((gpio_get(MOTOR_C_ENCODER_A_PIN) ? 0x2u : 0u) |
                     (gpio_get(MOTOR_C_ENCODER_B_PIN) ? 0x1u : 0u));
}

int32_t motorCAbs(int32_t value) {
    return value >= 0 ? value : -value;
}

void stopMotorC() {
    pwm_set_gpio_level(MOTOR_C_IN1_PIN, 0);
    pwm_set_gpio_level(MOTOR_C_IN2_PIN, 0);
}

void setMotorCDrive(int direction, uint16_t drive_level) {
    if (direction > 0) {
        pwm_set_gpio_level(MOTOR_C_IN1_PIN, drive_level);
        pwm_set_gpio_level(MOTOR_C_IN2_PIN, 0);
    } else if (direction < 0) {
        pwm_set_gpio_level(MOTOR_C_IN1_PIN, 0);
        pwm_set_gpio_level(MOTOR_C_IN2_PIN, drive_level);
    } else {
        stopMotorC();
    }
}

uint16_t chooseMotorCDriveLevel(int32_t error) {
    if (motorCAbs(error) <= MOTOR_C_SLOWDOWN_WINDOW_COUNTS) {
        return MOTOR_C_PWM_APPROACH_LEVEL;
    }

    return MOTOR_C_PWM_DRIVE_LEVEL;
}

void logMotorCTarget(const char* prefix) {
    printf("Motor C %s target=%ld delta=%ld stop=%ld angle=%ld deg\n",
           prefix,
           (long)g_motorC.target_count,
           (long)(g_motorC.target_count - g_motorC.segment_start_count),
           (long)g_motorC.target_stop_index,
           (long)(g_motorC.target_stop_index * 90));
}

void advanceMotorCTarget() {
    int32_t next_stop_index = g_motorC.target_stop_index + g_motorC.motion_direction;

    if (next_stop_index > MOTOR_C_TOTAL_STOPS) {
        g_motorC.motion_direction = -1;
        next_stop_index = g_motorC.target_stop_index - 1;
    } else if (next_stop_index < 0) {
        g_motorC.motion_direction = 1;
        next_stop_index = g_motorC.target_stop_index + 1;
    }

    g_motorC.target_stop_index = next_stop_index;
    g_motorC.segment_start_count = g_motorC.encoder_count;
    g_motorC.target_count =
        g_motorC.segment_start_count + (g_motorC.motion_direction * (int32_t)(MOTOR_C_COUNTS_PER_QUADRANT + 0.5f));
    logMotorCTarget("seeking");
}

bool initMotorC() {
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
        printf("Motor C PWM slice mismatch: GPIO%u -> %u, GPIO%u -> %u\n",
               MOTOR_C_IN1_PIN,
               slice_in1,
               MOTOR_C_IN2_PIN,
               slice_in2);
        return false;
    }

    g_motorC.pwm_slice = slice_in1;
    pwm_set_wrap(g_motorC.pwm_slice, MOTOR_C_PWM_WRAP);
    pwm_set_clkdiv(g_motorC.pwm_slice, MOTOR_C_PWM_CLKDIV);
    stopMotorC();
    pwm_set_enabled(g_motorC.pwm_slice, true);

    gpio_init(MOTOR_C_ENCODER_A_PIN);
    gpio_set_dir(MOTOR_C_ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(MOTOR_C_ENCODER_A_PIN);

    gpio_init(MOTOR_C_ENCODER_B_PIN);
    gpio_set_dir(MOTOR_C_ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(MOTOR_C_ENCODER_B_PIN);

    g_motorC.encoder_prev = readMotorCEncoderState();
    g_motorC.encoder_count = 0;
    g_motorC.last_reported_encoder_count = 0;
    g_motorC.segment_start_count = 0;
    g_motorC.target_stop_index = 0;
    g_motorC.target_count = 0;
    g_motorC.motion_direction = 1;
    g_motorC.phase = MotorCTestPhase::Dwell;
    g_motorC.phase_deadline = make_timeout_time_ms(MOTOR_C_STOP_DWELL_MS);
    g_motorC.next_encoder_log = make_timeout_time_ms(MOTOR_C_ENCODER_LOG_MS);
    g_motorC.ready = true;

    printf("Motor C ready: PWM GPIO%u/%u encoder GPIO%u/%u\n",
        MOTOR_C_IN1_PIN,
        MOTOR_C_IN2_PIN,
        MOTOR_C_ENCODER_A_PIN,
        MOTOR_C_ENCODER_B_PIN);
    printf("Motor C tuning: counts/rev=%ld stop interval=%ld counts max angle=%ld deg\n",
        (long)MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV,
            (long)((int32_t)(MOTOR_C_COUNTS_PER_QUADRANT + 0.5f)),
        (long)(MOTOR_C_TOTAL_STOPS * 90));
    printf("Motor C drive levels: cruise=%u approach=%u dwell=%lu ms\n",
        MOTOR_C_PWM_DRIVE_LEVEL,
        MOTOR_C_PWM_APPROACH_LEVEL,
        (unsigned long)MOTOR_C_STOP_DWELL_MS);
    return true;
}

void updateMotorCEncoder() {
    if (!g_motorC.ready) return;

    const uint8_t current = readMotorCEncoderState();
    const uint8_t transition = (uint8_t)((g_motorC.encoder_prev << 2) | current);
    g_motorC.encoder_count += QUADRATURE_DECODE_TABLE[transition];
    g_motorC.encoder_prev = current;
}

void maybeLogMotorCEncoder() {
    if (!g_motorC.ready) return;
    if (!time_reached(g_motorC.next_encoder_log)) return;

    const int32_t delta = g_motorC.encoder_count - g_motorC.last_reported_encoder_count;
    const int32_t counts_per_second = (delta * 1000) / (int32_t)MOTOR_C_ENCODER_LOG_MS;
    const int32_t rpm_x10 =
        (counts_per_second * 600) / MOTOR_C_ENCODER_COUNTS_PER_OUTPUT_REV;
    printf("Motor C encoder count=%ld delta=%ld cps=%ld rpm=%ld.%ld target=%ld\n",
           (long)g_motorC.encoder_count,
           (long)delta,
           (long)counts_per_second,
           (long)(rpm_x10 / 10),
           (long)motorCAbs(rpm_x10 % 10),
           (long)g_motorC.target_count);

    g_motorC.last_reported_encoder_count = g_motorC.encoder_count;
    g_motorC.next_encoder_log = make_timeout_time_ms(MOTOR_C_ENCODER_LOG_MS);
}

void updateMotorCSweep() {
    if (!g_motorC.ready) return;
    if (g_motorC.phase == MotorCTestPhase::Dwell) {
        stopMotorC();
        if (!time_reached(g_motorC.phase_deadline)) return;

        advanceMotorCTarget();
        g_motorC.phase = MotorCTestPhase::Seek;
        return;
    }

    const int32_t error = g_motorC.target_count - g_motorC.encoder_count;
    if (motorCAbs(error) <= MOTOR_C_POSITION_TOLERANCE_COUNTS) {
        stopMotorC();
        g_motorC.phase = MotorCTestPhase::Dwell;
        g_motorC.phase_deadline = make_timeout_time_ms(MOTOR_C_STOP_DWELL_MS);
         printf("Motor C stop reached: target=%ld actual=%ld error=%ld delta=%ld\n",
               (long)g_motorC.target_count,
               (long)g_motorC.encoder_count,
             (long)error,
             (long)(g_motorC.encoder_count - g_motorC.segment_start_count));
        return;
    }

    setMotorCDrive(error > 0 ? 1 : -1, chooseMotorCDriveLevel(error));
}

bool initMotor(x27_motor_t* motor,
               bool* ready,
               const x27_gpio_config_t* cfg,
               const char* name,
               bool directionInverted) {
    if (!x27_init_gpio(motor, cfg, MOTOR_STEP_MODE)) {
        printf("%s init failed\n", name);
        *ready = false;
        return false;
    }

    x27_set_speed(motor, MOTOR_STEP_DELAY_US);
    x27_set_direction_inverted(motor, directionInverted);

    if (RUN_MECHANICAL_HOME_ON_BOOT) {
        x27_home_to_stop(motor, HOME_DIRECTION, HOME_MAX_STEPS);
    }

    motor->current_position = 0;
    motor->target_position = 0;

    printf("%s ready on pins %u %u %u %u\n",
           name,
           cfg->pin_coil1_a,
           cfg->pin_coil1_b,
           cfg->pin_coil2_a,
           cfg->pin_coil2_b);

    *ready = true;
    return true;
}

void updateMotors() {
    if (g_motorAReady) x27_update(&g_motorA);
    if (g_motorBReady) x27_update(&g_motorB);
    updateMotorCEncoder();
    updateMotorCSweep();
    maybeLogMotorCEncoder();
}

}  // namespace

void onPltVviNeedleChange(unsigned int newValue) {
    if (!g_motorAReady) return;
    x27_set_position(&g_motorA, rawToX27Steps(newValue));
}
DcsBios::IntegerBuffer pltVviNeedleBuffer(0x2bc4, 0xffff, 0, onPltVviNeedleChange);

void onPltRadarAltNeedleChange(unsigned int newValue) {
    if (!g_motorBReady) return;
    x27_set_position(&g_motorB, rawToX27Steps(newValue));
}
DcsBios::IntegerBuffer pltRadarAltNeedleBuffer(0x2a8a, 0xffff, 0, onPltRadarAltNeedleChange);

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(1500);

    DcsBios::BoardMode board = DcsBios::determineBoardMode(RS485_SLAVE_ADDRESS);
    DcsBios::currentBoardMode = board;

    if (board.mode != DcsBios::BoardModeType::SLAVE) {
        printf("Invalid board mode for address 0x%X\n", RS485_SLAVE_ADDRESS);
        while (true) {
            DcsBios::updateHeartbeat();
            sleep_ms(100);
        }
    }

    printf("Starting RS485 slave address 0x%X\n", RS485_SLAVE_ADDRESS);
    printf("RS485 UART0: TX=%d RX=%d EN=%d baud=%lu\n",
           UART0_TX,
           UART0_RX,
           RS485_EN,
           (unsigned long)RS485_BAUD);

    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, RS485_BAUD);
    multicore_launch_core1(DcsBios::core1_task);
    DcsBios::setup();

    initMotor(&g_motorA, &g_motorAReady, &MOTOR_A_CFG, "Motor A (VVI)", MOTOR_A_DIRECTION_INVERTED);
    initMotor(&g_motorB, &g_motorBReady, &MOTOR_B_CFG, "Motor B (Radar Alt)", MOTOR_B_DIRECTION_INVERTED);
    initMotorC();

    while (true) {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        updateMotors();
        sleep_us(20);
    }

    return 0;
}
