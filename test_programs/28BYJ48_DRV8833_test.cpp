// Dual 28BYJ-48 DRV8833 test program for RP2350-Zero.
// This replaces the copied X27 test with native 28BYJ driver logic.
//
// Motor A (confirmed from X27 DRV8833 test PCB):
//   IN1=GPIO3, IN2=GPIO4, IN3=GPIO5, IN4=GPIO6
//
// Motor B (second DRV8833 channel on same PCB layout):
//   IN1=GPIO9, IN2=GPIO10, IN3=GPIO11, IN4=GPIO12
//   If your board is wired differently, only update MOTOR_B_PINS below.

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "src/internal/BYJ48_stepper.h"

namespace {

constexpr uint MOTOR_A_PINS[4] = {3, 4, 5, 6};
constexpr uint MOTOR_B_PINS[4] = {9, 10, 11, 12};
constexpr uint BACKLIGHT_PWM_PIN = 8;
constexpr int MOTOR_A_ZERO_SENSOR_PIN = 7;
constexpr int MOTOR_B_ZERO_SENSOR_PIN = 13;
constexpr int MOTOR_C_ZERO_SENSOR_PIN = 28;  // Reserved for future Motor C bring-up.
constexpr bool MOTOR_A_ZERO_SENSOR_ACTIVE_LOW = true;
constexpr bool MOTOR_A_ZERO_SENSOR_ENABLE_PULLUP = true;
constexpr bool MOTOR_B_ZERO_SENSOR_ACTIVE_LOW = true;
constexpr bool MOTOR_B_ZERO_SENSOR_ENABLE_PULLUP = true;

constexpr byj_step_mode_t STEP_MODE = BYJ_MODE_FULL_SINGLE;
constexpr byj_output_mode_t OUTPUT_MODE = BYJ_OUTPUT_BIPOLAR;
constexpr uint32_t STEP_DELAY_US = 1500;
constexpr int32_t SWEEP_STEPS = 2048;
constexpr int32_t INTERLEAVE_CHUNK_STEPS = 6;
constexpr uint32_t ENDPOINT_DWELL_MS = 400;
constexpr bool INVERT_MOTOR_A_DIRECTION = false;
constexpr bool INVERT_MOTOR_B_DIRECTION = true;
constexpr int MOTOR_A_COUNTERCLOCKWISE_DIRECTION = 1;
constexpr int MOTOR_B_COUNTERCLOCKWISE_DIRECTION = 1;
constexpr int32_t BOOSTPUMP_SWITCH_GUARD_STEPS = 32;
constexpr int32_t BOOSTPUMP_ZERO_L = BYJ48_STEPS_PER_REV - BOOSTPUMP_SWITCH_GUARD_STEPS;
constexpr int32_t BOOSTPUMP_ZERO_R = BYJ48_STEPS_PER_REV - BOOSTPUMP_SWITCH_GUARD_STEPS;
constexpr bool RUN_STEPS_PER_REV_CALIBRATION_ON_BOOT = false;
constexpr int CALIBRATION_SAMPLES = 3;
constexpr int32_t CALIBRATION_MAX_SEARCH_STEPS = BYJ48_STEPS_PER_REV * 2;
constexpr int32_t CALIBRATION_MAX_ACTIVE_WINDOW_STEPS = BYJ48_STEPS_PER_REV / 2;
constexpr int32_t SENSOR_REFINEMENT_EXTRA_BACKOFF_STEPS = 8;
constexpr bool RUN_PRECISION_SECOND_PASS = false;
constexpr uint32_t CALIBRATION_COARSE_DELAY_US = 700;
constexpr uint32_t CALIBRATION_MEASURE_DELAY_US = 700;
constexpr uint32_t REFINED_APPROACH_DELAY_US = 1000;
constexpr uint32_t PRECISION_MEASURE_DELAY_US = 1200;
constexpr int SENSOR_DEBOUNCE_READS = 5;
constexpr int SENSOR_STATE_CONFIRM_STEPS = 2;
constexpr char CALIBRATION_COMMAND_A = 'a';
constexpr char CALIBRATION_COMMAND = 'c';

bool g_motor_a_zero_sensor_ready = false;
bool g_motor_a_zero_sensor_last_active = false;
bool g_zero_sensor_ready = false;
bool g_zero_sensor_last_active = false;
int32_t g_motor_a_active_steps = 0;
int32_t g_motor_b_active_steps = 0;
int32_t g_motor_a_last_active_window_steps = 0;
int32_t g_motor_b_last_active_window_steps = 0;
int32_t g_motor_a_last_cw_steps = 0;
int32_t g_motor_a_last_ccw_steps = 0;
int32_t g_motor_b_last_cw_steps = 0;
int32_t g_motor_b_last_ccw_steps = 0;
int32_t g_motor_a_last_cw_active_steps = 0;
int32_t g_motor_a_last_ccw_active_steps = 0;
int32_t g_motor_b_last_cw_active_steps = 0;
int32_t g_motor_b_last_ccw_active_steps = 0;

using sensor_active_fn = bool (*)();
using sensor_raw_fn = int (*)();
using step_single_fn = void (*)(byj_motor_t*, int);

int read_motor_a_zero_sensor_raw() {
    if (MOTOR_A_ZERO_SENSOR_PIN < 0) {
        return -1;
    }
    return gpio_get((uint)MOTOR_A_ZERO_SENSOR_PIN);
}

int read_motor_b_zero_sensor_raw() {
    if (MOTOR_B_ZERO_SENSOR_PIN < 0) {
        return -1;
    }
    return gpio_get((uint)MOTOR_B_ZERO_SENSOR_PIN);
}

bool majority_vote_active(bool (*sample_fn)()) {
    int active_count = 0;
    for (int i = 0; i < SENSOR_DEBOUNCE_READS; ++i) {
        if (sample_fn()) {
            active_count++;
        }
    }
    return active_count >= ((SENSOR_DEBOUNCE_READS + 1) / 2);
}

bool read_motor_a_zero_sensor_active_single_sample() {
    if (!g_motor_a_zero_sensor_ready || MOTOR_A_ZERO_SENSOR_PIN < 0) {
        return false;
    }

    const int raw = read_motor_a_zero_sensor_raw();
    return MOTOR_A_ZERO_SENSOR_ACTIVE_LOW ? (raw == 0) : (raw != 0);
}

bool read_motor_a_zero_sensor_active() {
    return majority_vote_active(read_motor_a_zero_sensor_active_single_sample);
}

void poll_motor_a_zero_sensor_edge() {
    if (!g_motor_a_zero_sensor_ready) {
        return;
    }

    const bool now_active = read_motor_a_zero_sensor_active();
    if (now_active && !g_motor_a_zero_sensor_last_active) {
        g_motor_a_active_steps = 0;
        printf("Motor A ZERO sensor triggered on GPIO%d (microswitch active)\n", MOTOR_A_ZERO_SENSOR_PIN);
    } else if (!now_active && g_motor_a_zero_sensor_last_active) {
        g_motor_a_last_active_window_steps = g_motor_a_active_steps;
        printf("Motor A ZERO sensor released on GPIO%d (active steps=%ld)\n",
               MOTOR_A_ZERO_SENSOR_PIN,
               (long)g_motor_a_last_active_window_steps);
    }
    g_motor_a_zero_sensor_last_active = now_active;
}

bool wait_for_sensor_state(
    byj_motor_t* motor,
    sensor_active_fn sensor_active,
    step_single_fn step_single,
    int direction,
    bool target_active,
    int32_t max_steps,
    int32_t* steps_taken
) {
    int32_t steps = 0;
    int stable_count = 0;

    while (steps < max_steps) {
        if (sensor_active() == target_active) {
            stable_count++;
            if (stable_count >= SENSOR_STATE_CONFIRM_STEPS) {
                if (steps_taken) {
                    *steps_taken = steps;
                }
                return true;
            }
        } else {
            stable_count = 0;
        }

        step_single(motor, direction);
        steps++;
    }

    if (steps_taken) {
        *steps_taken = steps;
    }
    return false;
}

bool read_zero_sensor_active_single_sample() {
    if (!g_zero_sensor_ready || MOTOR_B_ZERO_SENSOR_PIN < 0) {
        return false;
    }

    const int raw = read_motor_b_zero_sensor_raw();
    return MOTOR_B_ZERO_SENSOR_ACTIVE_LOW ? (raw == 0) : (raw != 0);
}

bool read_zero_sensor_active() {
    return majority_vote_active(read_zero_sensor_active_single_sample);
}

void poll_zero_sensor_edge() {
    if (!g_zero_sensor_ready) {
        return;
    }

    const bool now_active = read_zero_sensor_active();
    if (now_active && !g_zero_sensor_last_active) {
        g_motor_b_active_steps = 0;
        printf("Motor B ZERO sensor triggered on GPIO%d (microswitch active)\n", MOTOR_B_ZERO_SENSOR_PIN);
    } else if (!now_active && g_zero_sensor_last_active) {
        g_motor_b_last_active_window_steps = g_motor_b_active_steps;
        printf("Motor B ZERO sensor released on GPIO%d (active steps=%ld)\n",
               MOTOR_B_ZERO_SENSOR_PIN,
               (long)g_motor_b_last_active_window_steps);
    }
    g_zero_sensor_last_active = now_active;
}

void init_zero_sensor() {
    if (MOTOR_B_ZERO_SENSOR_PIN < 0) {
        printf("ZERO sensor disabled (pin < 0)\n");
        return;
    }

    gpio_init((uint)MOTOR_B_ZERO_SENSOR_PIN);
    gpio_set_dir((uint)MOTOR_B_ZERO_SENSOR_PIN, GPIO_IN);
    if (MOTOR_B_ZERO_SENSOR_ENABLE_PULLUP) {
        gpio_pull_up((uint)MOTOR_B_ZERO_SENSOR_PIN);
    } else {
        gpio_disable_pulls((uint)MOTOR_B_ZERO_SENSOR_PIN);
    }

    g_zero_sensor_ready = true;
    g_zero_sensor_last_active = read_zero_sensor_active();

    printf("Motor B ZERO sensor configured: pin=%d active_%s pullup=%d initial=%d\n",
           MOTOR_B_ZERO_SENSOR_PIN,
            MOTOR_B_ZERO_SENSOR_ACTIVE_LOW ? "low" : "high",
            MOTOR_B_ZERO_SENSOR_ENABLE_PULLUP ? 1 : 0,
           g_zero_sensor_last_active ? 1 : 0);
        printf("Motor B ZERO raw level now: %d\n", read_motor_b_zero_sensor_raw());

    printf("Motor C ZERO sensor reserved on GPIO%d (motor C not active in this test)\n", MOTOR_C_ZERO_SENSOR_PIN);
}

void init_motor_a_zero_sensor() {
    if (MOTOR_A_ZERO_SENSOR_PIN < 0) {
        printf("Motor A ZERO sensor disabled (pin < 0)\n");
        return;
    }

    gpio_init((uint)MOTOR_A_ZERO_SENSOR_PIN);
    gpio_set_dir((uint)MOTOR_A_ZERO_SENSOR_PIN, GPIO_IN);
    if (MOTOR_A_ZERO_SENSOR_ENABLE_PULLUP) {
        gpio_pull_up((uint)MOTOR_A_ZERO_SENSOR_PIN);
    } else {
        gpio_disable_pulls((uint)MOTOR_A_ZERO_SENSOR_PIN);
    }

    g_motor_a_zero_sensor_ready = true;
    g_motor_a_zero_sensor_last_active = read_motor_a_zero_sensor_active();

    printf("Motor A ZERO sensor configured: pin=%d active_%s pullup=%d initial=%d\n",
           MOTOR_A_ZERO_SENSOR_PIN,
            MOTOR_A_ZERO_SENSOR_ACTIVE_LOW ? "low" : "high",
            MOTOR_A_ZERO_SENSOR_ENABLE_PULLUP ? 1 : 0,
           g_motor_a_zero_sensor_last_active ? 1 : 0);
        printf("Motor A ZERO raw level now: %d\n", read_motor_a_zero_sensor_raw());
}

void dwell_with_sensor_poll(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) {
        poll_motor_a_zero_sensor_edge();
        poll_zero_sensor_edge();
        sleep_ms(1);
    }
}

bool init_motor(byj_motor_t* motor, const uint pins[4], const char* name) {
    const byj_gpio_config_t cfg = {pins[0], pins[1], pins[2], pins[3]};
    if (!byj_init_gpio(motor, &cfg, STEP_MODE)) {
        printf("%s init failed\n", name);
        return false;
    }

    byj_set_output_mode(motor, OUTPUT_MODE);
    byj_set_speed(motor, STEP_DELAY_US);

    printf("%s init OK on pins %u %u %u %u\n", name, pins[0], pins[1], pins[2], pins[3]);
    return true;
}

void step_relative(byj_motor_t* motor, int32_t steps, bool invert_direction) {
    const int32_t commanded = invert_direction ? -steps : steps;
    byj_step_steps(motor, commanded);
}

void step_motor_a_single(byj_motor_t* motor_a, int direction) {
    const int32_t one_step = (direction >= 0) ? 1 : -1;
    step_relative(motor_a, one_step, INVERT_MOTOR_A_DIRECTION);
    if (read_motor_a_zero_sensor_active()) {
        g_motor_a_active_steps++;
    }
    poll_motor_a_zero_sensor_edge();
}

void step_motor_b_single(byj_motor_t* motor_b, int direction) {
    const int32_t one_step = (direction >= 0) ? 1 : -1;
    step_relative(motor_b, one_step, INVERT_MOTOR_B_DIRECTION);
    if (read_zero_sensor_active()) {
        g_motor_b_active_steps++;
    }
    poll_zero_sensor_edge();
}

bool clear_active_window(
    byj_motor_t* motor,
    sensor_active_fn sensor_active,
    step_single_fn step_single,
    int direction,
    int32_t max_steps,
    int32_t* steps_taken
) {
    return wait_for_sensor_state(
        motor,
        sensor_active,
        step_single,
        direction,
        false,
        max_steps,
        steps_taken
    );
}

bool seek_trigger(
    byj_motor_t* motor,
    sensor_active_fn sensor_active,
    step_single_fn step_single,
    int direction,
    int32_t max_steps,
    int32_t* steps_taken
) {
    return wait_for_sensor_state(
        motor,
        sensor_active,
        step_single,
        direction,
        true,
        max_steps,
        steps_taken
    );
}

bool measure_steps_trigger_to_trigger(
    byj_motor_t* motor,
    sensor_active_fn sensor_active,
    step_single_fn step_single,
    int direction,
    int32_t max_steps,
    int32_t* out_steps,
    int32_t* out_active_steps
) {
    const int32_t active_window_limit =
        (CALIBRATION_MAX_ACTIVE_WINDOW_STEPS < max_steps) ?
            CALIBRATION_MAX_ACTIVE_WINDOW_STEPS : max_steps;

    if (!sensor_active()) {
        int32_t reacquire = 0;
        if (!wait_for_sensor_state(motor, sensor_active, step_single, direction, true, max_steps, &reacquire)) {
            return false;
        }
    }

    int32_t clear_steps = 0;
    if (!wait_for_sensor_state(motor, sensor_active, step_single, direction, false, active_window_limit, &clear_steps)) {
        return false;
    }

    int32_t rotation_steps = 0;
    if (!wait_for_sensor_state(motor, sensor_active, step_single, direction, true, max_steps, &rotation_steps)) {
        return false;
    }

    int32_t active_steps = 0;
    if (!wait_for_sensor_state(motor, sensor_active, step_single, direction, false, active_window_limit, &active_steps)) {
        return false;
    }

    if (out_steps) {
        *out_steps = rotation_steps;
    }
    if (out_active_steps) {
        *out_active_steps = active_steps;
    }

    return true;
}

bool run_refined_zero_and_measure_both_directions(
    byj_motor_t* motor,
    const char* motor_name,
    int sensor_pin,
    sensor_active_fn sensor_active,
    sensor_raw_fn sensor_raw,
    step_single_fn step_single,
    int clockwise_direction,
    int counterclockwise_direction,
    int32_t* out_clockwise_steps,
    int32_t* out_counterclockwise_steps,
    int32_t* out_clockwise_active_steps,
    int32_t* out_counterclockwise_active_steps
) {
    uint32_t saved_delay = motor->step_delay_us;
    byj_set_speed(motor, CALIBRATION_COARSE_DELAY_US);

    int cw_dir = clockwise_direction;
    int ccw_dir = counterclockwise_direction;

    int32_t moved = 0;
    if (sensor_active()) {
        printf("%s sensor active at startup, clearing window first...\n", motor_name);
        if (!clear_active_window(motor, sensor_active, step_single, ccw_dir, CALIBRATION_MAX_ACTIVE_WINDOW_STEPS, &moved)) {
            printf("%s could not clear startup window using configured CCW direction; trying opposite...\n", motor_name);
            if (!clear_active_window(motor, sensor_active, step_single, -ccw_dir, CALIBRATION_MAX_ACTIVE_WINDOW_STEPS, &moved)) {
                printf("%s failed to clear active window at startup (raw=%d)\n", motor_name, sensor_raw());
                byj_set_speed(motor, saved_delay);
                return false;
            }
            ccw_dir = -ccw_dir;
            cw_dir = -cw_dir;
            printf("%s direction map flipped after startup clear (sensor GPIO%d)\n", motor_name, sensor_pin);
        }
    }

    int32_t coarse_steps = 0;
    if (!seek_trigger(motor, sensor_active, step_single, cw_dir, CALIBRATION_MAX_SEARCH_STEPS, &coarse_steps)) {
        printf("%s failed coarse clockwise search in configured direction; trying opposite...\n", motor_name);
        if (!seek_trigger(motor, sensor_active, step_single, -cw_dir, CALIBRATION_MAX_SEARCH_STEPS, &coarse_steps)) {
            printf("%s failed coarse clockwise trigger search (raw=%d)\n", motor_name, sensor_raw());
            byj_set_speed(motor, saved_delay);
            return false;
        }
        cw_dir = -cw_dir;
        ccw_dir = -ccw_dir;
        printf("%s direction map flipped after coarse seek (sensor GPIO%d)\n", motor_name, sensor_pin);
    }
    printf("%s coarse zero trigger found after %ld steps (clockwise)\n", motor_name, (long)coarse_steps);

    int32_t clear_steps = 0;
    if (!clear_active_window(motor, sensor_active, step_single, ccw_dir, CALIBRATION_MAX_ACTIVE_WINDOW_STEPS, &clear_steps)) {
        printf("%s failed to clear sensor before refinement using configured CCW; trying opposite...\n", motor_name);
        if (!clear_active_window(motor, sensor_active, step_single, -ccw_dir, CALIBRATION_MAX_ACTIVE_WINDOW_STEPS, &clear_steps)) {
            printf("%s failed to clear sensor before refinement (raw=%d)\n", motor_name, sensor_raw());
            byj_set_speed(motor, saved_delay);
            return false;
        }
        ccw_dir = -ccw_dir;
        cw_dir = -cw_dir;
        printf("%s direction map flipped after refinement-clear (sensor GPIO%d)\n", motor_name, sensor_pin);
    }

    for (int32_t i = 0; i < SENSOR_REFINEMENT_EXTRA_BACKOFF_STEPS; ++i) {
        step_single(motor, ccw_dir);
    }

    byj_set_speed(motor, REFINED_APPROACH_DELAY_US);
    int32_t refined_steps = 0;
    if (!seek_trigger(motor, sensor_active, step_single, cw_dir, CALIBRATION_MAX_SEARCH_STEPS, &refined_steps)) {
        printf("%s failed refined zero approach in configured CW direction; trying opposite...\n", motor_name);
        if (!seek_trigger(motor, sensor_active, step_single, -cw_dir, CALIBRATION_MAX_SEARCH_STEPS, &refined_steps)) {
            printf("%s failed refined zero approach (raw=%d)\n", motor_name, sensor_raw());
            byj_set_speed(motor, saved_delay);
            return false;
        }
        cw_dir = -cw_dir;
        ccw_dir = -ccw_dir;
        printf("%s direction map flipped after refined seek (sensor GPIO%d)\n", motor_name, sensor_pin);
    }
    printf("%s refined zero established after %ld slow approach steps\n", motor_name, (long)refined_steps);

    byj_set_speed(motor, CALIBRATION_MEASURE_DELAY_US);
    int32_t clockwise_steps = 0;
    int32_t clockwise_active_steps = 0;
    if (!measure_steps_trigger_to_trigger(motor, sensor_active, step_single, cw_dir, CALIBRATION_MAX_SEARCH_STEPS, &clockwise_steps, &clockwise_active_steps)) {
        printf("%s failed clockwise trigger-to-trigger measurement (raw=%d)\n", motor_name, sensor_raw());
        byj_set_speed(motor, saved_delay);
        return false;
    }
    printf("%s clockwise rotation steps=%ld, switch-active steps=%ld (normal)\n",
           motor_name,
           (long)clockwise_steps,
           (long)clockwise_active_steps);

    int32_t clockwise_precise = clockwise_steps;
    int32_t clockwise_active_precise = clockwise_active_steps;
    if (RUN_PRECISION_SECOND_PASS) {
        byj_set_speed(motor, PRECISION_MEASURE_DELAY_US);
        if (measure_steps_trigger_to_trigger(motor, sensor_active, step_single, cw_dir, CALIBRATION_MAX_SEARCH_STEPS, &clockwise_precise, &clockwise_active_precise)) {
            printf("%s clockwise rotation steps=%ld, switch-active steps=%ld (precision)\n",
                   motor_name,
                   (long)clockwise_precise,
                   (long)clockwise_active_precise);
        } else {
            printf("%s precision clockwise pass failed, keeping normal result\n", motor_name);
            clockwise_precise = clockwise_steps;
            clockwise_active_precise = clockwise_active_steps;
        }
    }

    byj_set_speed(motor, CALIBRATION_MEASURE_DELAY_US);
    int32_t counterclockwise_steps = 0;
    int32_t counterclockwise_active_steps = 0;
    if (!measure_steps_trigger_to_trigger(motor, sensor_active, step_single, ccw_dir, CALIBRATION_MAX_SEARCH_STEPS, &counterclockwise_steps, &counterclockwise_active_steps)) {
        printf("%s failed counterclockwise trigger-to-trigger measurement (raw=%d)\n", motor_name, sensor_raw());
        byj_set_speed(motor, saved_delay);
        return false;
    }
    printf("%s counterclockwise rotation steps=%ld, switch-active steps=%ld (normal)\n",
           motor_name,
           (long)counterclockwise_steps,
           (long)counterclockwise_active_steps);

    int32_t counterclockwise_precise = counterclockwise_steps;
    int32_t counterclockwise_active_precise = counterclockwise_active_steps;
    if (RUN_PRECISION_SECOND_PASS) {
        byj_set_speed(motor, PRECISION_MEASURE_DELAY_US);
        if (measure_steps_trigger_to_trigger(motor, sensor_active, step_single, ccw_dir, CALIBRATION_MAX_SEARCH_STEPS, &counterclockwise_precise, &counterclockwise_active_precise)) {
            printf("%s counterclockwise rotation steps=%ld, switch-active steps=%ld (precision)\n",
                   motor_name,
                   (long)counterclockwise_precise,
                   (long)counterclockwise_active_precise);
        } else {
            printf("%s precision counterclockwise pass failed, keeping normal result\n", motor_name);
            counterclockwise_precise = counterclockwise_steps;
            counterclockwise_active_precise = counterclockwise_active_steps;
        }
    }

    byj_set_speed(motor, saved_delay);

    if (out_clockwise_steps) {
        *out_clockwise_steps = RUN_PRECISION_SECOND_PASS ? clockwise_precise : clockwise_steps;
    }
    if (out_counterclockwise_steps) {
        if (RUN_PRECISION_SECOND_PASS) {
            *out_counterclockwise_steps = counterclockwise_precise;
        } else {
            *out_counterclockwise_steps = counterclockwise_steps;
        }
    }
    if (out_clockwise_active_steps) {
        *out_clockwise_active_steps = RUN_PRECISION_SECOND_PASS ? clockwise_active_precise : clockwise_active_steps;
    }
    if (out_counterclockwise_active_steps) {
        *out_counterclockwise_active_steps = RUN_PRECISION_SECOND_PASS ? counterclockwise_active_precise : counterclockwise_active_steps;
    }

    return true;
}

void move_motor_a_counterclockwise_to_boostpump_zero(byj_motor_t* motor_a, int32_t steps_from_zero) {
    if (steps_from_zero <= 0) {
        return;
    }
    printf("Parking Motor A at BOOSTPUMP_ZERO_L=%ld (counterclockwise)\n", (long)steps_from_zero);
    for (int32_t i = 0; i < steps_from_zero; ++i) {
        step_motor_a_single(motor_a, MOTOR_A_COUNTERCLOCKWISE_DIRECTION);
    }
}

void move_motor_b_counterclockwise_to_boostpump_zero(byj_motor_t* motor_b, int32_t steps_from_zero) {
    if (steps_from_zero <= 0) {
        return;
    }
    printf("Parking Motor B at BOOSTPUMP_ZERO_R=%ld (counterclockwise)\n", (long)steps_from_zero);
    for (int32_t i = 0; i < steps_from_zero; ++i) {
        step_motor_b_single(motor_b, MOTOR_B_COUNTERCLOCKWISE_DIRECTION);
    }
}

bool calibrate_motor_a_steps_per_rev(byj_motor_t* motor_a) {
    if (!g_motor_a_zero_sensor_ready) {
        printf("Calibration skipped: Motor A ZERO sensor not configured\n");
        return false;
    }

    int32_t clockwise_steps = 0;
    int32_t counterclockwise_steps = 0;
    int32_t clockwise_active_steps = 0;
    int32_t counterclockwise_active_steps = 0;
    const bool ok = run_refined_zero_and_measure_both_directions(
        motor_a,
        "Motor A",
        MOTOR_A_ZERO_SENSOR_PIN,
        read_motor_a_zero_sensor_active,
        read_motor_a_zero_sensor_raw,
        step_motor_a_single,
        -MOTOR_A_COUNTERCLOCKWISE_DIRECTION,
        MOTOR_A_COUNTERCLOCKWISE_DIRECTION,
        &clockwise_steps,
        &counterclockwise_steps,
        &clockwise_active_steps,
        &counterclockwise_active_steps
    );
    if (!ok) {
        return false;
    }

    g_motor_a_last_cw_steps = clockwise_steps;
    g_motor_a_last_ccw_steps = counterclockwise_steps;
    g_motor_a_last_cw_active_steps = clockwise_active_steps;
    g_motor_a_last_ccw_active_steps = counterclockwise_active_steps;

    const int32_t avg_steps = (clockwise_steps + counterclockwise_steps + 1) / 2;
        printf("Motor A effective sweep: CW=%ld (active=%ld) CCW=%ld (active=%ld) AVG=%ld\n",
           (long)clockwise_steps,
            (long)clockwise_active_steps,
           (long)counterclockwise_steps,
            (long)counterclockwise_active_steps,
           (long)avg_steps);
    printf("Configured BOOSTPUMP_ZERO_L=%ld (guard=%ld)\n", (long)BOOSTPUMP_ZERO_L, (long)BOOSTPUMP_SWITCH_GUARD_STEPS);
    move_motor_a_counterclockwise_to_boostpump_zero(motor_a, BOOSTPUMP_ZERO_L);
    printf("Apply this value to BYJ48_STEPS_PER_REV or your local sweep constant\n");
    return true;
}

bool calibrate_motor_b_steps_per_rev(byj_motor_t* motor_b) {
    if (!g_zero_sensor_ready) {
        printf("Calibration skipped: Motor B ZERO sensor not configured\n");
        return false;
    }

    int32_t clockwise_steps = 0;
    int32_t counterclockwise_steps = 0;
    int32_t clockwise_active_steps = 0;
    int32_t counterclockwise_active_steps = 0;
    const bool ok = run_refined_zero_and_measure_both_directions(
        motor_b,
        "Motor B",
        MOTOR_B_ZERO_SENSOR_PIN,
        read_zero_sensor_active,
        read_motor_b_zero_sensor_raw,
        step_motor_b_single,
        -MOTOR_B_COUNTERCLOCKWISE_DIRECTION,
        MOTOR_B_COUNTERCLOCKWISE_DIRECTION,
        &clockwise_steps,
        &counterclockwise_steps,
        &clockwise_active_steps,
        &counterclockwise_active_steps
    );
    if (!ok) {
        return false;
    }

    g_motor_b_last_cw_steps = clockwise_steps;
    g_motor_b_last_ccw_steps = counterclockwise_steps;
    g_motor_b_last_cw_active_steps = clockwise_active_steps;
    g_motor_b_last_ccw_active_steps = counterclockwise_active_steps;

    const int32_t avg_steps = (clockwise_steps + counterclockwise_steps + 1) / 2;
        printf("Motor B effective sweep: CW=%ld (active=%ld) CCW=%ld (active=%ld) AVG=%ld\n",
           (long)clockwise_steps,
            (long)clockwise_active_steps,
           (long)counterclockwise_steps,
            (long)counterclockwise_active_steps,
           (long)avg_steps);
    printf("Configured BOOSTPUMP_ZERO_R=%ld (guard=%ld)\n", (long)BOOSTPUMP_ZERO_R, (long)BOOSTPUMP_SWITCH_GUARD_STEPS);
    move_motor_b_counterclockwise_to_boostpump_zero(motor_b, BOOSTPUMP_ZERO_R);
    printf("Apply this value to BYJ48_STEPS_PER_REV or your local sweep constant\n");
    return true;
}

void poll_runtime_commands(byj_motor_t* motor_a, byj_motor_t* motor_b) {
    const int ch = getchar_timeout_us(0);
    if (ch == PICO_ERROR_TIMEOUT) {
        return;
    }

    if (ch == CALIBRATION_COMMAND_A || ch == 'A') {
        printf("Command '%c': running Motor A steps/rev calibration\n", CALIBRATION_COMMAND_A);
        calibrate_motor_a_steps_per_rev(motor_a);
    } else if (ch == CALIBRATION_COMMAND || ch == 'C') {
        printf("Command '%c': running Motor B steps/rev calibration\n", CALIBRATION_COMMAND);
        calibrate_motor_b_steps_per_rev(motor_b);
    }
}

void step_both_interleaved(
    byj_motor_t* motor_a,
    byj_motor_t* motor_b,
    int32_t steps_a,
    int32_t steps_b,
    bool invert_a,
    bool invert_b
) {
    int32_t remaining_a = (steps_a >= 0) ? steps_a : -steps_a;
    int32_t remaining_b = (steps_b >= 0) ? steps_b : -steps_b;
    const int dir_a = (steps_a >= 0) ? 1 : -1;
    const int dir_b = (steps_b >= 0) ? 1 : -1;

    while ((remaining_a > 0) || (remaining_b > 0)) {
        poll_motor_a_zero_sensor_edge();
        poll_zero_sensor_edge();

        if (remaining_a > 0) {
            int32_t chunk_a = (remaining_a > INTERLEAVE_CHUNK_STEPS) ? INTERLEAVE_CHUNK_STEPS : remaining_a;
            step_relative(motor_a, dir_a * chunk_a, invert_a);
            remaining_a -= chunk_a;
            poll_motor_a_zero_sensor_edge();
            poll_zero_sensor_edge();
        }

        if (remaining_b > 0) {
            int32_t chunk_b = (remaining_b > INTERLEAVE_CHUNK_STEPS) ? INTERLEAVE_CHUNK_STEPS : remaining_b;
            step_relative(motor_b, dir_b * chunk_b, invert_b);
            remaining_b -= chunk_b;
            poll_zero_sensor_edge();
        }
    }
}

void set_status_led(bool on) {
    gpio_put(PICO_DEFAULT_LED_PIN, on ? 1 : 0);
    pwm_set_gpio_level(BACKLIGHT_PWM_PIN, on ? 255 : 0);
}

void print_cycle_summary(uint32_t cycle_index, const byj_motor_t* motor_a, const byj_motor_t* motor_b) {
    printf("Cycle %lu summary | A(pos=%ld cw=%ld cwAct=%ld ccw=%ld ccwAct=%ld lastWin=%ld) | B(pos=%ld cw=%ld cwAct=%ld ccw=%ld ccwAct=%ld lastWin=%ld)\n",
           (unsigned long)cycle_index,
           (long)motor_a->current_position,
           (long)g_motor_a_last_cw_steps,
           (long)g_motor_a_last_cw_active_steps,
           (long)g_motor_a_last_ccw_steps,
           (long)g_motor_a_last_ccw_active_steps,
           (long)g_motor_a_last_active_window_steps,
           (long)motor_b->current_position,
           (long)g_motor_b_last_cw_steps,
           (long)g_motor_b_last_cw_active_steps,
           (long)g_motor_b_last_ccw_steps,
           (long)g_motor_b_last_ccw_active_steps,
           (long)g_motor_b_last_active_window_steps);
}

}  // namespace

int main() {
    stdio_init_all();
    sleep_ms(2000);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    gpio_set_function(BACKLIGHT_PWM_PIN, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(BACKLIGHT_PWM_PIN);
    pwm_set_wrap(slice, 255);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_enabled(slice, true);
    pwm_set_gpio_level(BACKLIGHT_PWM_PIN, 0);

    byj_motor_t motor_a;
    byj_motor_t motor_b;

    printf("Dual 28BYJ-48 DRV8833 test startup\n");
    printf("Mode=%d output=%d step_delay=%luus sweep=%ld steps\n",
           (int)STEP_MODE,
           (int)OUTPUT_MODE,
           (unsigned long)STEP_DELAY_US,
           (long)SWEEP_STEPS);

    if (!init_motor(&motor_a, MOTOR_A_PINS, "Motor A")) {
        return 1;
    }
    if (!init_motor(&motor_b, MOTOR_B_PINS, "Motor B")) {
        return 1;
    }

    init_zero_sensor();
    init_motor_a_zero_sensor();

    printf("Direction invert: A=%d B=%d\n",
           INVERT_MOTOR_A_DIRECTION ? 1 : 0,
           INVERT_MOTOR_B_DIRECTION ? 1 : 0);
    printf("Runtime commands: '%c' for Motor A calibration, '%c' for Motor B calibration\n",
           CALIBRATION_COMMAND_A,
           CALIBRATION_COMMAND);

    if (RUN_STEPS_PER_REV_CALIBRATION_ON_BOOT) {
        calibrate_motor_a_steps_per_rev(&motor_a);
        calibrate_motor_b_steps_per_rev(&motor_b);
    }

    printf("Starting continuous interleaved sweep\n");

    uint32_t cycle_index = 0;

    while (true) {
        poll_runtime_commands(&motor_a, &motor_b);

        printf("Forward sweep\n");
        set_status_led(true);
        step_both_interleaved(
            &motor_a,
            &motor_b,
            SWEEP_STEPS,
            SWEEP_STEPS,
            INVERT_MOTOR_A_DIRECTION,
            INVERT_MOTOR_B_DIRECTION
        );
        printf("Positions after forward: A=%ld B=%ld\n",
               (long)motor_a.current_position,
               (long)motor_b.current_position);
         dwell_with_sensor_poll(ENDPOINT_DWELL_MS);
        poll_runtime_commands(&motor_a, &motor_b);

        printf("Reverse sweep\n");
        set_status_led(false);
        step_both_interleaved(
            &motor_a,
            &motor_b,
            -SWEEP_STEPS,
            -SWEEP_STEPS,
            INVERT_MOTOR_A_DIRECTION,
            INVERT_MOTOR_B_DIRECTION
        );
        printf("Positions after reverse: A=%ld B=%ld\n",
               (long)motor_a.current_position,
               (long)motor_b.current_position);
         dwell_with_sensor_poll(ENDPOINT_DWELL_MS);
            poll_runtime_commands(&motor_a, &motor_b);

        cycle_index++;
        print_cycle_summary(cycle_index, &motor_a, &motor_b);
    }

    return 0;
}
