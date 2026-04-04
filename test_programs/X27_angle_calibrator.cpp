#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "internal/X27_stepper.h"

namespace {

constexpr uint VVI_COIL_1A_PIN = 6;
constexpr uint VVI_COIL_1B_PIN = 7;
constexpr uint VVI_COIL_2A_PIN = 8;
constexpr uint VVI_COIL_2B_PIN = 9;
constexpr uint VVI_HOME_SENSOR_PIN = 10;

constexpr uint AOA_COIL_1A_PIN = 11;
constexpr uint AOA_COIL_1B_PIN = 12;
constexpr uint AOA_COIL_2A_PIN = 13;
constexpr uint AOA_COIL_2B_PIN = 14;
constexpr uint AOA_HOME_SENSOR_PIN = 15;

constexpr int MOTOR_COUNT = 2;
constexpr int MOTOR_VVI = 0;
constexpr int MOTOR_AOA = 1;

constexpr bool MOTOR_DIRECTION_INVERTED[MOTOR_COUNT] = {
    true,
    false,
};

const char* motor_name(int index) {
    return (index == MOTOR_VVI) ? "VVI" : "AoA";
}

uint motor_home_pin(int index) {
    return (index == MOTOR_VVI) ? VVI_HOME_SENSOR_PIN : AOA_HOME_SENSOR_PIN;
}

constexpr int32_t HOME_SWEEP_STEPS = 2200;
constexpr int32_t DEFAULT_HOME_LOGICAL_POSITION = 100;
constexpr int32_t DEFAULT_NUDGE_STEPS = 10;
constexpr float DEFAULT_NUDGE_ANGLE = 1.0f;
constexpr uint32_t DEFAULT_STEP_DELAY_US = 1000;
constexpr uint32_t DEFAULT_HOME_STEP_DELAY_US = 2800;
constexpr int HOME_SENSOR_DEBOUNCE_SAMPLES = 5;
constexpr uint32_t HOME_SENSOR_DEBOUNCE_US = 500;
constexpr uint32_t HOME_SENSOR_SETTLE_MS = 5;
constexpr int HOME_SEARCH_RETRIES = 2;

struct MotionTracker {
    int32_t last_position = 0;
    int32_t reference_position = 0;
    int64_t signed_steps_since_boot = 0;
    uint64_t absolute_steps_since_boot = 0;
};

const char* mode_to_text(x27_step_mode_t mode) {
    switch (mode) {
        case X27_MODE_FULL_STEP: return "full";
        case X27_MODE_HALF_STEP: return "half";
        case X27_MODE_MICRO_STEP: return "micro";
        default: return "unknown";
    }
}

bool parse_step_mode(char token, x27_step_mode_t& mode) {
    switch ((char)tolower((unsigned char)token)) {
        case 'f':
            mode = X27_MODE_FULL_STEP;
            return true;
        case 'h':
            mode = X27_MODE_HALF_STEP;
            return true;
        case 'm':
            mode = X27_MODE_MICRO_STEP;
            return true;
        default:
            return false;
    }
}

bool is_home_sensor_active_stable(const x27_motor_t& motor) {
    if (!motor.homing_configured || motor.homing_pin < 0) {
        return false;
    }

    for (int sample = 0; sample < HOME_SENSOR_DEBOUNCE_SAMPLES; ++sample) {
        int value = gpio_get((uint)motor.homing_pin);
        bool active = motor.homing_active_high ? (value != 0) : (value == 0);
        if (!active) {
            return false;
        }
        busy_wait_us(HOME_SENSOR_DEBOUNCE_US);
    }
    return true;
}

bool home_with_sensor_validated(x27_motor_t& motor, int8_t dir, uint32_t max_steps) {
    if (!x27_home_with_sensor(&motor, dir, max_steps)) {
        return false;
    }
    sleep_ms(HOME_SENSOR_SETTLE_MS);
    return is_home_sensor_active_stable(motor);
}

void print_status(const char* active_name, const x27_motor_t& motor, const MotionTracker& tracker) {
    int32_t pos = x27_get_position(&motor);
    float angle = x27_get_angle(&motor);
    int32_t delta_steps = pos - tracker.reference_position;
    float delta_angle = (float)delta_steps * x27_get_effective_step_angle(&motor);
    printf("STATUS [%s] pos=%ld/%d angle=%.2f deg target=%ld speed=%lu us invert=%s\n",
           active_name,
           (long)pos,
           X27_MAX_POSITION,
           angle,
           (long)motor.target_position,
           (unsigned long)motor.step_delay_us,
           motor.direction_inverted ? "on" : "off");
        printf("       mode=%s step=%.4f deg eff_steps_per_rev=%lu\n",
            mode_to_text(motor.step_mode),
            x27_get_effective_step_angle(&motor),
            (unsigned long)x27_get_effective_steps_per_rev(&motor));
    printf("       ref=%ld delta=%+ld steps (%+.2f deg) moved(abs=%llu signed=%+lld)\n",
           (long)tracker.reference_position,
           (long)delta_steps,
           delta_angle,
           (unsigned long long)tracker.absolute_steps_since_boot,
           (long long)tracker.signed_steps_since_boot);
}

void print_help() {
    printf("\n=== X27 Angle Calibrator ===\n");
    printf("VVI Pins: C1A=%u C1B=%u C2A=%u C2B=%u HOME=%u\n",
           VVI_COIL_1A_PIN,
           VVI_COIL_1B_PIN,
           VVI_COIL_2A_PIN,
           VVI_COIL_2B_PIN,
           VVI_HOME_SENSOR_PIN);
    printf("AoA Pins: C1A=%u C1B=%u C2A=%u C2B=%u HOME=%u\n",
           AOA_COIL_1A_PIN,
           AOA_COIL_1B_PIN,
           AOA_COIL_2A_PIN,
           AOA_COIL_2B_PIN,
           AOA_HOME_SENSOR_PIN);
    printf("Commands (enter then press Return):\n");
    printf("  g <v|a>          switch active motor (v=VVI, a=AoA)\n");
    printf("  h                help\n");
    printf("  r                report current position and angle\n");
    printf("  m                mark current position as reference\n");
    printf("  d                report delta from reference\n");
    printf("  c                clear movement counters\n");
    printf("  0                re-zero current logical position (no movement)\n");
    printf("  q <steps>        set current logical position (re-zero without moving, requires Return)\n");
    printf("  a <deg>          move to angle (0..360)\n");
    printf("  p <steps>        move to position (0..%d)\n", X27_MAX_POSITION);
    printf("  + [steps]        nudge forward (default %ld)\n", (long)DEFAULT_NUDGE_STEPS);
    printf("  - [steps]        nudge backward (default %ld)\n", (long)DEFAULT_NUDGE_STEPS);
    printf("  ]                nudge forward exactly 1 step\n");
    printf("  [                nudge backward exactly 1 step\n");
    printf("  > [deg]          nudge angle forward (default %.1f)\n", DEFAULT_NUDGE_ANGLE);
    printf("  < [deg]          nudge angle backward (default %.1f)\n", DEFAULT_NUDGE_ANGLE);
    printf("  s <us>           set step delay in microseconds (min %d)\n", X27_MIN_STEP_US);
    printf("  k <f|h|m>        set step mode: full/half/micro (requires Return)\n");
    printf("  i                toggle direction inversion\n");
    printf("  z                home to mechanical stop (CCW sweep)\n");
    printf("  t                quick movement self-test (+60 then -60 steps)\n");
    printf("  x                de-energize coils (sleep)\n");
    printf("  (single-key commands above also work without pressing Return)\n");
    printf("\nTip: adjust angle until pointer matches the mark, then read 'r'.\n\n");
}

char* skip_spaces(char* text) {
    while (*text && isspace((unsigned char)*text)) {
        ++text;
    }
    return text;
}

void process_command(x27_motor_t motors[MOTOR_COUNT], MotionTracker trackers[MOTOR_COUNT], int& active_motor, char* line) {
    char* cmd = skip_spaces(line);
    if (*cmd == '\0') {
        return;
    }

    x27_motor_t& motor = motors[active_motor];
    MotionTracker& tracker = trackers[active_motor];

    if (*cmd == 'g') {
        char* args = skip_spaces(cmd + 1);
        if (*args == '\0') {
            printf("ACTIVE motor=%s\n", motor_name(active_motor));
            return;
        }

        char token = (char)tolower((unsigned char)*args);
        if (token == 'v') {
            active_motor = MOTOR_VVI;
        } else if (token == 'a') {
            active_motor = MOTOR_AOA;
        } else {
            printf("ERR invalid motor; use g v or g a\n");
            return;
        }

        x27_motor_t& new_motor = motors[active_motor];
        MotionTracker& new_tracker = trackers[active_motor];
        printf("ACTIVE switched to %s\n", motor_name(active_motor));
        print_status(motor_name(active_motor), new_motor, new_tracker);
        return;
    }

    if (*cmd == 'h' || *cmd == '?') {
        print_help();
        return;
    }

    if (*cmd == 'r') {
        print_status(motor_name(active_motor), motor, tracker);
        return;
    }

    if (*cmd == 'm') {
        tracker.reference_position = x27_get_position(&motor);
        printf("REFERENCE set to %ld\n", (long)tracker.reference_position);
        return;
    }

    if (*cmd == 'd') {
        int32_t pos = x27_get_position(&motor);
        int32_t delta_steps = pos - tracker.reference_position;
        float delta_angle = (float)delta_steps * x27_get_effective_step_angle(&motor);
        printf("DELTA %ld steps (%+.2f deg) from reference\n", (long)delta_steps, delta_angle);
        return;
    }

    if (*cmd == 'c') {
        tracker.signed_steps_since_boot = 0;
        tracker.absolute_steps_since_boot = 0;
        printf("COUNTERS cleared\n");
        return;
    }

    if (*cmd == '0') {
        motor.current_position = 0;
        motor.target_position = 0;
        tracker.reference_position = 0;
        tracker.last_position = 0;
        printf("REZERO logical position set to 0 (no physical movement)\n");
        return;
    }

    if (*cmd == 'q') {
        char* args = skip_spaces(cmd + 1);
        long new_pos = strtol(args, nullptr, 10);
        if (new_pos < 0) new_pos = 0;
        if (new_pos > X27_MAX_POSITION) new_pos = X27_MAX_POSITION;
        motor.current_position = (int32_t)new_pos;
        motor.target_position = (int32_t)new_pos;
        tracker.reference_position = (int32_t)new_pos;
        tracker.last_position = (int32_t)new_pos;
        printf("LOGICAL position set to %ld (no physical movement)\n", new_pos);
        return;
    }

    if (*cmd == 'a') {
        char* args = skip_spaces(cmd + 1);
        float angle = strtof(args, nullptr);
        x27_set_angle(&motor, angle);
        printf("MOVE angle=%.2f deg\n", angle);
        return;
    }

    if (*cmd == 'p') {
        char* args = skip_spaces(cmd + 1);
        long pos = strtol(args, nullptr, 10);
        x27_set_position(&motor, (int32_t)pos);
        printf("MOVE pos=%ld\n", pos);
        return;
    }

    if (*cmd == '+' || *cmd == '-') {
        char* args = skip_spaces(cmd + 1);
        long amount = (*args == '\0') ? DEFAULT_NUDGE_STEPS : strtol(args, nullptr, 10);
        if (amount < 0) amount = -amount;
        if (*cmd == '-') amount = -amount;
        int32_t target = x27_get_position(&motor) + (int32_t)amount;
        x27_set_position(&motor, target);
        printf("NUDGE steps=%ld\n", amount);
        return;
    }

    if (*cmd == ']' || *cmd == '[') {
        int32_t amount = (*cmd == ']') ? 1 : -1;
        int32_t target = x27_get_position(&motor) + amount;
        x27_set_position(&motor, target);
        printf("NUDGE step=%+ld\n", (long)amount);
        return;
    }

    if (*cmd == '>' || *cmd == '<') {
        char* args = skip_spaces(cmd + 1);
        float amount = (*args == '\0') ? DEFAULT_NUDGE_ANGLE : strtof(args, nullptr);
        if (amount < 0.0f) amount = -amount;
        if (*cmd == '<') amount = -amount;
        float current_angle = x27_get_angle(&motor);
        x27_set_angle(&motor, current_angle + amount);
        printf("NUDGE angle=%.2f deg\n", amount);
        return;
    }

    if (*cmd == 's') {
        char* args = skip_spaces(cmd + 1);
        long speed_us = strtol(args, nullptr, 10);
        if (speed_us <= 0) {
            printf("ERR invalid speed\n");
            return;
        }
        x27_set_speed(&motor, (uint32_t)speed_us);
        printf("SPEED %lu us\n", (unsigned long)motor.step_delay_us);
        return;
    }

    if (*cmd == 'k') {
        char* args = skip_spaces(cmd + 1);
        if (*args == '\0') {
            printf("MODE %s (use: k f|h|m)\n", mode_to_text(motor.step_mode));
            return;
        }
        x27_step_mode_t new_mode;
        if (!parse_step_mode(*args, new_mode)) {
            printf("ERR invalid mode; use f|h|m\n");
            return;
        }
        motor.step_mode = new_mode;
        printf("MODE set to %s (step=%.4f deg eff_steps_per_rev=%lu)\n",
               mode_to_text(motor.step_mode),
               x27_get_effective_step_angle(&motor),
               (unsigned long)x27_get_effective_steps_per_rev(&motor));
        return;
    }

    if (*cmd == 'i') {
        x27_set_direction_inverted(&motor, !motor.direction_inverted);
        printf("INVERT %s\n", motor.direction_inverted ? "on" : "off");
        return;
    }

    if (*cmd == 'z') {
        printf("HOMING %s...\n", motor_name(active_motor));
        bool sensor_ok = x27_config_homing_sensor(&motor, (int)motor_home_pin(active_motor), false, true);
        bool homed = false;
        x27_set_speed(&motor, DEFAULT_HOME_STEP_DELAY_US);
        if (sensor_ok) {
            sleep_ms(HOME_SENSOR_SETTLE_MS);

            if (is_home_sensor_active_stable(motor)) {
                homed = true;
                printf("HOME sensor already active before sweep\n");
            } else {
                for (int attempt = 0; attempt < HOME_SEARCH_RETRIES && !homed; ++attempt) {
                    homed = home_with_sensor_validated(motor, -1, HOME_SWEEP_STEPS);
                    if (!homed) {
                        homed = home_with_sensor_validated(motor, +1, HOME_SWEEP_STEPS);
                    }
                }
            }

            if (!homed) {
                printf("HOME sensor not detected reliably; using stop fallback\n");
            }
        }
        if (!homed) {
            x27_home_to_stop(&motor, -1, HOME_SWEEP_STEPS);
        }
        x27_set_speed(&motor, DEFAULT_STEP_DELAY_US);
        // Keep physical shaft at the sensor/home point, but assign a logical offset
        // so motion is available on both sides of home despite 0..MAX clamping.
        motor.current_position = DEFAULT_HOME_LOGICAL_POSITION;
        motor.target_position = DEFAULT_HOME_LOGICAL_POSITION;
        tracker.reference_position = DEFAULT_HOME_LOGICAL_POSITION;
        tracker.last_position = DEFAULT_HOME_LOGICAL_POSITION;
         motor.homing_configured = false;
         motor.homing_pin = -1;
        printf("HOME complete (sensor=%d homed=%d logical_home=%ld)\n",
               sensor_ok ? 1 : 0,
               homed ? 1 : 0,
               (long)DEFAULT_HOME_LOGICAL_POSITION);
         printf("HOME sensor disabled for commanded travel (open-loop step tracking)\n");
        return;
    }

    if (*cmd == 'x') {
        x27_sleep(&motor);
        printf("SLEEP\n");
        return;
    }

    if (*cmd == 't') {
        int32_t start = x27_get_position(&motor);
        x27_set_position(&motor, start + 60);
        printf("TEST forward\n");
        return;
    }

    printf("ERR unknown command: %s\n", cmd);
}

} // namespace

int main() {
    stdio_init_all();
    sleep_ms(1500);

    x27_motor_t motors[MOTOR_COUNT] = {};
    MotionTracker trackers[MOTOR_COUNT] = {};
    x27_gpio_config_t configs[MOTOR_COUNT] = {
        {VVI_COIL_1A_PIN, VVI_COIL_1B_PIN, VVI_COIL_2A_PIN, VVI_COIL_2B_PIN},
        {AOA_COIL_1A_PIN, AOA_COIL_1B_PIN, AOA_COIL_2A_PIN, AOA_COIL_2B_PIN},
    };

    for (int index = 0; index < MOTOR_COUNT; ++index) {
        if (!x27_init_gpio(&motors[index], &configs[index], X27_MODE_FULL_STEP)) {
            printf("ERROR: failed to init %s motor\n", motor_name(index));
            while (true) {
                sleep_ms(1000);
            }
        }

        x27_set_direction_inverted(&motors[index], MOTOR_DIRECTION_INVERTED[index]);
        x27_set_speed(&motors[index], DEFAULT_STEP_DELAY_US);
        trackers[index].last_position = x27_get_position(&motors[index]);
        trackers[index].reference_position = trackers[index].last_position;
    }

    int active_motor = MOTOR_AOA;
    printf("X27 calibrator booted (VVI + AoA). Active=%s. Help menu will print in 3 seconds...\n", motor_name(active_motor));

    char line[80] = {0};
    size_t len = 0;
    absolute_time_t boot_time = get_absolute_time();
    bool boot_help_printed = false;

    while (true) {
        for (int index = 0; index < MOTOR_COUNT; ++index) {
            x27_update(&motors[index]);
            int32_t now_pos = x27_get_position(&motors[index]);
            int32_t delta = now_pos - trackers[index].last_position;
            if (delta != 0) {
                trackers[index].signed_steps_since_boot += delta;
                trackers[index].absolute_steps_since_boot += (uint64_t)(delta > 0 ? delta : -delta);
                trackers[index].last_position = now_pos;
            }
        }

        if (!boot_help_printed && absolute_time_diff_us(boot_time, get_absolute_time()) >= 3000000) {
            print_help();
            print_status(motor_name(active_motor), motors[active_motor], trackers[active_motor]);
            boot_help_printed = true;
        }

        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            sleep_ms(1);
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (len > 0) {
                line[len] = '\0';
                process_command(motors, trackers, active_motor, line);
                if (line[0] == 't') {
                    x27_wait_complete(&motors[active_motor]);
                    int32_t start = x27_get_position(&motors[active_motor]);
                    x27_set_position(&motors[active_motor], start - 60);
                    printf("TEST reverse\n");
                }
                len = 0;
            }
            continue;
        }

        if (len == 0 && (ch == 'h' || ch == '?' || ch == 'r' || ch == 'm' || ch == 'd' || ch == 'c' || ch == '0' || ch == 'i' || ch == 'z' || ch == 'x' || ch == 't' || ch == '+' || ch == '-' || ch == '>' || ch == '<' || ch == '[' || ch == ']')) {
            line[0] = (char)ch;
            line[1] = '\0';
            process_command(motors, trackers, active_motor, line);
            if (ch == 't') {
                x27_wait_complete(&motors[active_motor]);
                int32_t start = x27_get_position(&motors[active_motor]);
                x27_set_position(&motors[active_motor], start - 60);
                printf("TEST reverse\n");
            }
            len = 0;
            continue;
        }

        if (len == 0 && (ch == 'g' || ch == 'G')) {
            line[len++] = (char)ch;
            continue;
        }

        if (isprint((unsigned char)ch) && len < sizeof(line) - 1) {
            line[len++] = (char)ch;
        }
    }
}
