#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#define X27_VID6606_DIR_SETUP_US 20
#define X27_VID6606_STEP_HIGH_US 20
#define X27_VID6606_STEP_LOW_US 20
#define X27_VID6606_RESET_RECOVERY_US 50

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"
#include "internal/X27_stepper.h"
#include "internal/ws2812.h"

namespace {

constexpr uint SHARED_RESET_PIN = 26;
constexpr bool RESET_ACTIVE_LOW = true;
constexpr uint8_t RESET_ENABLE_LEVEL = RESET_ACTIVE_LOW ? 1 : 0;
constexpr int MODE_SELECT_PIN = 2;  // STI board "SLAVE" pin (if populated)
constexpr bool MODE_SELECT_ACTIVE_HIGH_FOR_SERIAL = true;
constexpr uint8_t MODE_STEP_DIR_LEVEL = MODE_SELECT_ACTIVE_HIGH_FOR_SERIAL ? 0 : 1;
constexpr bool BRINGUP_TEST_BOTH_MODE_LEVELS = true;
constexpr bool RUN_DIRECT_MATRIX_FOREVER = true;
constexpr uint HEARTBEAT_LED_PIN = 16;
constexpr bool HEARTBEAT_ENABLED = false;
constexpr uint BACKLIGHT_DISABLE_PINS[] = {4, 5};
constexpr size_t BACKLIGHT_DISABLE_PIN_COUNT = sizeof(BACKLIGHT_DISABLE_PINS) / sizeof(BACKLIGHT_DISABLE_PINS[0]);
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 500;
constexpr uint8_t HEARTBEAT_LEVEL = 24;
constexpr uint32_t STEP_DELAY_US = 2600;
constexpr uint32_t HOMING_STEP_DELAY_US = 3200;
constexpr uint32_t HOMING_SEARCH_MAX_STEPS = 2800;
constexpr uint32_t HOMING_STOP_FALLBACK_STEPS = 1200;
constexpr uint32_t ENDPOINT_DWELL_MS = 400;
constexpr uint32_t BRINGUP_PULSE_COUNT = 2000;
constexpr uint32_t BRINGUP_PULSE_HIGH_US = 80;
constexpr uint32_t BRINGUP_PULSE_LOW_US = 6000;
constexpr uint32_t BRINGUP_DIR_SETTLE_MS = 20;
constexpr uint32_t BRINGUP_BETWEEN_DIRECTIONS_MS = 800;
constexpr bool BRINGUP_TEST_BOTH_STEP_POLARITIES = true;
constexpr bool BRINGUP_DEFAULT_STEP_ACTIVE_HIGH = true;
constexpr bool BRINGUP_TEST_OPEN_DRAIN_STYLE = true;
constexpr uint32_t SERIAL_WAIT_TIMEOUT_MS = 60000;
constexpr const char* FW_SIGNATURE = "X27_STI_MOTOR_D_MATRIX_20260330_D";
constexpr uint UNUSED_STI_INPUT_PINS[] = {3, 4, 6, 7, 9, 10};  // A/B/C step+dir inputs
constexpr size_t UNUSED_STI_INPUT_PIN_COUNT = sizeof(UNUSED_STI_INPUT_PINS) / sizeof(UNUSED_STI_INPUT_PINS[0]);

struct GaugePinMap {
    const char* name;
    uint pin_step;
    uint pin_dir;
    int pin_sensor;  // -1 if no sensor fitted
    bool sensor_active_high;
};

// Single-motor board profile: only channel D is wired.
// D: STEP=GPIO12 DIR=GPIO13 SENSOR=GPIO14(optional)
constexpr GaugePinMap GAUGES[] = {
    {"Gauge D", 12, 13, -1, false},
};

constexpr size_t GAUGE_COUNT = sizeof(GAUGES) / sizeof(GAUGES[0]);

struct GaugeState {
    x27_motor_t motor;
    bool ready;
    bool used_sensor_home;
    int32_t min_pos;
    int32_t max_pos;
};

GaugeState g_state[GAUGE_COUNT] = {};
WS2812 g_heartbeat_led(pio0, 0, HEARTBEAT_LED_PIN, false);
bool g_heartbeat_ready = false;

void update_heartbeat_led();

void wait_for_serial_attach_window() {
    absolute_time_t deadline = make_timeout_time_ms(SERIAL_WAIT_TIMEOUT_MS);
    while (!stdio_usb_connected()) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            break;
        }
        sleep_ms(20);
    }
}

void init_heartbeat_led() {
    if (!HEARTBEAT_ENABLED) {
        g_heartbeat_ready = false;
        return;
    }

    g_heartbeat_ready = g_heartbeat_led.begin(1);
    if (!g_heartbeat_ready) {
        printf("Heartbeat WS2812 init failed on GPIO%u\n", HEARTBEAT_LED_PIN);
        return;
    }
    g_heartbeat_led.setPixel(0, g_heartbeat_led.rgb(0, 0, 0));
    g_heartbeat_led.show();
}

void force_backlight_off() {
    for (size_t i = 0; i < BACKLIGHT_DISABLE_PIN_COUNT; ++i) {
        uint pin = BACKLIGHT_DISABLE_PINS[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_disable_pulls(pin);
        gpio_put(pin, 0);
    }
}

void tie_unused_sti_inputs_defined_levels() {
    for (size_t i = 0; i < UNUSED_STI_INPUT_PIN_COUNT; ++i) {
        uint pin = UNUSED_STI_INPUT_PINS[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_disable_pulls(pin);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_put(pin, 0);
    }
}

void init_shared_reset_pin() {
    // Keep shared reset control local in this test. If polarity is opposite on
    // your board, flip RESET_ACTIVE_LOW above.
    gpio_init(SHARED_RESET_PIN);
    gpio_set_dir(SHARED_RESET_PIN, GPIO_OUT);
    gpio_set_drive_strength(SHARED_RESET_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(SHARED_RESET_PIN, GPIO_SLEW_RATE_FAST);
    gpio_disable_pulls(SHARED_RESET_PIN);
    gpio_put(SHARED_RESET_PIN, RESET_ENABLE_LEVEL);
    sleep_ms(2);
}

void set_mode_select_level(uint8_t level) {
    if (MODE_SELECT_PIN < 0) {
        return;
    }
    gpio_put((uint)MODE_SELECT_PIN, level ? 1 : 0);
}

void init_mode_select_pin() {
    if (MODE_SELECT_PIN < 0) {
        return;
    }

    gpio_init((uint)MODE_SELECT_PIN);
    gpio_set_dir((uint)MODE_SELECT_PIN, GPIO_OUT);
    gpio_set_drive_strength((uint)MODE_SELECT_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate((uint)MODE_SELECT_PIN, GPIO_SLEW_RATE_FAST);
    gpio_disable_pulls((uint)MODE_SELECT_PIN);
    set_mode_select_level(MODE_STEP_DIR_LEVEL);
    sleep_ms(1);
}

void drive_logic_level(uint pin, bool level, bool open_drain_style) {
    if (!open_drain_style) {
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, level ? 1 : 0);
        return;
    }

    // Open-drain style: low = strong pull-down, high = hi-Z.
    if (level) {
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
    } else {
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, 0);
    }
}

void pulse_step_pin(uint pin_step, uint32_t count, bool step_active_high, bool open_drain_style) {
    bool idle_level = step_active_high ? false : true;
    bool pulse_level = !idle_level;

    drive_logic_level(pin_step, idle_level, open_drain_style);
    for (uint32_t i = 0; i < count; ++i) {
        drive_logic_level(pin_step, pulse_level, open_drain_style);
        sleep_us(BRINGUP_PULSE_HIGH_US);
        drive_logic_level(pin_step, idle_level, open_drain_style);
        sleep_us(BRINGUP_PULSE_LOW_US);
        update_heartbeat_led();
    }

    drive_logic_level(pin_step, idle_level, open_drain_style);
}

void run_direct_pulse_bringup() {
    printf("Direct pulse bring-up: testing mode/reset/step-polarity/open-drain\n");

    for (size_t i = 0; i < GAUGE_COUNT; ++i) {
        gpio_init(GAUGES[i].pin_step);
        gpio_set_dir(GAUGES[i].pin_step, GPIO_OUT);
        gpio_set_drive_strength(GAUGES[i].pin_step, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(GAUGES[i].pin_step, GPIO_SLEW_RATE_FAST);
        gpio_disable_pulls(GAUGES[i].pin_step);
        gpio_put(GAUGES[i].pin_step, 0);

        gpio_init(GAUGES[i].pin_dir);
        gpio_set_dir(GAUGES[i].pin_dir, GPIO_OUT);
        gpio_set_drive_strength(GAUGES[i].pin_dir, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(GAUGES[i].pin_dir, GPIO_SLEW_RATE_FAST);
        gpio_disable_pulls(GAUGES[i].pin_dir);
        gpio_put(GAUGES[i].pin_dir, 0);
    }

    uint8_t polarity_start = BRINGUP_TEST_BOTH_STEP_POLARITIES ? 0 : (BRINGUP_DEFAULT_STEP_ACTIVE_HIGH ? 1 : 0);
    uint8_t polarity_end = BRINGUP_TEST_BOTH_STEP_POLARITIES ? 1 : (BRINGUP_DEFAULT_STEP_ACTIVE_HIGH ? 1 : 0);
    uint8_t open_drain_end = BRINGUP_TEST_OPEN_DRAIN_STYLE ? 1 : 0;

    uint8_t mode_start = BRINGUP_TEST_BOTH_MODE_LEVELS ? 0 : MODE_STEP_DIR_LEVEL;
    uint8_t mode_end = BRINGUP_TEST_BOTH_MODE_LEVELS ? 1 : MODE_STEP_DIR_LEVEL;

    for (uint8_t open_drain = 0; open_drain <= open_drain_end; ++open_drain) {
        bool open_drain_style = (open_drain != 0);
        for (uint8_t step_active_high_u8 = polarity_start; step_active_high_u8 <= polarity_end; ++step_active_high_u8) {
            bool step_active_high = (step_active_high_u8 != 0);

            printf("Signal style: open_drain=%u step_active_high=%u\n",
                   open_drain_style ? 1u : 0u,
                   step_active_high ? 1u : 0u);

            for (uint8_t mode_level = mode_start; mode_level <= mode_end; ++mode_level) {
                set_mode_select_level(mode_level);
                sleep_ms(2);
                printf("Mode select level=%u (pin=%d)\n", mode_level, MODE_SELECT_PIN);

                for (uint8_t reset_level = 0; reset_level <= 1; ++reset_level) {
                    gpio_put(SHARED_RESET_PIN, reset_level);
                    sleep_ms(10);
                    printf("  Reset test level=%u\n", reset_level);

                    for (size_t i = 0; i < GAUGE_COUNT; ++i) {
                        const GaugePinMap& g = GAUGES[i];
                        printf("    %s dir=0 pulses=%lu\n", g.name, (unsigned long)BRINGUP_PULSE_COUNT);
                        drive_logic_level(g.pin_dir, false, open_drain_style);
                        sleep_ms(BRINGUP_DIR_SETTLE_MS);
                        pulse_step_pin(g.pin_step, BRINGUP_PULSE_COUNT, step_active_high, open_drain_style);
                        sleep_ms(BRINGUP_BETWEEN_DIRECTIONS_MS);

                        printf("    %s dir=1 pulses=%lu\n", g.name, (unsigned long)BRINGUP_PULSE_COUNT);
                        drive_logic_level(g.pin_dir, true, open_drain_style);
                        sleep_ms(BRINGUP_DIR_SETTLE_MS);
                        pulse_step_pin(g.pin_step, BRINGUP_PULSE_COUNT, step_active_high, open_drain_style);
                        sleep_ms(BRINGUP_BETWEEN_DIRECTIONS_MS);
                    }
                }
            }
        }
    }

    set_mode_select_level(MODE_STEP_DIR_LEVEL);
    gpio_put(SHARED_RESET_PIN, RESET_ENABLE_LEVEL);
    sleep_ms(5);
    printf("Direct pulse bring-up complete; mode/reset restored to configured run levels\n");
}

void update_heartbeat_led() {
    if (!HEARTBEAT_ENABLED) {
        return;
    }

    static absolute_time_t next_toggle = nil_time;
    static bool level = false;

    if (is_nil_time(next_toggle)) {
        next_toggle = make_timeout_time_ms(HEARTBEAT_INTERVAL_MS);
    }

    if (absolute_time_diff_us(get_absolute_time(), next_toggle) <= 0) {
        level = !level;
        if (g_heartbeat_ready) {
            uint32_t color = level ? g_heartbeat_led.rgb(0, HEARTBEAT_LEVEL, 0) : g_heartbeat_led.rgb(0, 0, 0);
            g_heartbeat_led.setPixel(0, color);
            g_heartbeat_led.show();
        }
        next_toggle = make_timeout_time_ms(HEARTBEAT_INTERVAL_MS);
    }
}

int next_ready_index(int start_index) {
    for (size_t i = 0; i < GAUGE_COUNT; ++i) {
        int idx = (start_index + (int)i) % (int)GAUGE_COUNT;
        if (g_state[idx].ready) return idx;
    }
    return -1;
}

bool home_gauge(size_t idx) {
    GaugeState& state = g_state[idx];
    x27_motor_t* motor = &state.motor;
    const GaugePinMap& pins = GAUGES[idx];

    x27_set_speed(motor, HOMING_STEP_DELAY_US);

    bool homed_with_sensor = false;
    bool sensor_configured = false;

    if (pins.pin_sensor >= 0) {
        sensor_configured = x27_config_homing_sensor(motor, pins.pin_sensor, pins.sensor_active_high, true);
        if (sensor_configured) {
            bool sensor_now = pins.sensor_active_high ? (gpio_get((uint)pins.pin_sensor) != 0) : (gpio_get((uint)pins.pin_sensor) == 0);
            if (sensor_now) {
                homed_with_sensor = true;
            } else {
                homed_with_sensor = x27_home_with_sensor(motor, -1, HOMING_SEARCH_MAX_STEPS);
                if (!homed_with_sensor) {
                    homed_with_sensor = x27_home_with_sensor(motor, +1, HOMING_SEARCH_MAX_STEPS);
                }
            }
        }
    }

    if (!homed_with_sensor) {
        // Fallback for gauges still using the internal stop as zero reference.
        x27_home_to_stop(motor, -1, HOMING_STOP_FALLBACK_STEPS);
    }

    motor->current_position = 0;
    motor->target_position = 0;
    state.used_sensor_home = homed_with_sensor;
    state.min_pos = 0;
    state.max_pos = X27_MAX_POSITION;

    x27_set_speed(motor, STEP_DELAY_US);
    x27_sleep(motor);

    printf("%s homed: sensor_cfg=%d sensor_home=%d range=[%ld..%ld]\n",
           pins.name,
           sensor_configured ? 1 : 0,
           homed_with_sensor ? 1 : 0,
           (long)state.min_pos,
           (long)state.max_pos);

    return true;
}

void init_all_gauges() {
    printf("Initializing STI6606Z motor D only profile...\n");

    for (size_t i = 0; i < GAUGE_COUNT; ++i) {
        const GaugePinMap& pins = GAUGES[i];
        x27_vid6606_config_t cfg = {
            .pin_step = pins.pin_step,
            .pin_dir = pins.pin_dir,
            .pin_reset = 0,
        };

        bool ok = x27_init_vid6606(&g_state[i].motor, &cfg, X27_MODE_MICRO_STEP);
        if (!ok) {
            g_state[i].ready = false;
            printf("%s init failed\n", pins.name);
            continue;
        }

        g_state[i].ready = true;
        home_gauge(i);
    }
}

void run_sequential_sweeps() {
    int active = next_ready_index(0);
    if (active < 0) {
        printf("No gauges initialized; idle loop\n");
        while (true) {
            update_heartbeat_led();
            sleep_ms(1000);
        }
    }

    bool leg_to_max = true;
    absolute_time_t hold_until = nil_time;
    absolute_time_t next_status = make_timeout_time_ms(2000);

    x27_wake(&g_state[active].motor);
    x27_set_position(&g_state[active].motor, g_state[active].max_pos);
    printf("Starting sweep on %s\n", GAUGES[active].name);

    while (true) {
        update_heartbeat_led();

        GaugeState& state = g_state[active];
        x27_update(&state.motor);

        if (x27_is_at_target(&state.motor)) {
            if (is_nil_time(hold_until)) {
                hold_until = make_timeout_time_ms(ENDPOINT_DWELL_MS);
            } else if (absolute_time_diff_us(get_absolute_time(), hold_until) <= 0) {
                hold_until = nil_time;
                if (leg_to_max) {
                    leg_to_max = false;
                    x27_set_position(&state.motor, state.min_pos);
                } else {
                    x27_sleep(&state.motor);
                    active = next_ready_index(active + 1);
                    leg_to_max = true;

                    GaugeState& next = g_state[active];
                    x27_wake(&next.motor);
                    x27_set_position(&next.motor, next.max_pos);
                    printf("Switching sweep to %s\n", GAUGES[active].name);
                }
            }
        }

        if (absolute_time_diff_us(get_absolute_time(), next_status) <= 0) {
            printf("[%s] active=%s pos=%ld target=%ld\n",
                   FW_SIGNATURE,
                   GAUGES[active].name,
                   (long)state.motor.current_position,
                   (long)state.motor.target_position);
            next_status = make_timeout_time_ms(2000);
        }

        sleep_us(250);
    }
}

}  // namespace

int main() {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    wait_for_serial_attach_window();
    sleep_ms(300);
    printf("[%s] boot\n", FW_SIGNATURE);
    force_backlight_off();
    tie_unused_sti_inputs_defined_levels();
    init_heartbeat_led();
    init_mode_select_pin();
    init_shared_reset_pin();
    run_direct_pulse_bringup();

    printf("STI6606Z X27 sequential sweep test\n");
    printf("Pulse timing us: dir=%d high=%d low=%d\n",
        X27_VID6606_DIR_SETUP_US,
        X27_VID6606_STEP_HIGH_US,
        X27_VID6606_STEP_LOW_US);
    printf("Shared reset pin=%u enable_level=%u (active_low=%u)\n",
        SHARED_RESET_PIN,
        RESET_ENABLE_LEVEL,
        RESET_ACTIVE_LOW ? 1u : 0u);
    printf("Mode select pin=%d run_level=%u (active_high_for_serial=%u, test_both=%u)\n",
        MODE_SELECT_PIN,
        MODE_STEP_DIR_LEVEL,
        MODE_SELECT_ACTIVE_HIGH_FOR_SERIAL ? 1u : 0u,
        BRINGUP_TEST_BOTH_MODE_LEVELS ? 1u : 0u);
    printf("Heartbeat/backlight enabled=%u\n", HEARTBEAT_ENABLED ? 1u : 0u);
    printf("Forced backlight pins low:");
    for (size_t i = 0; i < BACKLIGHT_DISABLE_PIN_COUNT; ++i) {
        printf(" %u", BACKLIGHT_DISABLE_PINS[i]);
    }
    printf("\n");
    printf("Unused STI inputs forced low:");
    for (size_t i = 0; i < UNUSED_STI_INPUT_PIN_COUNT; ++i) {
        printf(" %u", UNUSED_STI_INPUT_PINS[i]);
    }
    printf("\n");
    printf("Startup: direct matrix for motor D\n");
    printf("Run: repeating mode/reset/step-polarity/open-drain matrix\n");

    if (RUN_DIRECT_MATRIX_FOREVER) {
        while (true) {
            run_direct_pulse_bringup();
            sleep_ms(500);
        }
    }

    init_all_gauges();
    run_sequential_sweeps();
    return 0;
}