// Test program for X27.168 using DRV8833 breakout board on RP2350-Zero.
// Actual board GPIO mapping used here (Motor A):
//   COIL1 -> GPIO3, COIL2 -> GPIO4, COIL3 -> GPIO5, COIL4 -> GPIO6
//   OPTICAL_A (hall sensor) -> GPIO7
//   GAUGE_BACKLIGHT (PWM test output) -> GPIO8
//
// **ALL TUNING PARAMETERS ARE IN THIS FILE - See defines below to adjust stepping mode,**
// **speed, and full-rotation behavior. No CMakeLists.txt changes needed.**

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "src/internal/X27_stepper.h"

namespace {

constexpr uint DRV8833_COIL1_PIN = 3;
constexpr uint DRV8833_COIL2_PIN = 4;
constexpr uint DRV8833_COIL3_PIN = 5;
constexpr uint DRV8833_COIL4_PIN = 6;
constexpr int HOMING_SENSOR_PIN = 7;
constexpr uint BACKLIGHT_PWM_PIN = 8;

// **TUNING: Stepping mode for current limiting and smoothness**
// X27_MODE_FULL_STEP:  4 coils, full current per step (faster, higher current)
// X27_MODE_HALF_STEP:  8 coils, alternates 2-coil and 1-coil phases (smoother, lower avg current)
// X27_MODE_MICRO_STEP: 12 coils (native 1/3 step resolution, smoothest)
// Use HALF_STEP for low-current, jitter-free operation on fixed PCBs without resistors.
#ifndef X27_TUNING_STEP_MODE
#define X27_TUNING_STEP_MODE X27_MODE_FULL_STEP
#endif

// **TUNING: Step delay in microseconds (lower = faster, higher = smoother)**
// Probe uses 7000 µs; main motor sweep uses a slower delay for reduced jitter.
// Increase this value to smooth out stepping vibration and reduce EMI.
#ifndef X27_TUNING_MAIN_DELAY_US
#define X27_TUNING_MAIN_DELAY_US 3000
#endif

// **TUNING: Full-rotation support**
// Set to true for full 360° rotation (0 to 1080 steps without mechanical limits)
// Set to false for limited-range X27 (e.g., standard 315° gauge, ~300 step sweep window)
#ifndef X27_FULL_ROTATION_MODE
#define X27_FULL_ROTATION_MODE true
#endif

// **TUNING: Hall active-window measurement**
// When enabled, prints rising/falling edge positions and active width in steps.
// Motion will continue through the hall window so width can be measured.
#ifndef X27_MEASURE_HALL_ACTIVE_STEPS
#define X27_MEASURE_HALL_ACTIVE_STEPS true
#endif

// **TUNING: Hall sensor offset compensation**
// The hall sensor mark occupies a range of steps (typically ~50 steps).
// These offsets are applied to zero_offset to compensate for the "dead zone."
// Positive offset advances zero past the hall window; negative offset delays it.
// Tune separately for forward and reverse because of mechanical backlash/hysteresis.
#ifndef X27_HALL_OFFSET_FORWARD_STEPS
#define X27_HALL_OFFSET_FORWARD_STEPS 0
#endif

#ifndef X27_HALL_OFFSET_REVERSE_STEPS
#define X27_HALL_OFFSET_REVERSE_STEPS 0
#endif

}  // namespace

int main() {
    stdio_init_all();
    sleep_ms(2000); // give host time to open the serial monitor after reboot

    x27_motor_t motor;
    const x27_gpio_config_t cfg = {
        DRV8833_COIL1_PIN,
        DRV8833_COIL2_PIN,
        DRV8833_COIL3_PIN,
        DRV8833_COIL4_PIN,
    };

    printf("Initializing X27 DRV8833 (Motor A) on pins %u %u %u %u\n",
           cfg.pin_coil1_a,
           cfg.pin_coil1_b,
           cfg.pin_coil2_a,
           cfg.pin_coil2_b);

    bool ok = x27_init_gpio(&motor, &cfg, X27_TUNING_STEP_MODE);
    if (!ok) {
        printf("X27 init failed\n");
        return 1;
    }

    // Reduce speed and sweep a safe limited range around center to avoid hitting mechanical stops
    x27_set_speed(&motor, X27_TUNING_MAIN_DELAY_US);  // Tuned for low current and smooth motion

#if X27_FULL_ROTATION_MODE
    printf("Full-rotation mode: Hall sensor homing required\n");
#endif

    // Configure hall homing sensor on OPTICAL_A (GPIO7).
    const bool SENSOR_ACTIVE_HIGH = false; // flip to true if your sensor asserts high
    if (x27_config_homing_sensor(&motor, HOMING_SENSOR_PIN, SENSOR_ACTIVE_HIGH, true)) {
        printf("Homing sensor configured on pin %d (active low, pull-up enabled)\n", HOMING_SENSOR_PIN);
        // Search backward up to 4000 steps (covers full 1080-step range multiple times)
        bool homed = x27_home_with_sensor(&motor, -1, 4000);
        if (homed) {
            printf("Homing: sensor triggered, position set to zero\n");
#if X27_FULL_ROTATION_MODE
            printf("Ready for full 360° rotation sweep (0 to %d)\n", X27_MAX_POSITION);
#endif
        } else {
            printf("Homing: sensor not found (search completed without trigger)\n");
#if X27_FULL_ROTATION_MODE
            printf("ERROR: Full-rotation mode requires hall sensor. Check GPIO%d wiring.\n", HOMING_SENSOR_PIN);
#else
            printf("Falling back to mechanical stop\n");
            x27_home_to_stop(&motor, -1, 500); // safe fallback for limited-range X27
#endif
        }
    } else {
        printf("Homing sensor configuration failed; skipping sensor homing\n");
    }

    const int32_t FULL_MAX = X27_MAX_POSITION;
    int32_t min_pos = 0;
    int32_t max_pos = FULL_MAX;

#if X27_FULL_ROTATION_MODE
    // Full rotation: allow 0 to 1080 after zeroing
    printf("Full-rotation mode: sweeping 0 to %d after zero detection\n", FULL_MAX);
#else
    // Limited sweep: constrain around center for safety (standard 315° X27 gauges)
    const int32_t center = FULL_MAX / 2;
    const int32_t safety_range = 300;
    min_pos = (center - safety_range > 0) ? center - safety_range : 0;
    max_pos = (center + safety_range < FULL_MAX) ? center + safety_range : FULL_MAX;
    printf("Limited-sweep mode (safety): %d to %d (center=%d)\n", min_pos, max_pos, center);
#endif

    int32_t zero_offset = 0;

    printf("Using sweep range %d .. %d\n", min_pos, max_pos);

    // Prepare LEDs: onboard (sensor indicator) + external on GAUGE_BACKLIGHT/GPIO15 (fade)
    const uint LED_PIN = PICO_DEFAULT_LED_PIN; // lights on sensor trigger
    const uint LED_PIN_EXT = BACKLIGHT_PWM_PIN; // fades continuously
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    gpio_set_function(LED_PIN_EXT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LED_PIN_EXT);
    pwm_set_wrap(slice, 255);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_enabled(slice, true);
    constexpr uint16_t BACKLIGHT_ON_LEVEL = 255;
    constexpr uint16_t BACKLIGHT_OFF_LEVEL = 0;
    pwm_set_gpio_level(LED_PIN_EXT, BACKLIGHT_OFF_LEVEL);

    // Quick preflight: sample sensor for ~2s and mirror active state to LEDs
    printf("Sensor preflight: observing GPIO%d for 2s (onboard LED + backlight mirror hall active)\n", motor.homing_pin);
    int last_raw = -1;
    {
        int raw = gpio_get(motor.homing_pin);
        bool active = motor.homing_active_high ? (raw != 0) : (raw == 0);
        printf("Initial hall state: raw=%d active=%d\n", raw, active ? 1 : 0);
    }
    for (int i = 0; i < 1000; ++i) { // 1000 * 2ms = 2s
        int raw = gpio_get(motor.homing_pin);
        bool active = motor.homing_active_high ? (raw != 0) : (raw == 0);
        if (raw != last_raw) {
            printf("Sensor level now %d\n", raw);
            if (active) {
                printf("Hall sensor detected during preflight\n");
            }
            last_raw = raw;
        }
        gpio_put(LED_PIN, active ? 1 : 0);
        pwm_set_gpio_level(LED_PIN_EXT, active ? BACKLIGHT_ON_LEVEL : BACKLIGHT_OFF_LEVEL);
        sleep_ms(2);
    }

    while (true) {
        printf("Sweep forward to %d\n", max_pos);
        x27_set_position(&motor, max_pos);
        // Monitor continuously during motion and report sensor triggers
        bool prev_active = motor.homing_configured ? (motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0)) : false;
        bool hall_window_open = prev_active;
        int32_t hall_window_start = motor.current_position;
        if (prev_active) {
            printf("Hall sensor already active at forward sweep start: step=%ld\n", (long)motor.current_position);
        }
        while (motor.current_position != motor.target_position) {
            x27_update(&motor);
            if (motor.homing_configured) {
                bool cur_active = motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0);
                gpio_put(LED_PIN, cur_active ? 1 : 0); // mirror sensor level on onboard LED
                pwm_set_gpio_level(LED_PIN_EXT, cur_active ? BACKLIGHT_ON_LEVEL : BACKLIGHT_OFF_LEVEL);
                if (cur_active && !prev_active) {
                    printf("Hall sensor detected at step %ld (forward sweep)\n", (long)motor.current_position);
                    hall_window_open = true;
                    hall_window_start = motor.current_position;
                    // set zero at this physical position and apply forward offset
                    zero_offset = motor.current_position + X27_HALL_OFFSET_FORWARD_STEPS;
#if !X27_MEASURE_HALL_ACTIVE_STEPS
                    motor.target_position = motor.current_position; // stop movement immediately
#endif
                    printf("Zero set at physical step %d\n", zero_offset);
                    gpio_put(LED_PIN, 1);
                    sleep_ms(120);
                    gpio_put(LED_PIN, 0);
#if X27_FULL_ROTATION_MODE
                    // Full rotation: use entire range (zero is just a reference)
                    min_pos = 0;
                    max_pos = FULL_MAX;
#else
                    // Limited sweep: recompute sweep bounds around detected zero
                    const int32_t safety_range = 300;
                    min_pos = (zero_offset - safety_range > 0) ? zero_offset - safety_range : 0;
                    max_pos = (zero_offset + safety_range < FULL_MAX) ? zero_offset + safety_range : FULL_MAX;
#endif
                }
                if (!cur_active && prev_active && hall_window_open) {
                    int32_t hall_window_end = motor.current_position;
                    int32_t span_steps = hall_window_end - hall_window_start;
                    if (span_steps < 0) span_steps = -span_steps;
                    span_steps += 1;
                    printf("Hall sensor cleared at step %ld (forward sweep)\n", (long)motor.current_position);
                    printf("Hall active window (forward): start=%ld end=%ld width=%ld steps\n",
                           (long)hall_window_start,
                           (long)hall_window_end,
                           (long)span_steps);
                    hall_window_open = false;
                }
                prev_active = cur_active;
            }
            sleep_ms(1);
        }
        if (hall_window_open) {
            printf("Hall active window (forward): did not see falling edge before target (start=%ld)\n", (long)hall_window_start);
        }
        x27_sleep(&motor); // de-energize briefly at end

        printf("Sweep back to %d\n", min_pos);
        x27_set_position(&motor, min_pos);
        // Monitor continuously during motion and report sensor triggers
        prev_active = motor.homing_configured ? (motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0)) : false;
        hall_window_open = prev_active;
        hall_window_start = motor.current_position;
        if (prev_active) {
            printf("Hall sensor already active at reverse sweep start: step=%ld\n", (long)motor.current_position);
        }
        while (motor.current_position != motor.target_position) {
            x27_update(&motor);
            if (motor.homing_configured) {
                bool cur_active = motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0);
                gpio_put(LED_PIN, cur_active ? 1 : 0); // mirror sensor level on onboard LED
                pwm_set_gpio_level(LED_PIN_EXT, cur_active ? BACKLIGHT_ON_LEVEL : BACKLIGHT_OFF_LEVEL);
                if (cur_active && !prev_active) {
                    printf("Hall sensor detected at step %ld (reverse sweep)\n", (long)motor.current_position);
                    hall_window_open = true;
                    hall_window_start = motor.current_position;
                    // set zero at this physical position and apply reverse offset
                    zero_offset = motor.current_position + X27_HALL_OFFSET_REVERSE_STEPS;
#if !X27_MEASURE_HALL_ACTIVE_STEPS
                    motor.target_position = motor.current_position;
#endif
                    printf("Zero set at physical step %d\n", zero_offset);
                    gpio_put(LED_PIN, 1);
                    sleep_ms(120);
                    gpio_put(LED_PIN, 0);
#if X27_FULL_ROTATION_MODE
                    // Full rotation: use entire range (zero is just a reference)
                    min_pos = 0;
                    max_pos = FULL_MAX;
#else
                    // Limited sweep: recompute sweep bounds around detected zero
                    const int32_t safety_range = 300;
                    min_pos = (zero_offset - safety_range > 0) ? zero_offset - safety_range : 0;
                    max_pos = (zero_offset + safety_range < FULL_MAX) ? zero_offset + safety_range : FULL_MAX;
#endif
                }
                if (!cur_active && prev_active && hall_window_open) {
                    int32_t hall_window_end = motor.current_position;
                    int32_t span_steps = hall_window_end - hall_window_start;
                    if (span_steps < 0) span_steps = -span_steps;
                    span_steps += 1;
                    printf("Hall sensor cleared at step %ld (reverse sweep)\n", (long)motor.current_position);
                    printf("Hall active window (reverse): start=%ld end=%ld width=%ld steps\n",
                           (long)hall_window_start,
                           (long)hall_window_end,
                           (long)span_steps);
                    hall_window_open = false;
                }
                prev_active = cur_active;
            }
            sleep_ms(1);
        }
        if (hall_window_open) {
            printf("Hall active window (reverse): did not see falling edge before target (start=%ld)\n", (long)hall_window_start);
        }
        x27_sleep(&motor);
    }

    return 0;
}
