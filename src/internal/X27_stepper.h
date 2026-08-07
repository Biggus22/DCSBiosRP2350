/*
 * X27.168 Stepper Motor Driver for RP2350
 * Header File: X27_stepper.h
 * Place in: include/ or src/internal/
 * 
 * Supports direct GPIO control and VID6606/STI6606 step/direction control
 * 
 * The X27.168 is a 6-wire bipolar stepper with 315° range (945 steps at 1/3 step)
 * Common in automotive instrument clusters
 * 
 * VID6606/STI6606: Each chip controls up to 4 motors via step/direction interface
 *   - Each motor requires 2 pins: STEP (f(scx)) and DIR (CW/CCW)
 *   - Optional shared RESET pin for all motors
 *   - Chip handles microstepping internally (1/12° resolution)
 */

#ifndef X27_STEPPER_H
#define X27_STEPPER_H

/*
 * MODIFIED MOTOR NOTE:
 * The default constants in this driver assume X27.168 motors with the internal
 * mechanical limiter removed, allowing 360° rotation. Stock X27.168 motors have
 * a 315° mechanical range.
 *
 * For unmodified (315°) motors:
 *   - Set X27_STEPS_PER_REV to 945 (or (1080 * 315) / 360)
 *   - Clamp x27_set_angle() input to 315° max
 *   - Adjust x27_home_to_stop() limit for the shorter range
 *
 * For modified (360°) motors (this driver's default):
 *   - X27_STEPS_PER_REV = 1080 covers full 360°
 *   - x27_set_angle() accepts up to 360°
 *   - x27_home_to_stop() default limit of 1300 steps exceeds one full revolution
 */

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// X27.168 specifications
// Can be overridden at build time, for example:
// target_compile_definitions(... PRIVATE X27_STEPS_PER_REV=1080)
#ifndef X27_STEPS_PER_REV
#define X27_STEPS_PER_REV 1080
#endif

#define X27_MAX_POSITION X27_STEPS_PER_REV
#define X27_STEP_ANGLE   (360.0f / (float)X27_STEPS_PER_REV)
#define X27_MIN_STEP_US  1000   // Minimum microseconds between steps

// VID6606/STI6606 timing guard-bands (datasheet minima are sub-microsecond).
// These defaults are intentionally conservative for robustness.
#ifndef X27_VID6606_DIR_SETUP_US
#define X27_VID6606_DIR_SETUP_US 1
#endif

#ifndef X27_VID6606_STEP_HIGH_US
#define X27_VID6606_STEP_HIGH_US 1
#endif

#ifndef X27_VID6606_STEP_LOW_US
#define X27_VID6606_STEP_LOW_US 1
#endif

#ifndef X27_VID6606_RESET_RECOVERY_US
#define X27_VID6606_RESET_RECOVERY_US 5
#endif

// Stepping modes
typedef enum {
    X27_MODE_FULL_STEP = 0,
    X27_MODE_HALF_STEP = 1,
    X27_MODE_MICRO_STEP = 2  // 1/3 step (native for X27.168)
} x27_step_mode_t;

// Driver types
typedef enum {
    X27_DRIVER_GPIO = 0,      // Direct GPIO control (4 wires)
    X27_DRIVER_VID6606 = 1    // VID6606/STI6606 step/direction control
} x27_driver_type_t;

// GPIO configuration for direct drive
typedef struct {
    uint pin_coil1_a;  // Coil 1, phase A
    uint pin_coil1_b;  // Coil 1, phase B
    uint pin_coil2_a;  // Coil 2, phase A
    uint pin_coil2_b;  // Coil 2, phase B
} x27_gpio_config_t;

// VID6606 configuration (step/direction control)
typedef struct {
    uint pin_step;     // f(scx) - Step pulse pin
    uint pin_dir;      // CW/CCW - Direction pin
    // RESET pin (shared across all motors).
    // Legacy behavior: 0 means "unused/tied high".
    uint pin_reset;
} x27_vid6606_config_t;

// Motor instance
typedef struct {
    x27_driver_type_t driver_type;
    union {
        x27_gpio_config_t gpio;
        x27_vid6606_config_t vid6606;
    } config;
    
    int32_t current_position;
    int32_t target_position;
    x27_step_mode_t step_mode;
    uint32_t step_delay_us;
    uint64_t last_step_time_us;
    uint8_t current_step_index;
    bool initialized;
    bool vid_outputs_enabled;
    bool direction_inverted;
    int32_t ramp_last_target_position;
    int8_t ramp_last_direction;
    uint32_t ramp_steps_taken;
    bool ramp_active;
    // Homing sensor configuration (optional)
    int homing_pin;           // GPIO pin for homing sensor (or -1 if unused)
    bool homing_active_high;  // true if sensor asserts high
    bool homing_configured;
} x27_motor_t;

// Function declarations

/**
 * Initialize a motor with direct GPIO control
 */
bool x27_init_gpio(x27_motor_t *motor, const x27_gpio_config_t *config, x27_step_mode_t mode);

/**
 * Initialize a motor with VID6606/STI6606 control
 * Each motor needs 2 pins (step + direction)
 * RESET pin is optional (set to 0 if tied to VDD)
 */
bool x27_init_vid6606(x27_motor_t *motor, const x27_vid6606_config_t *config, x27_step_mode_t mode);

/**
 * Home the motor (sweep to zero position)
 */
void x27_home(x27_motor_t *motor);

/**
 * Set target position (0 to X27_MAX_POSITION)
 */
void x27_set_position(x27_motor_t *motor, int32_t position);

/**
 * Set target position by angle in degrees (0 to 315)
 */
void x27_set_angle(x27_motor_t *motor, float angle);

/**
 * Get effective steps/rev for current stepping mode.
 * If X27_STEPS_PER_REV represents micro-step resolution (default), then:
 *  - FULL_STEP = X27_STEPS_PER_REV / 3
 *  - HALF_STEP = X27_STEPS_PER_REV * 2 / 3
 *  - MICRO_STEP = X27_STEPS_PER_REV
 */
uint32_t x27_get_effective_steps_per_rev(const x27_motor_t *motor);

/**
 * Get effective angle per step for current stepping mode.
 */
float x27_get_effective_step_angle(const x27_motor_t *motor);

/**
 * Get current motor angle in degrees using mode-aware conversion.
 */
float x27_get_angle(const x27_motor_t *motor);

/**
 * Update motor - call regularly to step toward target
 * Returns true if motor is still moving
 */
bool x27_update(x27_motor_t *motor);

/**
 * Block until motor reaches target position
 */
void x27_wait_complete(x27_motor_t *motor);

/**
 * Set step delay in microseconds
 */
void x27_set_speed(x27_motor_t *motor, uint32_t delay_us);

/**
 * Invert the logical direction used by x27_step/x27_update.
 */
void x27_set_direction_inverted(x27_motor_t *motor, bool inverted);

/**
 * Get current position
 */
int32_t x27_get_position(const x27_motor_t *motor);

/**
 * Check if motor is at target
 */
bool x27_is_at_target(const x27_motor_t *motor);

/**
 * De-energize motor coils (reduce power consumption)
 */
void x27_sleep(x27_motor_t *motor);

/**
 * Re-enable outputs after x27_sleep(). For direct GPIO this is a no-op;
 * coil energization resumes on next commanded step.
 */
void x27_wake(x27_motor_t *motor);

/**
 * Configure an external homing/sensor input. When configured, call
 * `x27_home_with_sensor()` to perform a homing sweep until the sensor
 * asserts. If not configured, caller can use `x27_home_to_stop()` which
 * will step a bounded number of steps to reach a mechanical endstop.
 *
 * @param motor Motor instance
 * @param pin GPIO pin number for sensor input
 * @param active_high True if sensor reads high when active
 * @param pull_up True to enable internal pull-up, false for pull-down
 * @return true on success
 */
bool x27_config_homing_sensor(x27_motor_t *motor, int pin, bool active_high, bool pull_up);

/**
 * Home using the configured sensor. Moves in the given direction until the
 * sensor asserts or `max_steps` are reached. On success sets `current_position`
 * to zero. If no sensor configured, returns false.
 */
bool x27_home_with_sensor(x27_motor_t *motor, int8_t dir, uint32_t max_steps);

/**
 * Home by driving until user-specified limit (mechanical stop). This will
 * perform up to `max_steps` steps in `dir` direction and then set position
 * to zero at the endpoint. Use a safe `max_steps` to avoid overdriving.
 */
void x27_home_to_stop(x27_motor_t *motor, int8_t dir, uint32_t max_steps);

#ifdef __cplusplus
}
#endif

#endif // X27_STEPPER_H