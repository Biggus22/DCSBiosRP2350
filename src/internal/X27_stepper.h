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

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// X27.168 specifications
#define X27_MAX_POSITION 945    // Maximum steps (315° / 3)
#define X27_STEP_ANGLE   0.333f // Degrees per step
#define X27_MIN_STEP_US  1000   // Minimum microseconds between steps

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
    uint pin_reset;    // RESET pin (shared across all motors, can be 0 if tied high)
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
    uint8_t current_step_index;
    bool initialized;
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

#ifdef __cplusplus
}
#endif

#endif // X27_STEPPER_H