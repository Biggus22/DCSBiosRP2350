/*
 * 28BYJ-48 Stepper Motor Driver (ULN2003) for RP2350
 * Header: BYJ48_stepper.h
 * Place in: src/internal/
 */

#ifndef BYJ48_STEPPER_H
#define BYJ48_STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// Typical 28BYJ-48 with internal gearbox: commonly 2048 steps/rev (depends on variant)
#define BYJ48_STEPS_PER_REV 2048

typedef enum {
    BYJ_MODE_FULL_DOUBLE = 0,
    BYJ_MODE_FULL_SINGLE = 1,
    BYJ_MODE_HALF = 2
} byj_step_mode_t;

typedef enum {
    BYJ_OUTPUT_UNIPOLAR = 0,
    BYJ_OUTPUT_BIPOLAR = 1
} byj_output_mode_t;

typedef struct {
    uint pin0;
    uint pin1;
    uint pin2;
    uint pin3;
} byj_gpio_config_t;

typedef struct {
    byj_gpio_config_t cfg;
    byj_step_mode_t mode;
    byj_output_mode_t output_mode;
    int32_t current_position; // relative step count
    int32_t target_position;
    uint32_t step_delay_us;   // microseconds between steps
    uint8_t current_step_index;
    bool initialized;
} byj_motor_t;

// Initialize for direct GPIO drive (ULN2003 inputs). Pins are the 4 input pins from ULN2003.
bool byj_init_gpio(byj_motor_t *motor, const byj_gpio_config_t *cfg, byj_step_mode_t mode);

// Set absolute target position (relative step count)
void byj_set_position(byj_motor_t *motor, int32_t position);

// Move by relative steps (blocking)
void byj_step_steps(byj_motor_t *motor, int32_t steps);

// Non-blocking update; call periodically. Returns true if still moving.
bool byj_update(byj_motor_t *motor);

// Block until target reached
void byj_wait_complete(byj_motor_t *motor);

// Set speed in microseconds per step
void byj_set_speed(byj_motor_t *motor, uint32_t delay_us);

// Set whether the motor outputs should be driven as unipolar (ULN2003) or
// bipolar (H-bridge inputs like MX1508). Default is unipolar.
void byj_set_output_mode(byj_motor_t *motor, int output_mode);

// De-energize coils
void byj_sleep(byj_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif // BYJ48_STEPPER_H
