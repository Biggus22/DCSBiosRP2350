/*
 * X27.168 Stepper Motor Driver for RP2350
 * Implementation File: X27_stepper.cpp
 * Place in: src/internal/
 */

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "X27_stepper.h"

static inline bool x27_vid_has_reset_pin(const x27_motor_t *motor) {
    // Preserve legacy behavior where pin_reset == 0 means "unused/tied high".
    return motor && (motor->config.vid6606.pin_reset != 0);
}

static inline void x27_vid_set_outputs_enabled(x27_motor_t *motor, bool enabled) {
    if (!motor || motor->driver_type != X27_DRIVER_VID6606) return;
    if (!x27_vid_has_reset_pin(motor)) {
        motor->vid_outputs_enabled = true;
        return;
    }

    if (enabled) {
        gpio_put(motor->config.vid6606.pin_reset, 1);
        sleep_us(X27_VID6606_RESET_RECOVERY_US);
        motor->vid_outputs_enabled = true;
    } else {
        gpio_put(motor->config.vid6606.pin_reset, 0);
        motor->vid_outputs_enabled = false;
    }
}

// Full step sequence (4 steps per cycle)
static const uint8_t FULL_STEP_SEQUENCE[4][4] = {
    {1, 0, 1, 0},  // Step 0
    {1, 0, 0, 1},  // Step 1
    {0, 1, 0, 1},  // Step 2
    {0, 1, 1, 0}   // Step 3
};

// Half step sequence (8 steps per cycle)
static const uint8_t HALF_STEP_SEQUENCE[8][4] = {
    {1, 0, 1, 0},  // Step 0
    {1, 0, 0, 0},  // Step 1
    {1, 0, 0, 1},  // Step 2
    {0, 0, 0, 1},  // Step 3
    {0, 1, 0, 1},  // Step 4
    {0, 1, 0, 0},  // Step 5
    {0, 1, 1, 0},  // Step 6
    {0, 0, 1, 0}   // Step 7
};

// MICRO_STEP_SEQUENCE removed: it repeated HALF_STEP_SEQUENCE positions 3x
// with no additional mechanical resolution. MICRO_STEP now aliases HALF_STEP.
// In VID6606 mode, the external chip performs its own microstepping.

// Internal helper functions
static void x27_set_coils_gpio(x27_motor_t *motor, uint8_t c1a, uint8_t c1b, uint8_t c2a, uint8_t c2b) {
    gpio_put(motor->config.gpio.pin_coil1_a, c1a);
    gpio_put(motor->config.gpio.pin_coil1_b, c1b);
    gpio_put(motor->config.gpio.pin_coil2_a, c2a);
    gpio_put(motor->config.gpio.pin_coil2_b, c2b);
}

static uint32_t x27_steps_per_rev_for_mode(x27_step_mode_t mode) {
    uint32_t steps = X27_STEPS_PER_REV;
    switch (mode) {
        case X27_MODE_FULL_STEP:
            // Empirical scaling for this GPIO stepping table implementation.
            // FULL_STEP commands should map close to expected travel in gauges.
            steps = (X27_STEPS_PER_REV * 2) / 3;
            break;
        case X27_MODE_HALF_STEP:
        case X27_MODE_MICRO_STEP:
            // MICRO_STEP is an alias for HALF_STEP in GPIO mode.
            // In VID6606 mode, the chip handles microstepping independently.
            steps = X27_STEPS_PER_REV;
            break;
        default:
            steps = X27_STEPS_PER_REV;
            break;
    }
    return (steps == 0) ? 1 : steps;
}

static uint32_t x27_ramped_delay_us(const x27_motor_t *motor) {
    if (!motor) return X27_MIN_STEP_US;

    int32_t remaining = motor->target_position - motor->current_position;
    if (remaining < 0) remaining = -remaining;

    const uint32_t ramp_steps = 36;
    uint32_t cruise_delay = motor->step_delay_us;
    uint32_t max_extra_delay = cruise_delay;

    uint32_t accel_phase = 0;
    if (motor->ramp_steps_taken < ramp_steps) {
        accel_phase = ramp_steps - motor->ramp_steps_taken;
    }

    uint32_t decel_phase = 0;
    if ((uint32_t)remaining < ramp_steps) {
        decel_phase = ramp_steps - (uint32_t)remaining;
    }

    uint32_t phase;
    if (accel_phase > decel_phase) {
        phase = accel_phase;
    } else {
        // Deceleration arm: keep phase low near target so motor stays at cruise
        // speed. Gauge motors rely on holding torque to hold position instantly.
        // A slow-down ramp would add latency without improving accuracy.
        phase = ramp_steps - decel_phase;
    }
    uint32_t delay = cruise_delay + (max_extra_delay * phase) / ramp_steps;
    if (delay < X27_MIN_STEP_US) delay = X27_MIN_STEP_US;
    return delay;
}

static void x27_step(x27_motor_t *motor, int8_t direction) {
    int8_t logical_direction = direction;
    int8_t physical_direction = direction;
    if (motor->direction_inverted) {
        physical_direction = -physical_direction;
    }

    if (motor->driver_type == X27_DRIVER_GPIO) {
        // Direct GPIO: manually control coil sequence
        const uint8_t (*sequence)[4];
        uint8_t seq_len;
        
        // Select stepping sequence
        switch (motor->step_mode) {
            case X27_MODE_FULL_STEP:
                sequence = FULL_STEP_SEQUENCE;
                seq_len = 4;
                break;
            case X27_MODE_HALF_STEP:
            case X27_MODE_MICRO_STEP:
                sequence = HALF_STEP_SEQUENCE;
                seq_len = 8;
                break;
            default:
                sequence = HALF_STEP_SEQUENCE;
                seq_len = 8;
                break;
        }
        
        // Update step index
        if (physical_direction > 0) {
            motor->current_step_index = (motor->current_step_index + 1) % seq_len;
        } else {
            motor->current_step_index = (motor->current_step_index + seq_len - 1) % seq_len;
        }
        
        // Apply coil states
        uint8_t c1a = sequence[motor->current_step_index][0];
        uint8_t c1b = sequence[motor->current_step_index][1];
        uint8_t c2a = sequence[motor->current_step_index][2];
        uint8_t c2b = sequence[motor->current_step_index][3];
        
        x27_set_coils_gpio(motor, c1a, c1b, c2a, c2b);
        
    } else {
        // VID6606/STI6606: use step/direction interface
        if (!motor->vid_outputs_enabled) {
            x27_vid_set_outputs_enabled(motor, true);
        }

        // Set direction
        gpio_put(motor->config.vid6606.pin_dir, physical_direction > 0 ? 1 : 0);
        sleep_us(X27_VID6606_DIR_SETUP_US);
        
        // Generate step pulse (min 450ns high per datasheet)
        gpio_put(motor->config.vid6606.pin_step, 1);
        sleep_us(X27_VID6606_STEP_HIGH_US);
        gpio_put(motor->config.vid6606.pin_step, 0);
        sleep_us(X27_VID6606_STEP_LOW_US);
    }
    
    // Update position
    if (logical_direction > 0) {
        motor->current_position++;
    } else {
        motor->current_position--;
    }
}

// Public API implementation

bool x27_init_gpio(x27_motor_t *motor, const x27_gpio_config_t *config, x27_step_mode_t mode) {
    if (!motor || !config) return false;

    motor->driver_type = X27_DRIVER_GPIO;
    motor->config.gpio = *config;
    motor->step_mode = mode;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->step_delay_us = 2000;
    motor->last_step_time_us = time_us_64();
    motor->current_step_index = 0;
    motor->direction_inverted = false;
    motor->vid_outputs_enabled = true;
    motor->ramp_last_target_position = 0;
    motor->ramp_last_direction = 0;
    motor->ramp_steps_taken = 0;
    motor->ramp_active = false;
    motor->homing_pin = -1;
    motor->homing_active_high = false;
    motor->homing_configured = false;
    
    // Initialize GPIO pins
    gpio_init(config->pin_coil1_a);
    gpio_init(config->pin_coil1_b);
    gpio_init(config->pin_coil2_a);
    gpio_init(config->pin_coil2_b);
    
    gpio_set_dir(config->pin_coil1_a, GPIO_OUT);
    gpio_set_dir(config->pin_coil1_b, GPIO_OUT);
    gpio_set_dir(config->pin_coil2_a, GPIO_OUT);
    gpio_set_dir(config->pin_coil2_b, GPIO_OUT);
    
    motor->initialized = true;
    return true;
}

bool x27_init_vid6606(x27_motor_t *motor, const x27_vid6606_config_t *config, x27_step_mode_t mode) {
    if (!motor || !config) return false;
    if (config->pin_step == config->pin_dir) return false;

    motor->driver_type = X27_DRIVER_VID6606;
    motor->config.vid6606 = *config;
    motor->step_mode = mode;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->step_delay_us = 2000;
    motor->last_step_time_us = time_us_64();
    motor->current_step_index = 0;
    motor->direction_inverted = false;
    motor->vid_outputs_enabled = false;
    motor->ramp_last_target_position = 0;
    motor->ramp_last_direction = 0;
    motor->ramp_steps_taken = 0;
    motor->ramp_active = false;
    motor->homing_pin = -1;
    motor->homing_active_high = false;
    motor->homing_configured = false;
    
    // Initialize step and direction pins
    gpio_init(config->pin_step);
    gpio_init(config->pin_dir);
    
    gpio_set_dir(config->pin_step, GPIO_OUT);
    gpio_set_dir(config->pin_dir, GPIO_OUT);
    
    gpio_put(config->pin_step, 0);
    gpio_put(config->pin_dir, 0);
    
    // Initialize reset pin if specified
    if (config->pin_reset != 0) {
        gpio_init(config->pin_reset);
        gpio_set_dir(config->pin_reset, GPIO_OUT);
        
        // Per datasheet: hold RESET low during power-up, then set high
        gpio_put(config->pin_reset, 0);
        sleep_ms(1);
        gpio_put(config->pin_reset, 1);
        sleep_us(X27_VID6606_RESET_RECOVERY_US);
        motor->vid_outputs_enabled = true;
    } else {
        motor->vid_outputs_enabled = true;
    }
    
    motor->initialized = true;
    return true;
}

bool x27_config_homing_sensor(x27_motor_t *motor, int pin, bool active_high, bool pull_up) {
    if (!motor) return false;
    motor->homing_pin = pin;
    motor->homing_active_high = active_high;
    motor->homing_configured = true;

    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pull_up) {
        gpio_pull_up(pin);
    } else {
        gpio_pull_down(pin);
    }
    return true;
}

bool x27_home_with_sensor(x27_motor_t *motor, int8_t dir, uint32_t max_steps) {
    if (!motor || !motor->initialized || !motor->homing_configured) return false;
    if (dir == 0) dir = -1; // default search direction

    uint32_t steps = 0;
    while (steps < max_steps) {
        // Check sensor
        int val = gpio_get(motor->homing_pin);
        bool triggered = motor->homing_active_high ? (val != 0) : (val == 0);
        if (triggered) {
            motor->current_position = 0;
            return true;
        }

        x27_step(motor, dir);
        busy_wait_us(motor->step_delay_us);
        steps++;
    }
    return false;
}

void x27_home_to_stop(x27_motor_t *motor, int8_t dir, uint32_t max_steps) {
    if (!motor || !motor->initialized) return;
    if (dir == 0) dir = -1;
    uint32_t steps = 0;
    uint32_t limit = (max_steps == 0) ? (uint32_t)X27_MAX_POSITION : max_steps;
    while (steps < limit) {
        x27_step(motor, dir);
        busy_wait_us(motor->step_delay_us);
        steps++;
    }
    // At endpoint, set zero
    motor->current_position = 0;
}

void x27_home(x27_motor_t *motor) {
    // Sweep to zero by moving counter-clockwise beyond limits
    motor->target_position = -X27_MAX_POSITION;
    x27_wait_complete(motor);
    motor->current_position = 0;
    motor->target_position = 0;
}

void x27_set_position(x27_motor_t *motor, int32_t position) {
    if (!motor) return;
    if (position < 0) position = 0;
    if (position > X27_MAX_POSITION) position = X27_MAX_POSITION;
    motor->target_position = position;
}

void x27_set_angle(x27_motor_t *motor, float angle) {
    if (!motor) return;
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 360.0f) angle = 360.0f;
    uint32_t steps_per_rev = x27_get_effective_steps_per_rev(motor);
    int32_t position = (int32_t)((angle * (float)steps_per_rev) / 360.0f + 0.5f);
    x27_set_position(motor, position);
}

uint32_t x27_get_effective_steps_per_rev(const x27_motor_t *motor) {
    if (!motor) return x27_steps_per_rev_for_mode(X27_MODE_MICRO_STEP);
    return x27_steps_per_rev_for_mode(motor->step_mode);
}

float x27_get_effective_step_angle(const x27_motor_t *motor) {
    uint32_t steps_per_rev = x27_get_effective_steps_per_rev(motor);
    return 360.0f / (float)steps_per_rev;
}

float x27_get_angle(const x27_motor_t *motor) {
    if (!motor) return 0.0f;
    return (float)motor->current_position * x27_get_effective_step_angle(motor);
}

bool x27_update(x27_motor_t *motor) {
    if (!motor || !motor->initialized) {
        return false;
    }
    
    if (motor->current_position == motor->target_position) {
        motor->ramp_active = false;
        motor->ramp_steps_taken = 0;
        motor->ramp_last_direction = 0;
        motor->ramp_last_target_position = motor->target_position;
        return false;
    }

    int8_t direction = (motor->target_position > motor->current_position) ? 1 : -1;
    bool target_changed = (motor->target_position != motor->ramp_last_target_position);
    bool direction_changed = (direction != motor->ramp_last_direction);
    if (!motor->ramp_active || target_changed || direction_changed) {
        motor->ramp_active = true;
        motor->ramp_steps_taken = 0;
        motor->ramp_last_target_position = motor->target_position;
        motor->ramp_last_direction = direction;
    }
    
    uint64_t now = time_us_64();
    if (motor->last_step_time_us == 0) {
        motor->last_step_time_us = now;
        return true;
    }
    uint32_t dynamic_delay_us = x27_ramped_delay_us(motor);
    if (now - motor->last_step_time_us < dynamic_delay_us) {
        return true;
    }
    
    motor->last_step_time_us = now;
    x27_step(motor, direction);
    if (motor->ramp_steps_taken < 0xffffffffu) {
        motor->ramp_steps_taken++;
    }
    motor->ramp_last_target_position = motor->target_position;
    motor->ramp_last_direction = direction;
    
    return motor->current_position != motor->target_position;
}

void x27_wait_complete(x27_motor_t *motor) {
    while (x27_update(motor)) {
        sleep_us(100);
    }
}

void x27_set_speed(x27_motor_t *motor, uint32_t delay_us) {
    if (!motor) return;
    if (delay_us < X27_MIN_STEP_US) delay_us = X27_MIN_STEP_US;
    motor->step_delay_us = delay_us;
}

void x27_set_direction_inverted(x27_motor_t *motor, bool inverted) {
    if (!motor) return;
    motor->direction_inverted = inverted;
}

int32_t x27_get_position(const x27_motor_t *motor) {
    if (!motor) return 0;
    return motor->current_position;
}

bool x27_is_at_target(const x27_motor_t *motor) {
    if (!motor) return true;
    return motor->current_position == motor->target_position;
}

void x27_sleep(x27_motor_t *motor) {
    if (!motor || !motor->initialized) return;

    if (motor->driver_type == X27_DRIVER_GPIO) {
        x27_set_coils_gpio(motor, 0, 0, 0, 0);
    } else {
        // VID6606/STI6606: disable outputs through RESET when available.
        x27_vid_set_outputs_enabled(motor, false);
    }
}

void x27_wake(x27_motor_t *motor) {
    if (!motor || !motor->initialized) return;
    if (motor->driver_type == X27_DRIVER_GPIO) return;
    x27_vid_set_outputs_enabled(motor, true);
}
