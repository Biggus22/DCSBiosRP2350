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

// Micro step sequence (12 steps per cycle - 1/3 step)
static const uint8_t MICRO_STEP_SEQUENCE[12][4] = {
    {1, 0, 1, 0},  // Step 0
    {1, 0, 1, 0},  // Step 1
    {1, 0, 0, 1},  // Step 2
    {1, 0, 0, 1},  // Step 3
    {0, 1, 0, 1},  // Step 4
    {0, 1, 0, 1},  // Step 5
    {0, 1, 1, 0},  // Step 6
    {0, 1, 1, 0},  // Step 7
    {1, 0, 1, 0},  // Step 8 (wraps around)
    {1, 0, 1, 0},  // Step 9
    {1, 0, 0, 1},  // Step 10
    {1, 0, 0, 1}   // Step 11
};

// Internal helper functions
static void x27_set_coils_gpio(x27_motor_t *motor, uint8_t c1a, uint8_t c1b, uint8_t c2a, uint8_t c2b) {
    gpio_put(motor->config.gpio.pin_coil1_a, c1a);
    gpio_put(motor->config.gpio.pin_coil1_b, c1b);
    gpio_put(motor->config.gpio.pin_coil2_a, c2a);
    gpio_put(motor->config.gpio.pin_coil2_b, c2b);
}

static void x27_step(x27_motor_t *motor, int8_t direction) {
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
                sequence = HALF_STEP_SEQUENCE;
                seq_len = 8;
                break;
            case X27_MODE_MICRO_STEP:
            default:
                sequence = MICRO_STEP_SEQUENCE;
                seq_len = 12;
                break;
        }
        
        // Update step index
        if (direction > 0) {
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
        // Set direction
        gpio_put(motor->config.vid6606.pin_dir, direction > 0 ? 1 : 0);
        sleep_us(1); // Setup time (100ns min per datasheet)
        
        // Generate step pulse (min 450ns high per datasheet)
        gpio_put(motor->config.vid6606.pin_step, 1);
        sleep_us(1);
        gpio_put(motor->config.vid6606.pin_step, 0);
        sleep_us(1);
    }
    
    // Update position
    if (direction > 0) {
        motor->current_position++;
    } else {
        motor->current_position--;
    }
}

// Public API implementation

bool x27_init_gpio(x27_motor_t *motor, const x27_gpio_config_t *config, x27_step_mode_t mode) {
    motor->driver_type = X27_DRIVER_GPIO;
    motor->config.gpio = *config;
    motor->step_mode = mode;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->step_delay_us = 2000;
    motor->current_step_index = 0;
    
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
    motor->driver_type = X27_DRIVER_VID6606;
    motor->config.vid6606 = *config;
    motor->step_mode = mode;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->step_delay_us = 2000;
    motor->current_step_index = 0;
    
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
    }
    
    motor->initialized = true;
    return true;
}

void x27_home(x27_motor_t *motor) {
    // Sweep to zero by moving counter-clockwise beyond limits
    motor->target_position = -X27_MAX_POSITION;
    x27_wait_complete(motor);
    motor->current_position = 0;
    motor->target_position = 0;
}

void x27_set_position(x27_motor_t *motor, int32_t position) {
    if (position < 0) position = 0;
    if (position > X27_MAX_POSITION) position = X27_MAX_POSITION;
    motor->target_position = position;
}

void x27_set_angle(x27_motor_t *motor, float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 315.0f) angle = 315.0f;
    int32_t position = (int32_t)(angle / X27_STEP_ANGLE);
    x27_set_position(motor, position);
}

bool x27_update(x27_motor_t *motor) {
    static uint64_t last_step_time = 0;
    
    if (motor->current_position == motor->target_position) {
        return false;
    }
    
    uint64_t now = time_us_64();
    if (now - last_step_time < motor->step_delay_us) {
        return true;
    }
    
    last_step_time = now;
    int8_t direction = (motor->target_position > motor->current_position) ? 1 : -1;
    x27_step(motor, direction);
    
    return motor->current_position != motor->target_position;
}

void x27_wait_complete(x27_motor_t *motor) {
    while (x27_update(motor)) {
        sleep_us(100);
    }
}

void x27_set_speed(x27_motor_t *motor, uint32_t delay_us) {
    if (delay_us < X27_MIN_STEP_US) delay_us = X27_MIN_STEP_US;
    motor->step_delay_us = delay_us;
}

int32_t x27_get_position(const x27_motor_t *motor) {
    return motor->current_position;
}

bool x27_is_at_target(const x27_motor_t *motor) {
    return motor->current_position == motor->target_position;
}

void x27_sleep(x27_motor_t *motor) {
    if (motor->driver_type == X27_DRIVER_GPIO) {
        x27_set_coils_gpio(motor, 0, 0, 0, 0);
    } else {
        // VID6606: use RESET pin to disable outputs if available
        if (motor->config.vid6606.pin_reset != 0) {
            gpio_put(motor->config.vid6606.pin_reset, 0);
        }
    }
}