/*
 * 28BYJ-48 Stepper Motor Driver (ULN2003) for RP2350
 * Implementation: BYJ48_stepper.cpp
 * Place in: src/internal/
 */

#include "BYJ48_stepper.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"

// Sequences copied from test harness
static const uint8_t SEQ_FULL_DOUBLE[4][4] = {
    {1,1,0,0},
    {0,1,1,0},
    {0,0,1,1},
    {1,0,0,1}
};

static const uint8_t SEQ_FULL_SINGLE[4][4] = {
    {1,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
};

static const uint8_t SEQ_HALF[8][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

// Bipolar H-bridge input sequence mapping (IN1..IN4) for full-step bipolar drive
static const uint8_t SEQ_BIPOLAR[4][4] = {
    {1,0,1,0},
    {0,1,1,0},
    {0,1,0,1},
    {1,0,0,1}
};

// Sets the four motor pins using the provided state array.
static void byj_set_pins(const byj_motor_t *m, const uint8_t state[4]) {
    gpio_put(m->cfg.pin0, state[0]);
    gpio_put(m->cfg.pin1, state[1]);
    gpio_put(m->cfg.pin2, state[2]);
    gpio_put(m->cfg.pin3, state[3]);
}

// Applies the step sequence based on the motor mode. If bipolar mode, it uses the bipolar sequence. Otherwise, it uses the sequence for the current mode.
static void byj_apply_step_index(byj_motor_t *m, int index) {
    // If operating in bipolar (H-bridge input) mode, use bipolar sequence
    if (m->output_mode == BYJ_OUTPUT_BIPOLAR) {
        const uint8_t *s = SEQ_BIPOLAR[index % 4];
        byj_set_pins(m, s);
        return;
    }

    switch (m->mode) {
        case BYJ_MODE_FULL_DOUBLE: {
            const uint8_t *s = SEQ_FULL_DOUBLE[index % 4];
            byj_set_pins(m, s);
            break;
        }
        case BYJ_MODE_FULL_SINGLE: {
            const uint8_t *s = SEQ_FULL_SINGLE[index % 4];
            byj_set_pins(m, s);
            break;
        }
        case BYJ_MODE_HALF: {
            const uint8_t *s = SEQ_HALF[index % 8];
            byj_set_pins(m, s);
            break;
        }
        default:
            break;
    }
}

// Initializes the motor configuration and GPIO pins. It sets the motor mode, position, and initializes all pins as outputs.
bool byj_init_gpio(byj_motor_t *motor, const byj_gpio_config_t *cfg, byj_step_mode_t mode) {
    if (!motor || !cfg) return false;
    motor->cfg = *cfg;
    motor->mode = mode;
    motor->current_position = 0;
    motor->target_position = 0;
    motor->step_delay_us = 2000;
    motor->current_step_index = 0;
    motor->output_mode = BYJ_OUTPUT_UNIPOLAR;
    motor->last_step_time_us = 0;

    gpio_init(motor->cfg.pin0);
    gpio_init(motor->cfg.pin1);
    gpio_init(motor->cfg.pin2);
    gpio_init(motor->cfg.pin3);

    gpio_set_dir(motor->cfg.pin0, GPIO_OUT);
    gpio_set_dir(motor->cfg.pin1, GPIO_OUT);
    gpio_set_dir(motor->cfg.pin2, GPIO_OUT);
    gpio_set_dir(motor->cfg.pin3, GPIO_OUT);

    // ensure outputs low
    gpio_put(motor->cfg.pin0, 0);
    gpio_put(motor->cfg.pin1, 0);
    gpio_put(motor->cfg.pin2, 0);
    gpio_put(motor->cfg.pin3, 0);

    motor->initialized = true;
    return true;
}

// Sets the motor's output mode. It changes the mode to bipolar if the input is BYJ_OUTPUT_BIPOLAR.
void byj_set_output_mode(byj_motor_t *motor, int output_mode) {
    if (!motor) return;
    motor->output_mode = (output_mode == BYJ_OUTPUT_BIPOLAR) ? BYJ_OUTPUT_BIPOLAR : BYJ_OUTPUT_UNIPOLAR;
}

void byj_set_speed(byj_motor_t *motor, uint32_t delay_us) {
    if (!motor) return;
    if (delay_us < 1) delay_us = 1;
    motor->step_delay_us = delay_us;
}

void byj_step_once(byj_motor_t *motor, int8_t dir) {
    int seq_len = (motor->mode == BYJ_MODE_HALF) ? 8 : 4;
    if (dir > 0) {
        motor->current_step_index = (motor->current_step_index + 1) % seq_len;
        motor->current_position++;
    } else {
        motor->current_step_index = (motor->current_step_index + seq_len - 1) % seq_len;
        motor->current_position--;
    }
    byj_apply_step_index(motor, motor->current_step_index);
}

void byj_step_steps(byj_motor_t *motor, int32_t steps) {
    if (!motor || !motor->initialized) return;
    int8_t dir = (steps >= 0) ? 1 : -1;
    int32_t count = (steps >= 0) ? steps : -steps;
    for (int32_t i = 0; i < count; ++i) {
        byj_step_once(motor, dir);
        busy_wait_us(motor->step_delay_us);
    }
}

void byj_set_position(byj_motor_t *motor, int32_t position) {
    if (!motor) return;
    motor->target_position = position;
}

bool byj_update(byj_motor_t *motor) {
    if (!motor || !motor->initialized) return false;
    if (motor->current_position == motor->target_position) return false;
    uint64_t now = time_us_64();
    if (now - motor->last_step_time_us < motor->step_delay_us) return true;
    motor->last_step_time_us = now;
    int8_t dir = (motor->target_position > motor->current_position) ? 1 : -1;
    byj_step_once(motor, dir);
    return motor->current_position != motor->target_position;
}

void byj_wait_complete(byj_motor_t *motor) {
    if (!motor) return;
    while (byj_update(motor)) {
        sleep_us(50);
    }
}

void byj_sleep(byj_motor_t *motor) {
    if (!motor || !motor->initialized) return;
    gpio_put(motor->cfg.pin0, 0);
    gpio_put(motor->cfg.pin1, 0);
    gpio_put(motor->cfg.pin2, 0);
    gpio_put(motor->cfg.pin3, 0);
}
