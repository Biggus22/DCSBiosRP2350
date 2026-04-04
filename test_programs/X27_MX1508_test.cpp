// Test program for X27.168 using MX1508 driver (IN1..IN4 -> GPIO 2..5)

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "src/internal/X27_stepper.h"

int main() {
    stdio_init_all();
    sleep_ms(2000); // give host time to open the serial monitor after reboot

    x27_motor_t motor;
    x27_gpio_config_t cfg;
    cfg.pin_coil1_a = 2; // IN1
    cfg.pin_coil1_b = 3; // IN2
    cfg.pin_coil2_a = 4; // IN3
    cfg.pin_coil2_b = 5; // IN4

    printf("Initializing X27 on pins %u %u %u %u\n", cfg.pin_coil1_a, cfg.pin_coil1_b, cfg.pin_coil2_a, cfg.pin_coil2_b);

    bool ok = x27_init_gpio(&motor, &cfg, X27_MODE_FULL_STEP);
    if (!ok) {
        printf("X27 init failed\n");
        return 1;
    }

    // Reduce speed and sweep a safe limited range around center to avoid hitting mechanical stops
    x27_set_speed(&motor, 3000); // 3ms per step (slower, more torque)

    // Configure hall homing sensor on GPIO16 (active low expected by default) with internal pull-up
    const bool SENSOR_ACTIVE_HIGH = false; // flip to true if your sensor asserts high
    if (x27_config_homing_sensor(&motor, 16, SENSOR_ACTIVE_HIGH, true)) {
        printf("Homing sensor configured on pin 16 (active low, pull-up enabled)\n");
        bool homed = x27_home_with_sensor(&motor, -1, 4000); // search backward up to 4000 steps
        if (homed) {
            printf("Homing: sensor triggered, position set to zero\n");
        } else {
            printf("Homing: sensor not found, falling back to mechanical stop\n");
            x27_home_to_stop(&motor, -1, 500); // safe fallback
        }
    } else {
        printf("Homing sensor configuration failed; skipping sensor homing\n");
    }

    const int32_t FULL_MAX = X27_MAX_POSITION;
    const int32_t center = FULL_MAX / 2;
    const int32_t safety_range = 300; // steps each side of detected zero (adjust as needed)
    int32_t min_pos = (center - safety_range > 0) ? center - safety_range : 0;
    int32_t max_pos = (center + safety_range < FULL_MAX) ? center + safety_range : FULL_MAX;

    bool zero_set = false;
    int32_t zero_offset = 0;

    printf("Using safe sweep range %d .. %d (center=%d)\n", min_pos, max_pos, center);

    // Prepare LEDs: onboard (sensor indicator) + external on GPIO15 (fade)
    const uint LED_PIN = PICO_DEFAULT_LED_PIN; // lights on sensor trigger
    const uint LED_PIN_EXT = 15;               // fades continuously
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    gpio_set_function(LED_PIN_EXT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LED_PIN_EXT);
    pwm_set_wrap(slice, 255);
    pwm_set_clkdiv(slice, 1.0f);
    pwm_set_enabled(slice, true);
    pwm_set_gpio_level(LED_PIN_EXT, 255); // max intensity

    // Quick preflight: sample the sensor for ~2s so you can see onboard LED state and raw reads in serial
    printf("Sensor preflight: observing GPIO%d for 2s (onboard LED mirrors raw level)\n", motor.homing_pin);
    int last_raw = -1;
    for (int i = 0; i < 1000; ++i) { // 1000 * 2ms = 2s
        int raw = gpio_get(motor.homing_pin);
        if (raw != last_raw) {
            printf("Sensor level now %d\n", raw);
            last_raw = raw;
        }
        gpio_put(LED_PIN, raw);
        sleep_ms(2);
    }

    while (true) {
        printf("Sweep forward to %d\n", max_pos);
        x27_set_position(&motor, max_pos);
        // Monitor continuously during motion and report sensor triggers
        bool prev_active = motor.homing_configured ? (motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0)) : false;
        while (motor.current_position != motor.target_position) {
            x27_update(&motor);
            if (motor.homing_configured) {
                bool cur_active = motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0);
                gpio_put(LED_PIN, cur_active ? 1 : 0); // mirror sensor level on onboard LED
                // external LED keeps fading separately
                if (cur_active && !prev_active) {
                    // set zero at this physical position and stop motion
                    zero_offset = motor.current_position;
                    zero_set = true;
                    motor.target_position = motor.current_position; // stop movement immediately
                    printf("Zero set at physical step %d\n", zero_offset);
                    gpio_put(LED_PIN, 1);
                    sleep_ms(120);
                    gpio_put(LED_PIN, 0);
                    // recompute sweep bounds around detected zero
                    min_pos = (zero_offset - safety_range > 0) ? zero_offset - safety_range : 0;
                    max_pos = (zero_offset + safety_range < FULL_MAX) ? zero_offset + safety_range : FULL_MAX;
                }
                prev_active = cur_active;
            }
            sleep_ms(1);
        }
        x27_sleep(&motor); // de-energize briefly at end

        printf("Sweep back to %d\n", min_pos);
        x27_set_position(&motor, min_pos);
        // Monitor continuously during motion and report sensor triggers
        prev_active = motor.homing_configured ? (motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0)) : false;
        while (motor.current_position != motor.target_position) {
            x27_update(&motor);
            if (motor.homing_configured) {
                bool cur_active = motor.homing_active_high ? (gpio_get(motor.homing_pin) != 0) : (gpio_get(motor.homing_pin) == 0);
                gpio_put(LED_PIN, cur_active ? 1 : 0); // mirror sensor level on onboard LED
                // external LED keeps fading separately
                if (cur_active && !prev_active) {
                    zero_offset = motor.current_position;
                    zero_set = true;
                    motor.target_position = motor.current_position;
                    printf("Zero set at physical step %d\n", zero_offset);
                    gpio_put(LED_PIN, 1);
                    sleep_ms(120);
                    gpio_put(LED_PIN, 0);
                    min_pos = (zero_offset - safety_range > 0) ? zero_offset - safety_range : 0;
                    max_pos = (zero_offset + safety_range < FULL_MAX) ? zero_offset + safety_range : FULL_MAX;
                }
                prev_active = cur_active;
            }
            sleep_ms(1);
        }
        x27_sleep(&motor);
    }

    return 0;
}
