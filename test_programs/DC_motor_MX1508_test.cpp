// Simple DC motor test for MX1508 driver on Raspberry Pi Pico
// Motor connected to GPIO4 (IN1) and GPIO5 (IN2)
// Behavior: forward 10s, short brake, backward 10s, short brake, repeat

#include "pico/stdlib.h"
#include <stdio.h>

const uint MOTOR_IN1_PIN = 2u; // GPIO2
const uint MOTOR_IN2_PIN = 3u; // GPIO3
const uint32_t RUN_MS = 2000u; // 2 seconds
const uint32_t BRAKE_MS = 2000u; // 2 second brake/pause between direction changes

int main() {
    stdio_init_all();

    // initialize pins
    gpio_init(MOTOR_IN1_PIN);
    gpio_init(MOTOR_IN2_PIN);
    gpio_set_dir(MOTOR_IN1_PIN, GPIO_OUT);
    gpio_set_dir(MOTOR_IN2_PIN, GPIO_OUT);

    printf("DC Motor MX1508 test starting\n");
    printf("IN1=%u IN2=%u\n", MOTOR_IN1_PIN, MOTOR_IN2_PIN);

    while (true) {
        // Forward: IN1=1 IN2=0
        printf("Forward for %u ms\n", RUN_MS);
        gpio_put(MOTOR_IN1_PIN, 1);
        gpio_put(MOTOR_IN2_PIN, 0);
        sleep_ms(RUN_MS);

        // Brake / coast briefly
        printf("Brake %u ms\n", BRAKE_MS);
        gpio_put(MOTOR_IN1_PIN, 0);
        gpio_put(MOTOR_IN2_PIN, 0);
        sleep_ms(BRAKE_MS);

        // Reverse: IN1=0 IN2=1
        printf("Reverse for %u ms\n", RUN_MS);
        gpio_put(MOTOR_IN1_PIN, 0);
        gpio_put(MOTOR_IN2_PIN, 1);
        sleep_ms(RUN_MS);

        // Brake / coast briefly
        printf("Brake %u ms\n", BRAKE_MS);
        gpio_put(MOTOR_IN1_PIN, 0);
        gpio_put(MOTOR_IN2_PIN, 0);
        sleep_ms(BRAKE_MS);
    }

    return 0;
}
