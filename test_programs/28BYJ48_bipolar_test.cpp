// Bipolar H-bridge diagnostic test for 28BYJ-48 converted for MX1508
// Pins: GPIO 2..5 -> IN1..IN4 on MX1508 (IN1=GPIO2 ... IN4=GPIO5)

#include "pico/stdlib.h"
#include <stdio.h>

const uint STEP_PINS[4] = {2, 3, 4, 5};

// Bipolar full-step sequence (A+/A-, B+/B- mapped to IN1..IN4)
static const uint8_t SEQ_BIPOLAR[4][4] = {
    {1,0,1,0}, // A+ B+
    {0,1,1,0}, // A- B+
    {0,1,0,1}, // A- B-
    {1,0,0,1}  // A+ B-
};

// Unipolar (ULN2003) full-double sequence for reference
static const uint8_t SEQ_UNIPOLAR_FULL_DOUBLE[4][4] = {
    {1,1,0,0},
    {0,1,1,0},
    {0,0,1,1},
    {1,0,0,1}
};

int main() {
    stdio_init_all();

    for (int i = 0; i < 4; ++i) {
        gpio_init(STEP_PINS[i]);
        gpio_set_dir(STEP_PINS[i], GPIO_OUT);
        gpio_put(STEP_PINS[i], 0);
    }

    printf("28BYJ-48 bipolar H-bridge diagnostic\n");
    printf("Pins: %u %u %u %u\n", STEP_PINS[0], STEP_PINS[1], STEP_PINS[2], STEP_PINS[3]);

    const int32_t sweep_steps = 512; // number of steps per sweep
    uint32_t step_delay_us = 1600;
    const uint32_t dwell_ms = 1000;

    bool use_bipolar = true;
    printf("Commands: 'b' = bipolar, 'u' = unipolar, 't' = toggle\n");
    printf("Additional: '1'..'4' drive individual IN pins, 'a' toggle coil A polarity, 'B' toggle coil B polarity\n");
    printf("'s' = slow step, 'f' = fast step, 'p' = pause/de-energize\n");

    while (true) {
        // forward sweep
        printf("Forward %d steps (%s)\n", sweep_steps, use_bipolar ? "bipolar" : "unipolar");
        int idx = 0;
        for (int32_t s = 0; s < sweep_steps; ++s) {
            // poll serial for runtime commands
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                if (ch == 'b') {
                    use_bipolar = true;
                    printf("Mode: bipolar\n");
                } else if (ch == 'u') {
                    use_bipolar = false;
                    printf("Mode: unipolar\n");
                } else if (ch == 't') {
                    use_bipolar = !use_bipolar;
                    printf("Mode toggled: %s\n", use_bipolar ? "bipolar" : "unipolar");
                }
                else if (ch >= '1' && ch <= '4') {
                    int pin = ch - '1';
                    // drive only selected pin high for observation
                    for (int p = 0; p < 4; ++p) gpio_put(STEP_PINS[p], (p == pin) ? 1 : 0);
                    printf("Driving IN%d high\n", pin+1);
                    continue;
                } else if (ch == 'a') {
                    // toggle coil A: IN1/IN2
                    static int a_state = 0;
                    a_state = (a_state + 1) % 3;
                    if (a_state == 0) { gpio_put(STEP_PINS[0], 0); gpio_put(STEP_PINS[1], 0); printf("Coil A off\n"); }
                    if (a_state == 1) { gpio_put(STEP_PINS[0], 1); gpio_put(STEP_PINS[1], 0); printf("Coil A A+\n"); }
                    if (a_state == 2) { gpio_put(STEP_PINS[0], 0); gpio_put(STEP_PINS[1], 1); printf("Coil A A-\n"); }
                    continue;
                } else if (ch == 'B') {
                    // toggle coil B: IN3/IN4
                    static int b_state = 0;
                    b_state = (b_state + 1) % 3;
                    if (b_state == 0) { gpio_put(STEP_PINS[2], 0); gpio_put(STEP_PINS[3], 0); printf("Coil B off\n"); }
                    if (b_state == 1) { gpio_put(STEP_PINS[2], 1); gpio_put(STEP_PINS[3], 0); printf("Coil B B+\n"); }
                    if (b_state == 2) { gpio_put(STEP_PINS[2], 0); gpio_put(STEP_PINS[3], 1); printf("Coil B B-\n"); }
                    continue;
                } else if (ch == 's') {
                    step_delay_us = 5000; printf("Slow step\n"); continue;
                } else if (ch == 'f') {
                    step_delay_us = 1600; printf("Fast step\n"); continue;
                } else if (ch == 'p') {
                    for (int p = 0; p < 4; ++p) gpio_put(STEP_PINS[p], 0); printf("Pause/de-energize\n"); continue;
                }
            }

            const uint8_t *seq = use_bipolar ? SEQ_BIPOLAR[idx] : SEQ_UNIPOLAR_FULL_DOUBLE[idx];
            for (int p = 0; p < 4; ++p) gpio_put(STEP_PINS[p], seq[p]);
            idx = (idx + 1) % 4;
            busy_wait_us(step_delay_us);
        }
        // de-energize briefly
        for (int p = 0; p < 4; ++p) gpio_put(STEP_PINS[p], 0);
        sleep_ms(dwell_ms);

        // backward sweep
        printf("Backward %d steps (%s)\n", sweep_steps, use_bipolar ? "bipolar" : "unipolar");
        idx = 3;
        for (int32_t s = 0; s < sweep_steps; ++s) {
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                if (ch == 'b') {
                    use_bipolar = true;
                    printf("Mode: bipolar\n");
                } else if (ch == 'u') {
                    use_bipolar = false;
                    printf("Mode: unipolar\n");
                } else if (ch == 't') {
                    use_bipolar = !use_bipolar;
                    printf("Mode toggled: %s\n", use_bipolar ? "bipolar" : "unipolar");
                }
            }

            const uint8_t *seq = use_bipolar ? SEQ_BIPOLAR[idx] : SEQ_UNIPOLAR_FULL_DOUBLE[idx];
            for (int p = 0; p < 4; ++p) gpio_put(STEP_PINS[p], seq[p]);
            idx = (idx - 1 + 4) % 4;
            busy_wait_us(step_delay_us);
        }
        for (int p = 0; p < 4; ++p) gpio_put(STEP_PINS[p], 0);
        sleep_ms(dwell_ms);
    }

    return 0;
}
