#include <stdio.h>
#include "pico/stdio.h"
#include "stdlib.h"
#include "math.h"
#include "internal/ws2812.h"
#include "hardware/pwm.h"
#include "pico/time.h"

#define NUM_LEDS 18     // Total number of LEDs to match your working code

// GPIO pin definitions for toggle switches
#define POWER_SWITCH_PIN1 28    // Power switch position 1
#define POWER_SWITCH_PIN2 29    // Power switch position 2
#define EFFECT_SWITCH1_PIN1 13  // Effect switch 1 position 1
#define EFFECT_SWITCH1_PIN2 15  // Effect switch 1 position 2
#define EFFECT_SWITCH2_PIN1 26  // Effect switch 2 position 1
#define EFFECT_SWITCH2_PIN2 27  // Effect switch 2 position 2

// WS2812 object for 18 LEDs on pin 14
WS2812 testLeds(pio0, 0, 14, false);

// Global variables for switch states
bool leds_enabled = false;
int current_effect = 0; // 0-3 for four different effects
int effect_step = 0;    // Current step in the effect animation
uint32_t last_effect_update = 0;
const uint32_t EFFECT_UPDATE_INTERVAL = 50; // Update effects every 50ms

// Function to setup GPIO pins for switches
void setup_switches() {
    // Setup power switch pins
    gpio_init(POWER_SWITCH_PIN1);
    gpio_set_dir(POWER_SWITCH_PIN1, GPIO_IN);
    gpio_pull_up(POWER_SWITCH_PIN1);
    
    gpio_init(POWER_SWITCH_PIN2);
    gpio_set_dir(POWER_SWITCH_PIN2, GPIO_IN);
    gpio_pull_up(POWER_SWITCH_PIN2);
    
    // Setup effect switch 1 pins
    gpio_init(EFFECT_SWITCH1_PIN1);
    gpio_set_dir(EFFECT_SWITCH1_PIN1, GPIO_IN);
    gpio_pull_up(EFFECT_SWITCH1_PIN1);
    
    gpio_init(EFFECT_SWITCH1_PIN2);
    gpio_set_dir(EFFECT_SWITCH1_PIN2, GPIO_IN);
    gpio_pull_up(EFFECT_SWITCH1_PIN2);
    
    // Setup effect switch 2 pins
    gpio_init(EFFECT_SWITCH2_PIN1);
    gpio_set_dir(EFFECT_SWITCH2_PIN1, GPIO_IN);
    gpio_pull_up(EFFECT_SWITCH2_PIN1);
    
    gpio_init(EFFECT_SWITCH2_PIN2);
    gpio_set_dir(EFFECT_SWITCH2_PIN2, GPIO_IN);
    gpio_pull_up(EFFECT_SWITCH2_PIN2);
}

// Function to read switch states
bool read_power_switch() {
    // Return true if switch is in "on" position (PIN1 is low, PIN2 is high)
    return (!gpio_get(POWER_SWITCH_PIN1) && gpio_get(POWER_SWITCH_PIN2));
}

int read_effect_switches() {
    bool switch1_pos = (!gpio_get(EFFECT_SWITCH1_PIN1) && gpio_get(EFFECT_SWITCH1_PIN2));
    bool switch2_pos = (!gpio_get(EFFECT_SWITCH2_PIN1) && gpio_get(EFFECT_SWITCH2_PIN2));
    
    // Combine two switches to get 4 different effects (0-3)
    return (switch2_pos ? 2 : 0) + (switch1_pos ? 1 : 0);
}

// Function to clear all LEDs
void clearAllLeds() {
    testLeds.clear();
    testLeds.show();
}

// Effect 0: Solid Green (like your working code)
void effect_solid_green() {
    for (int i = 0; i < NUM_LEDS; i++) {
        testLeds.setPixel(i, testLeds.rgbw(0, 201, 0, 0));
    }
    testLeds.show();
}

// Effect 1: Rainbow cycle
void effect_rainbow() {
    for (int i = 0; i < NUM_LEDS; i++) {
        int hue = (effect_step + (i * 20)) % 360;
        uint8_t r, g, b;
        
        if (hue < 60) {
            r = 255; g = (hue * 255) / 60; b = 0;
        } else if (hue < 120) {
            r = 255 - ((hue - 60) * 255) / 60; g = 255; b = 0;
        } else if (hue < 180) {
            r = 0; g = 255; b = ((hue - 120) * 255) / 60;
        } else if (hue < 240) {
            r = 0; g = 255 - ((hue - 180) * 255) / 60; b = 255;
        } else if (hue < 300) {
            r = ((hue - 240) * 255) / 60; g = 0; b = 255;
        } else {
            r = 255; g = 0; b = 255 - ((hue - 300) * 255) / 60;
        }
        
        testLeds.setPixel(i, testLeds.rgbw(r, g, b, 0));
    }
    testLeds.show();
    effect_step = (effect_step + 5) % 360;
}

// Effect 2: Chaser effect
void effect_chaser() {
    clearAllLeds();
    
    // Create a moving dot
    int pos = effect_step % (NUM_LEDS * 2);
    if (pos < NUM_LEDS) {
        // Forward direction
        testLeds.setPixel(pos, testLeds.rgbw(0, 0, 255, 0)); // Blue
        // Add trailing dots
        if (pos > 0) testLeds.setPixel(pos - 1, testLeds.rgbw(0, 0, 100, 0));
        if (pos > 1) testLeds.setPixel(pos - 2, testLeds.rgbw(0, 0, 50, 0));
    } else {
        // Backward direction
        int back_pos = NUM_LEDS - 1 - (pos - NUM_LEDS);
        testLeds.setPixel(back_pos, testLeds.rgbw(255, 0, 0, 0)); // Red
        // Add trailing dots
        if (back_pos < NUM_LEDS - 1) testLeds.setPixel(back_pos + 1, testLeds.rgbw(100, 0, 0, 0));
        if (back_pos < NUM_LEDS - 2) testLeds.setPixel(back_pos + 2, testLeds.rgbw(50, 0, 0, 0));
    }
    
    testLeds.show();
    effect_step++;
}

// Effect 3: Breathing effect
void effect_breathing() {
    // Calculate brightness using sine wave
    float brightness_factor = (sin(effect_step * 0.1) + 1.0) / 2.0; // 0.0 to 1.0
    uint8_t brightness = (uint8_t)(brightness_factor * 255);
    
    // Apply breathing effect to all LEDs in purple
    for (int i = 0; i < NUM_LEDS; i++) {
        uint8_t r = (uint8_t)(128 * brightness_factor);  // Purple R component
        uint8_t g = 0;                                   // Purple G component
        uint8_t b = (uint8_t)(128 * brightness_factor);  // Purple B component
        testLeds.setPixel(i, testLeds.rgbw(r, g, b, 0));
    }
    testLeds.show();
    effect_step++;
}

// Function to update the current effect
void update_effect() {
    switch (current_effect) {
        case 0:
            effect_solid_green();
            break;
        case 1:
            effect_rainbow();
            break;
        case 2:
            effect_chaser();
            break;
        case 3:
            effect_breathing();
            break;
        default:
            effect_solid_green();
            break;
    }
}

// Function to print current status
void print_status() {
    printf("Power: %s, Effect: %d ", leds_enabled ? "ON" : "OFF", current_effect);
    switch (current_effect) {
        case 0: printf("(Solid Green)\n"); break;
        case 1: printf("(Rainbow)\n"); break;
        case 2: printf("(Chaser)\n"); break;
        case 3: printf("(Breathing)\n"); break;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB CDC to be ready
    
    printf("Interactive LED Test Program Starting...\n");
    printf("Testing %d NeoPixels on pin 14\n", NUM_LEDS);
    printf("Switch Controls:\n");
    printf("- GPIO 28/29: Power ON/OFF\n");
    printf("- GPIO 13/15: Effect Select 1\n");
    printf("- GPIO 26/27: Effect Select 2\n");
    printf("=============================\n");
    
    // Initialize switches
    setup_switches();
    
    // Initialize the LED strip
    testLeds.begin(NUM_LEDS);
    testLeds.setBrightness(255);
    
    // Power-on test flash
    printf("Power-on flash test...\n");
    for (int i = 0; i < NUM_LEDS; i++) {
        testLeds.setPixel(i, testLeds.rgbw(201, 0, 0, 0)); // Red flash
    }
    testLeds.show();
    sleep_ms(1000);
    clearAllLeds();
    
    bool last_power_state = false;
    int last_effect = -1;
    
    printf("Ready! Use switches to control LEDs.\n");
    
    while (true) {
        // Read switch states
        bool power_state = read_power_switch();
        int effect_selection = read_effect_switches();
        
        // Check if power state changed
        if (power_state != last_power_state) {
            leds_enabled = power_state;
            last_power_state = power_state;
            
            if (!leds_enabled) {
                clearAllLeds();
                printf("LEDs turned OFF\n");
            } else {
                printf("LEDs turned ON\n");
            }
        }
        
        // Check if effect changed
        if (effect_selection != last_effect) {
            current_effect = effect_selection;
            last_effect = effect_selection;
            effect_step = 0; // Reset effect animation
            print_status();
        }
        
        // Update effects if LEDs are enabled
        if (leds_enabled) {
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - last_effect_update >= EFFECT_UPDATE_INTERVAL) {
                update_effect();
                last_effect_update = current_time;
            }
        }
        
        sleep_ms(10); // Small delay to prevent excessive polling
    }
    
    return 0;
}