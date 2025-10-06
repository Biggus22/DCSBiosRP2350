#include <stdio.h>
#include "pico/stdio.h"
#include "stdlib.h"
#include "math.h"
#include "internal/ws2812.h"
#include "hardware/pwm.h"
#include "pico/time.h"

#define NUM_LEDS 18     // Total number of LEDs to match your working code

// WS2812 object for 18 LEDs on pin 14
WS2812 testLeds(pio0, 0, 5, false);

// Global variables for effect control
int current_effect = 0; // 0-3 for four different effects
int effect_step = 0;    // Current step in the effect animation
uint32_t last_effect_update = 0;
const uint32_t EFFECT_UPDATE_INTERVAL = 50; // Update effects every 50ms
const uint32_t EFFECT_DURATION = 5000;      // Change effect every 5 seconds
uint32_t last_effect_change = 0;

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

// Function to print current effect
void print_effect() {
    printf("Effect: %d ", current_effect);
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
    
    printf("Automatic LED Test Program Starting...\n");
    printf("Testing %d NeoPixels on pin 14\n", NUM_LEDS);
    printf("Program will automatically cycle through all effects\n");
    printf("=============================\n");
    
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
    
    printf("Ready! Cycling through effects automatically.\n");
    print_effect();
    
    last_effect_change = to_ms_since_boot(get_absolute_time());
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Change effect every EFFECT_DURATION milliseconds
        if (current_time - last_effect_change >= EFFECT_DURATION) {
            current_effect = (current_effect + 1) % 4; // Cycle through 0-3
            effect_step = 0; // Reset effect animation
            last_effect_change = current_time;
            print_effect();
        }
        
        // Update effects
        if (current_time - last_effect_update >= EFFECT_UPDATE_INTERVAL) {
            update_effect();
            last_effect_update = current_time;
        }
        
        sleep_ms(10); // Small delay to prevent excessive polling
    }
    
    return 0;
}