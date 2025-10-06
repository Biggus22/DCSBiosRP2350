#ifndef RP2040_BOARD
#define RP2040_BOARD
#endif
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "internal/ws2812.h"

// Uncomment the line below to use analog input for brightness control.
// Comment it out to set brightness to full (255).
//#define USE_ANALOG_BRIGHTNESS

#define NUM_LEDS 10
#define ADC_PIN 27
#define CHASER_DELAY_MS 100

WS2812 externalLeds(pio0, 0, 16, false); // WS2812 object for external NeoPixels on pin 14

// Pin definitions
const uint8_t colorSelectPins[3] = {5, 3, 4};  // PLT_ICS_AMPLIFIER pins for color selection
const uint8_t modePins[2] = {8, 9};            // PLT_ICS_MODE pins for effect control

// Color definitions - No blue channel working, so using red/green combinations
// The WS2812 library on the Pico expects colors in GRBW (Green, Red, Blue, White) order.
struct Color {
    uint8_t g, r, b, w;  // Green, Red, Blue, White order
};

const Color colors[8] = {
    {0, 0, 0, 0},      // 000 - Off
    {0, 255, 0, 0},    // 001 - Red
    {255, 0, 0, 0},    // 010 - Green  
    {255, 255, 0, 0},  // 011 - Yellow
    {128, 255, 0, 0},  // 100 - Orange 
    {0, 0, 0, 255},    // 101 - Blue (using white channel!)
    {64, 255, 0, 0},   // 110 - Red-Orange
    {255, 255, 255, 0} // 111 - White
};

// Global variables
uint8_t currentBrightness = 255;
uint8_t selectedColor = 0;
bool colorChaserMode = false;
bool rainbowChaserMode = false;
int chaserPosition = 0;
int rainbowColorIndex = 0;
absolute_time_t lastChaserUpdate;

void initializeGPIO() {
    // Initialize color select pins as inputs with pull-ups
    for (int i = 0; i < 3; i++) {
        gpio_init(colorSelectPins[i]);
        gpio_set_dir(colorSelectPins[i], GPIO_IN);
        gpio_pull_up(colorSelectPins[i]);
    }
    
    // Initialize mode pins as inputs with pull-ups
    for (int i = 0; i < 2; i++) {
        gpio_init(modePins[i]);
        gpio_set_dir(modePins[i], GPIO_IN);
        gpio_pull_up(modePins[i]);
    }
    
    // Initialize ADC only if the analog brightness feature is enabled
    #ifdef USE_ANALOG_BRIGHTNESS
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(1); // GPIO27 is ADC1
    #endif
}

uint8_t readColorSelect() {
    // Read the individual pins
    bool pin3_active = !gpio_get(colorSelectPins[1]);  // Pin 3
    bool pin5_active = !gpio_get(colorSelectPins[0]);  // Pin 5
    bool pin4_active = !gpio_get(colorSelectPins[2]);  // Pin 4
    
    // Rotary switch logic - treat "no pins active" as blue
    uint8_t value = 0;
    
    // --- START: MODIFICATION TO HARDCODE COLOR TO RED ---
    // Since you can only access pins 8 and 9, we are overriding the color selection logic
    // and forcing the color to be red (value 1) regardless of the state of pins 3, 4, and 5.
    value = 1; 
    // --- END: MODIFICATION ---
    
    // Original logic commented out below:
    /*
    if (!pin3_active && !pin5_active && !pin4_active) {
        // No pins active = Orange (since blue doesn't work)
        value = 4;
    } else if (pin5_active && !pin3_active) {
        // Only pin 5 = Red (position 1) 
        value = 1;
    } else if (pin3_active && !pin5_active) {
        // Only pin 3 = Green (position 2)
        value = 2;
    } else if (pin4_active) {
        // Pin 4 active = Try mixing yellow and green to make blue
        value = 5; // Light Green mix
    } else {
        // Multiple pins or other combinations
        value = 7; // White for unknown states
    }
    */
    
    // Debug output - show every loop to catch intermittent issues
    printf("Live pins: 3=%d 5=%d 4=%d -> Color:%d\n", 
           pin3_active, pin5_active, pin4_active, value);
    
    return value;
}

void readModeSelect(bool &rainbowChaser, bool &colorChaser) {
    bool pin8 = !gpio_get(modePins[0]);  // Pin 8, inverted
    bool pin9 = !gpio_get(modePins[1]);  // Pin 9, inverted
    
    rainbowChaser = pin8 && !pin9;      // Pin 8 on, pin 9 off = rainbow chaser
    colorChaser = !pin8 && pin9;       // Pin 8 off, pin 9 on = selected color chaser
    // Both off or both on = solid color
}

uint8_t readBrightness() {
    #ifdef USE_ANALOG_BRIGHTNESS
    uint16_t adcValue = adc_read();
    // Convert 12-bit ADC (0-4095) to 8-bit brightness (0-255)
    // Reverse the potentiometer direction: 4095 -> 0, 0 -> 255
    return (uint8_t)(((4095 - adcValue) * 255) / 4095);
    #else
    // If analog brightness is not used, return max brightness
    return 255;
    #endif
}

void updateLEDs() {
    if (rainbowChaserMode) {
        // Rainbow chaser - each LED shows a different color cycling through all combinations
        externalLeds.clear();
        for (int i = 0; i < NUM_LEDS; i++) {
            int colorIndex = (rainbowColorIndex + i) % 8; // Cycle through colors 0-7
            if (colorIndex == 0) colorIndex = 1; // Skip "off" color, use red instead
            
            Color currentColor = colors[colorIndex];
            Color scaledColor = {
                (uint8_t)((currentColor.g * currentBrightness) / 255),
                (uint8_t)((currentColor.r * currentBrightness) / 255),
                (uint8_t)((currentColor.b * currentBrightness) / 255),
                (uint8_t)((currentColor.w * currentBrightness) / 255)
            };
            // CORRECTED: Pass green and red values in the correct order for the WS2812 library.
            externalLeds.setPixel(i, externalLeds.rgbw(scaledColor.g, scaledColor.r, scaledColor.b, scaledColor.w));
        }
    } else if (colorChaserMode) {
        // Single color chaser - only one LED on at a time with selected color
        externalLeds.clear();
        Color currentColor = colors[selectedColor];
        Color scaledColor = {
            (uint8_t)((currentColor.g * currentBrightness) / 255),
            (uint8_t)((currentColor.r * currentBrightness) / 255),
            (uint8_t)((currentColor.b * currentBrightness) / 255),
            (uint8_t)((currentColor.w * currentBrightness) / 255)
        };
        // CORRECTED: Pass green and red values in the correct order for the WS2812 library.
        externalLeds.setPixel(chaserPosition, 
                                 externalLeds.rgbw(scaledColor.g, scaledColor.r, scaledColor.b, scaledColor.w));
    } else {
        // Solid color - all LEDs same color
        Color currentColor = colors[selectedColor];
        Color scaledColor = {
            (uint8_t)((currentColor.g * currentBrightness) / 255),
            (uint8_t)((currentColor.r * currentBrightness) / 255),
            (uint8_t)((currentColor.b * currentBrightness) / 255),
            (uint8_t)((currentColor.w * currentBrightness) / 255)
        };
        for (int i = 0; i < NUM_LEDS; i++) {
            // CORRECTED: Pass green and red values in the correct order for the WS2812 library.
            externalLeds.setPixel(i, 
                                     externalLeds.rgbw(scaledColor.g, scaledColor.r, scaledColor.b, scaledColor.w));
        }
    }
    
    externalLeds.show();
}

void updateChaser() {
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(lastChaserUpdate, now) >= (CHASER_DELAY_MS * 1000)) {
        if (rainbowChaserMode) {
            // Rainbow chaser - advance the color pattern
            rainbowColorIndex = (rainbowColorIndex + 1) % 7; // Cycle through colors 1-7 (skip 0=off)
            if (rainbowColorIndex == 0) rainbowColorIndex = 1;
        }
        if (colorChaserMode || rainbowChaserMode) {
            // Advance chaser position for both modes
            chaserPosition = (chaserPosition + 1) % NUM_LEDS;
        }
        lastChaserUpdate = now;
    }
}

void printStatus() {
    static uint8_t lastColor = 255;
    static bool lastRainbowChaser = false;
    static bool lastColorChaser = false;
    static uint8_t lastBrightness = 255;
    
    // Only print when something changes
    if (selectedColor != lastColor || rainbowChaserMode != lastRainbowChaser || 
        colorChaserMode != lastColorChaser || currentBrightness != lastBrightness) {
        
        printf("Color: %d, Rainbow: %s, Chaser: %s, Brightness: %d%%\n", 
               selectedColor, 
               rainbowChaserMode ? "YES" : "NO",
               colorChaserMode ? "YES" : "NO",
               (currentBrightness * 100) / 255);
        
        lastColor = selectedColor;
        lastRainbowChaser = rainbowChaserMode;
        lastColorChaser = colorChaserMode;
        lastBrightness = currentBrightness;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB CDC to be ready
    
    printf("LED Test Program Starting...\n");
    printf("Pin assignments:\n");
    printf("Rotary switch logic:\n");
    printf("  Color Select: Pin 5 (red), Pin 3 (green), Pin 4 (test)\n");
    printf("  Mode: Pin 8 (off), Pin 9 (chaser)\n");
    printf("  Brightness: Pin 27 (potentiometer)\n");
    printf("Rotary switch mapping (Blue channel not working):\n");
    printf("  No pins active = Orange\n");
    printf("  Pin 5 only = Red\n"); 
    printf("  Pin 3 only = Green\n");
    printf("  Pin 4 active = Blue (using white channel)\n");
    printf("  Multiple pins = White\n");
    printf("Mode mapping:\n");
    printf("  Pin 8 ON + Pin 9 OFF = Rainbow Chaser (cycles all colors)\n");
    printf("  Pin 8 OFF + Pin 9 ON = Color Chaser (selected color only)\n");
    printf("  Both OFF or Both ON = Solid Color\n");
    printf("\n");
    
    initializeGPIO();
    externalLeds.begin(NUM_LEDS);
    
    // Initialize chaser timer
    lastChaserUpdate = get_absolute_time();
    
    // Power-on test - quick rainbow
    printf("Running power-on LED test...\n");
    for (int color = 1; color <= 7; color++) {
        Color testColor = colors[color];
        for (int i = 0; i < NUM_LEDS; i++) {
            // CORRECTED: Pass green and red values in the correct order for the WS2812 library.
            externalLeds.setPixel(i, externalLeds.rgbw(testColor.g, testColor.r, testColor.b, testColor.w));
        }
        externalLeds.show();
        sleep_ms(200);
    }
    
    // Clear LEDs
    externalLeds.clear();
    externalLeds.show();
    
    printf("LED test complete. Starting interactive mode...\n");
    
    while (true) {
        // Read inputs
        selectedColor = readColorSelect();
        readModeSelect(rainbowChaserMode, colorChaserMode);
        currentBrightness = readBrightness();
        
        // Update chaser position if needed
        updateChaser();
        
        // Update LEDs
        updateLEDs();
        
        // Print status changes
        printStatus();
        
        // Small delay to avoid overwhelming the system
        sleep_ms(10);
    }
    
    return 0;
}
