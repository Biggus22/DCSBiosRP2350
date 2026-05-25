/**
 * WS2812 LED String Example - F-4E Console Brightness Reactive
 * 
 * This example reads the F-4E console brightness command from DCS-Bios
 * and outputs a string of RGB LEDs that change color based on the
 * commanded brightness level.
 * 
 * Brightness Mapping (0-65535 from DCS-Bios):
 *   0       - Dim/Off (black)
 *   1-16383 - Dim Blue (deep blue → blue)
 *   16384-32767 - Medium Cyan (blue → cyan)
 *   32768-49151 - Bright White (cyan → white)
 *   49152-65535 - Intense White (white → bright white)
 * 
 * Hardware:
 *   - WS2812 LED string connected to GPIO 14 (PIO0, SM0)
 *   - Recommended: 18 LEDs for cockpit panel illumination
 *   - Power: 5V supply with common ground
 * 
 * F-4E Addresses:
 *   - PLT Console Brightness: 0x02D6E
 *   - WSO Console Brightness: 0x02D92
 */

#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdio.h>
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#include "DcsBios.h"
#include "internal/ws2812.h"

// ============================================================================
// Configuration
// ============================================================================

// WS2812 LED strip configuration
#define LED_DATA_PIN        14      // GPIO pin for WS2812 data
#define NUM_LEDS            18      // Number of LEDs in the string
#define LED_BRIGHTNESS      200     // Overall brightness (0-255)

// Use PIO0 for WS2812 (more PIO channels available)
#define LED_PIO             pio0
#define LED_SM_CHANNEL      0

// RS485 Configuration
#define RS485_BAUD          250000
#define RS485_SLAVE_ADDRESS 0x01
#define UART_TX_PIN         8
#define UART_RX_PIN         9
#define UART_EN_PIN         10

// ============================================================================
// LED Color Helpers
// ============================================================================

/**
 * Convert brightness value (0-65535) to LED color segments.
 * 
 * The LED string is divided into segments that each represent a different
 * color response to the brightness:
 *   - Segment 0-5:  Main indicator (brightness-reactive)
 *   - Segment 6-11: Accent strip (brightness-reactive, offset color)
 *   - Segment 12-17: Panel wash (uniform, brightness-scaled)
 */
static void brightnessToColor(uint16_t brightness, uint32_t* colors, uint16_t numColors) {
    // Normalize brightness to 0.0-1.0 range
    float norm = (float)brightness / 65535.0f;
    
    // Clamp to avoid overflow
    if (norm > 1.0f) norm = 1.0f;
    if (norm < 0.0f) norm = 0.0f;
    
    // Apply gamma correction for more natural appearance
    norm = powf(norm, 0.8f);
    
    // Scale by overall brightness setting
    float scaled = norm * ((float)LED_BRIGHTNESS / 255.0f);
    
    // Determine color based on brightness level
    uint8_t r, g, b;
    
    if (brightness == 0) {
        // Completely off
        r = 0; g = 0; b = 0;
    } else if (brightness < 8192) {
        // Very dim: Warm amber glow (typical of dimmed cockpit lighting)
        float t = (float)brightness / 8192.0f;
        r = (uint8_t)(255 * t);
        g = (uint8_t)(140 * t);
        b = (uint8_t)(30 * t);
    } else if (brightness < 16384) {
        // Low: Warm white transition
        float t = (float)(brightness - 8192) / 8192.0f;
        r = (uint8_t)(255);
        g = (uint8_t)(140 + 80 * t);
        b = (uint8_t)(30 + 50 * t);
    } else if (brightness < 32768) {
        // Medium: Neutral white
        float t = (float)(brightness - 16384) / 16384.0f;
        r = (uint8_t)(255);
        g = (uint8_t)(220 + 35 * t);
        b = (uint8_t)(80 + 100 * t);
    } else if (brightness < 49152) {
        // Bright: Cool white
        float t = (float)(brightness - 32768) / 16384.0f;
        r = (uint8_t)(255);
        g = (uint8_t)(255);
        b = (uint8_t)(180 + 75 * t);
    } else {
        // Maximum: Bright white with slight blue tint (night vision compatible)
        float t = (float)(brightness - 49152) / 16384.0f;
        r = (uint8_t)(255);
        g = (uint8_t)(255);
        b = (uint8_t)(255);
    }
    
    // Fill all LED colors
    for (uint16_t i = 0; i < numColors; i++) {
        colors[i] = testLeds.rgbw(r, g, b, 0);
    }
    
    // Add subtle variation for different segments
    // Segment 0-5: Add slight warm tint
    for (uint16_t i = 0; i < 6 && i < numColors; i++) {
        uint8_t wr = (uint8_t)(r * 0.97f);
        uint8_t wg = (uint8_t)(g * 1.0f);
        uint8_t wb = (uint8_t)(b * 0.9f);
        colors[i] = testLeds.rgbw(wr, wg, wb, 0);
    }
    
    // Segment 6-11: Add slight cool tint
    for (uint16_t i = 6; i < 12 && i < numColors; i++) {
        uint8_t cr = (uint8_t)(r * 0.95f);
        uint8_t cg = (uint8_t)(g * 0.98f);
        uint8_t cb = (uint8_t)(b * 1.02f);
        colors[i] = testLeds.rgbw(cr, cg, cb, 0);
    }
    
    // Segment 12-17: Uniform wash (full brightness)
    for (uint16_t i = 12; i < 18 && i < numColors; i++) {
        colors[i] = testLeds.rgbw(r, g, b, 0);
    }
}

// ============================================================================
// WS2812 Instance (global for callback access)
// ============================================================================

static WS2812 testLeds(LED_PIO, LED_SM_CHANNEL, LED_DATA_PIN, false);

// ============================================================================
// DCS-Bios Callbacks
// ============================================================================

/**
 * Callback for F-4E Pilot Console Brightness
 * Address: 0x02D6E, Mask: 0xFFFF, Shift: 0
 */
static uint16_t currentPltBrightness = 0;
static bool brightnessUpdated = false;

void onPltConsoleBrightnessChange(unsigned int newValue) {
    currentPltBrightness = (uint16_t)(newValue & 0xFFFF);
    brightnessUpdated = true;
}
DcsBios::IntegerBuffer pltConsoleBrightnessBuffer(0x02D6E, 0xFFFF, 0, onPltConsoleBrightnessChange);

/**
 * Callback for F-4E WSO Console Brightness
 * Address: 0x02D92, Mask: 0xFFFF, Shift: 0
 */
static uint16_t currentWsoBrightness = 0;

void onWsoConsoleBrightnessChange(unsigned int newValue) {
    currentWsoBrightness = (uint32_t)(newValue & 0xFFFF);
    // Can use this for separate WSO panel LEDs if needed
}
DcsBios::IntegerBuffer wsoConsoleBrightnessBuffer(0x02D92, 0xFFFF, 0, onWsoConsoleBrightnessChange);

// ============================================================================
// Main
// ============================================================================

int main() {
    // Initialize stdio
    stdio_init_all();
    
    // Brief delay for USB CDC connection
    sleep_ms(1500);
    
    printf("\n========================================\n");
    printf("  WS2812 Console Brightness Example\n");
    printf("  F-4E-45MC Panel Illumination\n");
    printf("========================================\n\n");
    
    printf("LED Configuration:\n");
    printf("  Pin: GPIO %d (PIO0, SM%d)\n", LED_DATA_PIN, LED_SM_CHANNEL);
    printf("  LEDs: %d\n", NUM_LEDS);
    printf("  Brightness: %d/255\n", LED_BRIGHTNESS);
    printf("\n");
    
    printf("DCS-Bios Brightness Sources:\n");
    printf("  PLT Console: 0x%04X\n", 0x02D6E);
    printf("  WSO Console: 0x%04X\n", 0x02D92);
    printf("\n");
    
    // Initialize WS2812 LED strip
    testLeds.begin(NUM_LEDS);
    testLeds.setBrightness(LED_BRIGHTNESS);
    
    // Initial LED state (off)
    for (int i = 0; i < NUM_LEDS; i++) {
        testLeds.setPixel(i, testLeds.rgbw(0, 0, 0, 0));
    }
    testLeds.show();
    
    // Determine board mode and initialize RS485
    DcsBios::BoardMode board = DcsBios::determineBoardMode(RS485_SLAVE_ADDRESS);
    DcsBios::currentBoardMode = board;
    
    if (board.mode != DcsBios::BoardModeType::SLAVE) {
        printf("ERROR: Invalid board mode for address 0x%X. Stopping.\n", RS485_SLAVE_ADDRESS);
        while (true) {
            sleep_ms(100);
        }
    }
    
    printf("RS485 Slave initialized: Address 0x%X, Baud %lu\n", 
           RS485_SLAVE_ADDRESS, (unsigned long)RS485_BAUD);
    
    // Start DCS-Bios on core1
    multicore_launch_core1(DcsBios::core1_task);
    
    // Setup DCS-Bios
    DcsBios::setup();
    
    printf("Starting main loop...\n\n");
    
    // Main loop
    uint32_t lastUpdate = 0;
    const uint32_t LED_UPDATE_INTERVAL = 33;  // ~30 Hz LED updates
    
    while (true) {
        // Process DCS-Bios messages
        DcsBios::loop();
        
        // Update LEDs at regular interval
        uint32_t currentTime = to_ms_since_boot(get_absolute_time());
        if (brightnessUpdated || (currentTime - lastUpdate >= LED_UPDATE_INTERVAL)) {
            if (brightnessUpdated) {
                printf("Brightness: PLT=%u, WSO=%u\n", 
                       currentPltBrightness, currentWsoBrightness);
                brightnessUpdated = false;
            }
            
            // Convert brightness to LED colors
            uint32_t colors[NUM_LEDS];
            brightnessToColor(currentPltBrightness, colors, NUM_LEDS);
            
            // Apply colors to LED strip
            for (int i = 0; i < NUM_LEDS; i++) {
                testLeds.setPixel(i, colors[i]);
            }
            testLeds.show();
            
            lastUpdate = currentTime;
        }
        
        // Small delay to prevent CPU overload
        sleep_us(20);
    }
    
    return 0;
}