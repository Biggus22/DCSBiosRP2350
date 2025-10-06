#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "hardware/pwm.h"
#include "internal/ws2812.h"


uart_inst_t* rs485_uart = uart0; // Control UART in main

// Global WS2812 object for RGB LED on pin 25
WS2812 onboardRgbLed(pio0, 0, NEO_DRIVE_PIN);

// Define pin for regular LED (if needed as alternative)
#define REGULAR_LED_PIN 17


// DCS-BIOS callback function for PLT_INT_LIGHT_CONSOLE
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    // newValue is expected to be a value from 0 to 65535 based on DCS-BIOS integer buffers.
    // Scale this value to a brightness from 0 to 255 for the WS2812 LED.
    uint8_t brightness = (uint8_t)((consoleBrightness * 255) / 65535);

    // Set the LED color with brightness control (White color to reflect console lighting)
    // Using RGB format: (Red 255, Green 255, Blue 255) scaled by brightness
    uint8_t scaledRed = (uint8_t)((255 * brightness) / 255);
    uint8_t scaledGreen = (uint8_t)((255 * brightness) / 255);
    uint8_t scaledBlue = (uint8_t)((255 * brightness) / 255);
    
    // Set the RGB LED color (GRB format for WS2812)
    onboardRgbLed.setPixel(0, onboardRgbLed.rgb(scaledRed, scaledGreen, scaledBlue));
    onboardRgbLed.show();

    // Also control regular LED on pin 17 as alternative (for those who want a simple LED)
    // The brightness level determines if it's on (high enough brightness) or off
    bool ledState = (brightness > 64); // Only turn on if brightness is more than 25% (64/255)
    gpio_put(REGULAR_LED_PIN, ledState);
}

// Declare the IntegerBuffer
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);


int main() {
    stdio_init_all();  // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
    sleep_ms(5000);   // Wait for USB CDC to be ready

    // Initialize the regular LED pin
    gpio_init(REGULAR_LED_PIN);
    gpio_set_dir(REGULAR_LED_PIN, GPIO_OUT);
    gpio_put(REGULAR_LED_PIN, 0); // Start with LED off

    // Initialize the WS2812 RGB LED
    if (onboardRgbLed.begin(1)) { // 1 pixel
        printf("WS2812 onboard LED initialized successfully!\n");
        // Set initial color to off
        onboardRgbLed.clear();
        onboardRgbLed.show();
    } else {
        printf("Failed to initialize WS2812 onboard LED!\n");
    }

    uint8_t boardAddress = 0xF;
    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);



    DcsBios::setup();  // Initialize DCS-BIOS framework
        printf("DCS-BIOS setup complete!");
    while (true) {
        DcsBios::loop(); // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}