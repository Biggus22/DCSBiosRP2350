#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
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
#include "internal/ws2812.h" // Include the WS2812 header
#include "hardware/gpio.h"

#define NUM_LEDS 3          // Total number of SK6812 LEDs
WS2812 externalLeds(pio0, 0, 15, false); // Global WS2812 object for external NeoPixels on pin 1

uart_inst_t *rs485_uart = uart0; // Control UART in main


// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0)); // Red for F-4
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// DCS-BIOS callback function for F-14 console lighting (red)
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    // F-14 console lighting switch has 9 positions (0-8)
    // Convert position to brightness percentage
    uint8_t brightness = 0;

    if (consoleBrightness <= 8) {
        // Position-based values: 0, 13, 25, 38, 50, 63, 75, 88, 100% (positions 0-8)
        uint8_t percentages[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};
        brightness = (percentages[consoleBrightness] * 255) / 100;
    } else {
        brightness = (uint8_t)((consoleBrightness * 255) / 65535);
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0)); // Red for F-14
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);



int main()
{
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);

    externalLeds.begin(NUM_LEDS);

    // Power-on Green Flash for half a second to indicate startup
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Set all pixels to green
    }
    externalLeds.show();
    sleep_ms(500); // Reduced from 5000ms to avoid confusion with actual lighting

    // Clear LEDs after the flash
    externalLeds.clear();
    externalLeds.show();

    uint8_t boardAddress = 0xF;

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);

    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);

    DcsBios::setup();

    while (true)
    {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        sleep_us(10);
    }
}