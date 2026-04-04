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
#define NUM_LEDS 30 // Total number of SK6812 LEDs
// Last two WS2812 pixels reserved for indicator LEDs (1-based LED 29 -> FLARES, LED 30 -> CMS)
// NOTE: FLARES shows BLUE while CMS shows GREEN (temporary mapping)
const uint8_t FLARES_PIXEL = NUM_LEDS - 2; // LED 29 (blue)
const uint8_t CMS_PIXEL = NUM_LEDS - 1;    // LED 30 (green)

WS2812 externalLeds(pio0, 0, 3, false); // Global WS2812 object for external NeoPixels on pin 3

uart_inst_t *rs485_uart = uart0;

// DCS-BIOS F-4E INPUT FUNCTIONS HERE
//const uint8_t pltFuelAirRefuelPins[2] = {7, 8};
DcsBios::Switch2PosT<> pltFuelAirRefuel("PLT_FUEL_AIR_REFUEL", 8, false);
DcsBios::Switch3PosMakeBeforeBreak pltFuelExternalTanksFeed("PLT_FUEL_EXTERNAL_TANKS_FEED", 28, 29);
DcsBios::SwitchWithCover2PosT<> pltFuelRefuelSelector("PLT_FUEL_REFUEL_SELECTOR", "PLT_FUEL_REFUEL_SELECTOR_COVER", 14);

//const uint8_t pltFuelWingFuelDumpPins[2] = {9, 10};
DcsBios::Switch2PosT<> pltFuelWingFuelDump("PLT_FUEL_WING_FUEL_DUMP", 9, true);

DcsBios::Switch2PosT<> pltFuelBoostPumpLCheck("PLT_FUEL_BOOST_PUMP_L_CHECK", 6);
DcsBios::Switch2PosT<> pltFuelBoostPumpRCheck("PLT_FUEL_BOOST_PUMP_R_CHECK", 5);

DcsBios::Switch2PosT<> pltFuelWingInternalFeed("PLT_FUEL_WING_INTERNAL_FEED", 26);

DcsBios::Switch2Pos pltCmFlareNormal("PLT_CM_FLARE_NORMAL", 11, true);

void onPltCmFlareLightChange(unsigned int newValue) {
    // Debug: show received value and update indicator
    printf("onPltCmFlareLightChange: %u\n", newValue);

    // FLARES indicator -> show BLUE when DCS-BIOS reports flares active (non-zero value)
    if (newValue > 0) {
        externalLeds.setPixel(FLARES_PIXEL, externalLeds.rgbw(0, 0, 201, 0)); // blue
    } else {
        externalLeds.setPixel(FLARES_PIXEL, externalLeds.rgbw(0, 0, 0, 0)); // off
    }
    externalLeds.show();
}

DcsBios::IntegerBuffer pltCmFlareLightBuffer(F_4E_PLT_CM_FLARE_LIGHT, onPltCmFlareLightChange);

void onPltCmOnLightChange(unsigned int newValue) {
    // Debug: show received value and update indicator
    printf("onPltCmOnLightChange: %u\n", newValue);

    // CMS indicator -> show GREEN when non-zero (temporary mapping)
    if (newValue > 0) {
        externalLeds.setPixel(CMS_PIXEL, externalLeds.rgbw(0, 201, 0, 0)); // green
    } else {
        externalLeds.setPixel(CMS_PIXEL, externalLeds.rgbw(0, 0, 0, 0)); // off
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltCmOnLightBuffer(F_4E_PLT_CM_ON_LIGHT, onPltCmOnLightChange);

// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    
    // Update console pixels only; leave last two indicator pixels unchanged
    for (int i = 0; i < NUM_LEDS - 2; i++) {
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

    for (int i = 0; i < NUM_LEDS - 2; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0)); // Red for F-14
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);

int main()
{
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready
    externalLeds.begin(NUM_LEDS); // Initialize with 10 pixels

    // Indicator LEDs are handled by the last two WS2812 pixels (FLARES_PIXEL, CMS_PIXEL)

    // Power-on Green Flash for 1 second (indicator pixels show their own colors)
    for (int i = 0; i < NUM_LEDS - 2; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Set console pixels to green
    }
    // Set indicator pixels explicitly: FLARES (blue), CMS (green)
    externalLeds.setPixel(FLARES_PIXEL, externalLeds.rgbw(0, 0, 201, 0));
    externalLeds.setPixel(CMS_PIXEL, externalLeds.rgbw(0, 201, 0, 0));
    externalLeds.show();
    sleep_ms(2000); // Display for 2 seconds

    // Clear LEDs after the flash
    externalLeds.clear();
    externalLeds.show();

    uint8_t boardAddress = 0xF; // Keep as 0xF for now, as the user is debugging buffer overflow on slave
                                // This will be changed to SLAVE_MODE_MIN in a later step if needed.

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    printf("Board address: 0x%X\n", boardAddress);

    switch (board.mode)
    {
    case DcsBios::BoardModeType::HOST:
        printf("HOST MODE\n");
        break;
    case DcsBios::BoardModeType::SLAVE:
        printf("SLAVE MODE\n");
        break;
    case DcsBios::BoardModeType::USB_ONLY:
        printf("STANDALONE USB MODE\n");
        break;
    case DcsBios::BoardModeType::RS485_TERMINAL:
        printf("RS485 TERMINAL MODE\n");
        break;
    default:
        printf("INVALID ADDRESS\n");
        break;
    }
    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");

    DcsBios::setup(); // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}
