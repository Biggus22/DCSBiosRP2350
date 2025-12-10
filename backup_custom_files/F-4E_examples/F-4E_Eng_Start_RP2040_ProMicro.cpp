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

#define NUM_LEDS 14          // Total number of SK6812 LEDs
#define ONBOARD_NEOPIXEL_PIN 16 // Define the pin for the onboard NeoPixel

WS2812 externalLeds(pio0, 0, 16, false); // Global WS2812 object for external NeoPixels on pin 16 (Changed to 24-bit RGB mode)



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
const uint8_t pltEngineMasterLPins[2] = {6, 7};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltEngineMasterL("PLT_ENGINE_MASTER_L", pltEngineMasterLPins);
const uint8_t pltEngineMasterRPins[2] = {14, 15};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltEngineMasterR("PLT_ENGINE_MASTER_R", pltEngineMasterRPins);
//const uint8_t pltEngineStartRPins[2] = {13, 12};
DcsBios::Switch3Pos2Pin pltEngineStart("PLT_ENGINE_START", 13, 12);
//const uint8_t pltControlsRudderTrimPins[2] = {9, 8};
DcsBios::Switch3Pos2Pin pltControlsRudderTrim("PLT_CONTROLS_RUDDER_TRIM", 9, 8);
//const uint8_t pltCadcCorrectionPins[2] = {11, 10};
DcsBios::Switch3Pos2Pin pltCadcCorrection("PLT_CADC_CORRECTION", 11, 10);
DcsBios::Potentiometer pltHudBrightness("PLT_HUD_BRIGHTNESS", 28, true);


// DCS-BIOS F-14A/B FUNCTIONS HERE
// DCS-BIOS callback function for F-14 console lighting (red)
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t brightness = 0;
    
    if (consoleBrightness <= 8) {
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

// Declare the IntegerBuffer for F-14 console lighting
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);

DcsBios::Switch3Pos2Pin pltEngineCrank("PLT_ENGINE_CRANK", 13, 12);

DcsBios::Potentiometer pltHudBright("PLT_HUD_BRIGHT", 28, true);



int main()
{
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready


    // Initialize the WS2812 strip for the power-on flash
    externalLeds.begin(NUM_LEDS); // Initialize with correct number of pixels

    // Power-on Green Flash for 1 second
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Set all pixels to green (R=0, G=255, B=0, W=0)
    }
    externalLeds.show();
    sleep_ms(5000); // Display green for 1 second

    // Clear LEDs after the flash
    externalLeds.clear();
    externalLeds.show();

    uint8_t boardAddress = 0xF;

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
    printf("Minimal testing program running\n");

    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}
