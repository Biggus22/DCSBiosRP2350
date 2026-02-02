#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <pico/multicore.h>
#include "../../src/DcsBios.h"
#include "../../src/internal/FoxConfig.h"
#include "../../src/internal/Leds.h"
#include "../../src/internal/Switches.h"
#include "../../src/internal/heartbeat.h"
#include "../../src/internal/DeviceAddress.h"
#include "../../src/internal/BoardMode.h"
#include "../../src/internal/rs485.h"
#include "../../src/internal/ws2812.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"



#if PICO_SDK_AVAILABLE

#define NUM_LEDS 17          // Total number of SK6812 LEDs
#define ANTI_SKID_LED_INDEX (NUM_LEDS - 1)

WS2812 externalLeds(pio0, 0, 0, false); // Global WS2812 object for external NeoPixels on pin 0

static bool g_antiSkidInop = false;

static void applyAntiSkidLed() {
    if (g_antiSkidInop) {
        // Amber warning indicator
        externalLeds.setPixel(ANTI_SKID_LED_INDEX, externalLeds.rgbw(255, 96, 0, 0));
    } else {
        // Off when not in warning state
        externalLeds.setPixel(ANTI_SKID_LED_INDEX, externalLeds.rgbw(0, 0, 0, 0));
    }
    externalLeds.show();
}

//uart_inst_t *rs485_uart = uart0;


// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);

    for (int i = 0; i < ANTI_SKID_LED_INDEX; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0)); // Red for F-4
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// DCS-BIOS callback function for F-14 console lighting (red)
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t brightness = 0;
    
    if (consoleBrightness <= 8) {
        uint8_t percentages[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};
        brightness = (percentages[consoleBrightness] * 255) / 100;
    } else {
        brightness = (uint8_t)((consoleBrightness * 255) / 65535);
    }
    
    for (int i = 0; i < ANTI_SKID_LED_INDEX; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0)); // Red for F-14
    }
    externalLeds.show();
}

// Declare the IntegerBuffer for F-14 console lighting
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);

// Anti-skid INOP warning for the last LED in the string
void onPltGearAntiSkidInopChange(unsigned int newValue) {
    g_antiSkidInop = (newValue != 0);
    applyAntiSkidLed();
}
DcsBios::IntegerBuffer pltGearAntiSkidInopBuffer(0x2ac0, 0x2000, 13, onPltGearAntiSkidInopChange);

// Servo output for oxygen flow valve - using original Arduino DCS-BIOS values
DcsBios::ServoOutput pltO2Flow(0x2b32, 9, 620, 2800);
DcsBios::ServoOutput pltO2Liters(0x2b36, 11, 544, 2400);
DcsBios::ServoOutput pltO2Pressure(0x2b34, 13, 544, 2400);

// O2 mixture switch (2-position) on GPIO pins 12 and 10
const uint8_t pltO2MixturePins[2] = {10, 12};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltO2Mixture("PLT_O2_MIXTURE", pltO2MixturePins);

// O2 supply switch (2-position) on GPIO pin 14
DcsBios::Switch2Pos pltO2Supply("PLT_O2_SUPPLY", 14);

int main()
{
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready

    externalLeds.begin(NUM_LEDS);

    // Power-on Green Flash for 1 second
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Set all pixels to green (R=0, G=255, B=0, W=0)
    }
    externalLeds.show();
    sleep_ms(5000);

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
    //DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

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
#else
int main()
{
    return 0;
}
#endif