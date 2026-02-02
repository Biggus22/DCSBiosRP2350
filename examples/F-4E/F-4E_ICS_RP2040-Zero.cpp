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
#include "hardware/adc.h"
#define NUM_LEDS 13 // Total number of SK6812 LEDs

WS2812 externalLeds(pio0, 0, 14, false); // Global WS2812 object for external NeoPixels on pin 14

uart_inst_t *rs485_uart = uart0;

// DCS-BIOS F-4E INPUT FUNCTIONS HERE
//const uint8_t pltIcsAmplifierPins[3] = {5, 4, 3};
//DcsBios::SyncingSwitchMultiPosT<POLL_EVERY_TIME, 3> pltIcsAmplifier("PLT_ICS_AMPLIFIER", pltIcsAmplifierPins,
//0x2a00, 0x0300, 8, 50);
const uint8_t pltIcsModePins[2] = {8, 9};  // Changed to 2 pins for 3-position switch
DcsBios::Switch3Pos2Pin pltIcsMode("PLT_ICS_MODE", pltIcsModePins[0], pltIcsModePins[1]);

DcsBios::Potentiometer pltIcsIntercomVol("PLT_ICS_INTERCOM_VOL", 27, true, 0, 4095);

// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0)); // Red for F-4
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// DCS-BIOS F-14A/B FUNCTIONS HERE
//const uint8_t pltIcsAmpSelPins[3] = {5, 4, 3};
//DcsBios::SyncingSwitchMultiPosT<POLL_EVERY_TIME, 3> pltIcsAmpSel("PLT_ICS_AMP_SEL", pltIcsAmpSelPins,
//0x1234, 0x0600, 9, 50);
const uint8_t pltIcsFuncSelPins[2] = {8, 9};  // Changed to 2 pins for 3-position switch
DcsBios::Switch3Pos2Pin pltIcsFuncSel("PLT_ICS_FUNC_SEL", pltIcsFuncSelPins[0], pltIcsFuncSelPins[1]);

DcsBios::Potentiometer pltIcsVol("PLT_ICS_VOL", 27, true, 0, 4095);

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
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready
    adc_init();
    adc_gpio_init(27);
    externalLeds.begin(NUM_LEDS); // Initialize with 10 pixels

    // Power-on Green Flash for 1 second
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Set all pixels to green (R=0, G=255, B=0, W=0)
    }
    externalLeds.show();
    sleep_ms(10000); // Display green for 1 second

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
