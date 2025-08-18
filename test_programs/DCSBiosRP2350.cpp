#ifndef PICO_BOARD
#define PICO_BOARD
#endif

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
#define CONSOLE_LIGHTING_LED 25     // Console LED on GPIO 25
#define CONSOLE_DIMMING_DEADZONE 0  // Deadzone for brightness changes

unsigned int lastBrightness = 0; // Last brightness value for console lighting

// DCS-BIOS input callback for console brightness LED
void onPltIntLightConsoleBrightnessChange(unsigned int newValue)
{
    if (abs((int)newValue - (int)lastBrightness) >= CONSOLE_DIMMING_DEADZONE)
    {
        lastBrightness = newValue;
        uint16_t pwmVal = newValue >> 8; // Map 0–65535 to 0–255
        pwm_set_gpio_level(CONSOLE_LIGHTING_LED, pwmVal);
    }
}
DcsBios::IntegerBuffer pltIntLightConsoleBrightnessBuffer(0x2d6e, 0xffff, 0, onPltIntLightConsoleBrightnessChange);
// DCS-BIOS potentiometer input from ADC0 to DCS
//DcsBios::PotentiometerEWMA<POLL_EVERY_TIME, 1024> pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", CONSOLE_LIGHTING_POT_PIN);
const uint8_t pltIntLightFloodWhiteTogglePins[2] = {14, 15};
DcsBios::SwitchMultiPos<2> pltIntLightFloodWhiteToggle("PLT_INT_LIGHT_FLOOD_WHITE_TOGGLE", pltIntLightFloodWhiteTogglePins);

int main() {
    stdio_init_all();  // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
    sleep_ms(1000);   // Wait for USB CDC to be ready

    
    uint8_t boardAddress = 0xF; // USB mode, set manually. See FoxConfig.h for details.

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    printf("Board address: 0x%X\n", boardAddress);

    switch (board.mode) {
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

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);  
    printf("Core 1 task launched!\n");

    DcsBios::Switch2Pos masterArmSw("MASTER_ARM_SW", 15);

    DcsBios::setup();  // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true) {
        DcsBios::loop(); // Handle input, output, and LED updates
        masterArmSw.pollInput(); // Poll the master arm switch
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}
