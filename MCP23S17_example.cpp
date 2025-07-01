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
#include "internal/MCP23S17.h"
#include "internal/Switches.h" // Include the modified Switches.h
#include "internal/Encoders.h" // Include the modified Encoders.h

// SPI Configuration
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

// MCP23S17 Configuration
#define MCP23S17_ADDRESS 0x20 // Hardware address (A2=A1=A0=0)

    // MCP23S17 instance (global declaration is fine)
    MCP23S17 ioExpander(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, MCP23S17_ADDRESS);

int main()
{
    stdio_init_all(); // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(1000); // Allow USB to settle

    uint8_t boardAddress = 0xF; // USB mode, set manually. See FoxConfig.h for details.

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

    // *** IMPORTANT: Initialize MCP23S17 BEFORE creating DCS-BIOS objects that use it ***
    printf("Initializing MCP23S17...\n");
    ioExpander.begin();

    // Configure MCP23S17 pins as inputs with pull-ups
    ioExpander.pinMode(0, INPUT_PULLUP);
    ioExpander.pinMode(1, INPUT_PULLUP);
    ioExpander.pinMode(2, INPUT_PULLUP);
    ioExpander.pinMode(3, INPUT_PULLUP);
    ioExpander.pinMode(4, INPUT_PULLUP);
    ioExpander.pinMode(5, INPUT_PULLUP);
    printf("MCP23S17 initialized, GPA0-5 configured as inputs with pull-up\n");
    // *** END OF IMPORTANT INITIALIZATION SECTION ***


    // Launch core1 for DCS-BIOS RS485/USB task
    multicore_launch_core1(DcsBios::core1_task);

    // DCS-BIOS init
    DcsBios::setup();
    printf("DCS-BIOS setup complete\n");

    // Now, create DCS-BIOS components, as ioExpander is fully initialized
    DcsBios::MCP23S17Switch2Pos pltPitotHeat("PLT_PITOT_HEAT", &ioExpander, 0);
    DcsBios::MCP23S17RotaryEncoder pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", "-3200", "+3200", &ioExpander, 1, 2);

    // Example for a 3-position switch using MCP23S17 on GPA3, GPA4, GPA5
    const uint8_t multiPosSwitchPins[] = {3, 4, 5};
    DcsBios::MCP23S17Switch3Pos pltExtLightAntiColl("PLT_EXT_LIGHT_ANTI_COLL", &ioExpander, multiPosSwitchPins, 50);

    DcsBios::Switch2Pos pltGearLever("PLT_GEAR_LEVER", 3);

    while (true)
    {
        DcsBios::loop();                          // Main DCS-BIOS handler
        pltPitotHeat.pollInput();                 // Poll the MCP23S17 switch
        pltIntLightConsoleBrightness.pollInput(); // Poll the MCP23S17 rotary encoder
        pltExtLightAntiColl.pollInput(); // Poll the MCP23S17 3-position switch
        pltGearLever.pollInput();                 // Poll the 2-position switch
        DcsBios::updateHeartbeat(); // Blink status LED (if connected)
        sleep_us(10);               // Slight delay
    }
}