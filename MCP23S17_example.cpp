#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/MCP23S17.h"
#include "internal/Switches.h"
#include "internal/Encoders.h"

// SPI Configuration
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

// MCP23S17 Configuration
#define MCP23S17_ADDRESS 0x20 // Hardware address (A2=A1=A0=0), VERIFY THIS!

// Raspberry Pi Pico GPIO pin connected to MCP23S17 INT (INTA or INTB)
#define MCP23S17_INT_PIN 20 // Example: Pico GPIO 20 for MCP23S17 interrupt

// Global MCP23S17 object
MCP23S17 ioExpander(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, MCP23S17_ADDRESS);

// Global pointers are used to reference inputs that are using interrupts
// and are polled in the main loop. This allows for easy access to these inputs

// Global flag to signal an MCP23S17 interrupt from the ISR to the main loop
static volatile bool mcpInterruptOccurred = false;

// Interrupt Service Routine (ISR) for MCP23S17
void mcp23s17_interrupt_callback(uint gpio, uint32_t events)
{
    mcpInterruptOccurred = true;
}

int main()
{
    stdio_init_all();                      // Initialize USB serial
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // 2-second delay

    // Initialize MCP23S17
    ioExpander.begin();

    // Configure MCP23S17 Interrupts (ONLY for interrupt-driven pins)
    // Configure interrupt output for the Pico's INT pin to catch *any* enabled MCP23S17 interrupt
    ioExpander.configureInterruptOutput(false, true); // Active-driver, Active-high

    // Configure MCP23S17 pins for the emulated concentric encoder (INTERRUPT-DRIVEN)
    ioExpander.pinMode(0, _INPUT_PULLUP); // Button on GPA0
    ioExpander.pinMode(1, _INPUT_PULLUP); // Encoder A on GPA1
    ioExpander.pinMode(2, _INPUT_PULLUP); // Encoder B on GPA2
    ioExpander.pinMode(3, _INPUT_PULLUP); // Unused pin on GPA3 (optional, can be used for other purposes)

    // Enable interrupts on encoder pins
    ioExpander.enableInterruptPin(0, true); // Button
    ioExpander.enableInterruptPin(1, true); // Encoder A
    ioExpander.enableInterruptPin(2, true); // Encoder B
    ioExpander.enableInterruptPin(3, true); // pitot heat pin

    // Set interrupt compare mode to compare against previous value for encoder pins
    ioExpander.setInterruptControlMode(0, true); // Button
    ioExpander.setInterruptControlMode(1, true); // Encoder A
    ioExpander.setInterruptControlMode(2, true); // Encoder B

    // Configure Pico's GPIO for MCP23S17 interrupt (still needed for the encoder)
    gpio_init(MCP23S17_INT_PIN);
    gpio_set_dir(MCP23S17_INT_PIN, GPIO_IN);
    gpio_pull_up(MCP23S17_INT_PIN);

    // Attach the interrupt callback (still needed for the encoder)
    gpio_set_irq_enabled_with_callback(MCP23S17_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mcp23s17_interrupt_callback);

    // Other DCS-BIOS components
    DcsBios::Switch2Pos pltWpnMasterArm("PLT_WPN_MASTER_ARM", 1);       // Pico native pin
    DcsBios::Switch2Pos pltPitotHeat("PLT_PITOT_HEAT", &ioExpander, 3); // MCP23S17 pin 4, no reverse, 1ms debounce
  //emulated concentric encoder for PLT_HSI_COURSE_SET and PLT_BARO_PRESSURE_KNOB using MCP23S17 and pins 
    DcsBios::EmulatedConcentricEncoderT<POLL_EVERY_TIME, DcsBios::TWO_STEPS_PER_DETENT, DcsBios::ONE_STEP_PER_DETENT> pltNavModeConcentricEncoder("PLT_HSI_COURSE_SET", "-3200", "+3200", "PLT_BARO_PRESSURE_KNOB", "-200", "+200", &ioExpander, 0, 1, 2, false, 20); // PLT_NAV_MODE and PLT_BARO_PRESSURE_KNOB on MCP23S17 pins 1, 2, 3, actve low, 2ms debounce

    // Global pointer to the Emulated Concentric Encoder.
    // This pointer is crucial for accessing the encoder object within the interrupt handling logic.
    DcsBios::EmulatedConcentricEncoderT<
        POLL_EVERY_TIME,
        DcsBios::TWO_STEPS_PER_DETENT,
        DcsBios::ONE_STEP_PER_DETENT> *g_pltNavModeConcentricEncoder = &pltNavModeConcentricEncoder; // Global pointer for easy access

    // Board configuration
    uint8_t boardAddress = 0xF; // USB mode
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

    // Launch core1 for DCS-BIOS RS485/USB task
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");

    // DCS-BIOS setup
    DcsBios::setup();

    // The main loop now handles DCS-BIOS communication and polling tasks.
    while (true)
    {
        DcsBios::loop();
        pltWpnMasterArm.pollInput(); // Pico native pin, conventionally polled
        pltPitotHeat.pollInput();    // MCP23S17 pin 4, conventionally polled
        DcsBios::updateHeartbeat();  // Update heartbeat LED

        // Check if an MCP23S17 interrupt occurred (ONLY for encoder pins)
        if (mcpInterruptOccurred)
        {
            mcpInterruptOccurred = false; // Clear the flag

            // Read the interrupt flags to know which pins caused the interrupt
            uint16_t interruptFlags = ioExpander.getInterruptFlags();

            // Reading INTCAP registers clears the interrupt on the MCP23S17
            uint16_t capturedValues = ioExpander.getInterruptCapturedValues();

            // Process the encoder based on its interrupt flags
            // This block processes interrupts from GPA0 (button), GPA1, GPA2 (encoder)
            if (interruptFlags & ((1 << 0) | (1 << 1) | (1 << 2)))
            {
                if (g_pltNavModeConcentricEncoder)
                {
                    g_pltNavModeConcentricEncoder->pollInput();
                }
            }
            // No check for GPA4 (Pitot Heat) here, as it's conventionally polled
        }
    }

    return 0;
}