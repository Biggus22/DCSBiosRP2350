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
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// MCP23S17 Configuration
#define MCP23S17_ADDRESS 0x20  // Hardware address (A2=A1=A0=0), VERIFY THIS!

// Raspberry Pi Pico GPIO pin connected to MCP23S17 INT (INTA or INTB)
// IMPORTANT: Connect the INT pin of your MCP23S17 to this Pico GPIO.
// It is recommended to use a pull-up resistor on the INT line if not already present.
#define MCP23S17_INT_PIN 20 // Example: Pico GPIO 20 for MCP23S17 interrupt

// Global MCP23S17 object
MCP23S17 ioExpander(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, MCP23S17_ADDRESS);

// Global pointers to components (optional, for ISR access)
DcsBios::MCP23S17Switch2Pos* g_pltPitotHeat = nullptr;

// MODIFIED: Update the type of the global pointer to match the encoder's specific template parameters
DcsBios::RotaryEncoderT<POLL_EVERY_TIME, DcsBios::TWO_STEPS_PER_DETENT>* g_pltIntLightConsoleBrightness = nullptr;
// If you used your own typedef like 'MCP23S17RotaryEncoderTwoSteps', you would use that here instead:
 //MCP23S17RotaryEncoderTwoSteps* g_pltIntLightConsoleBrightness = nullptr;

// Add more global pointers here if needed for other interrupt-driven components

// Global flag to signal an MCP23S17 interrupt from the ISR to the main loop
static volatile bool mcpInterruptOccurred = false;

// Interrupt Service Routine (ISR) for MCP23S17
void mcp23s17_interrupt_callback(uint gpio, uint32_t events) {
    // This ISR is triggered when the MCP23S17 INT pin goes low (assuming active-low interrupt)
    // We only need to set a flag here; actual processing happens in the main loop to avoid
    // doing too much in the ISR.
    
    // Set the global flag to signal the main loop to process the interrupt
    mcpInterruptOccurred = true;
}


int main() {
    stdio_init_all(); // Initialize USB serial
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
   sleep_ms(2000); // <--- ADD THIS LINE (2-second delay)

    // Initialize MCP23S17
    ioExpander.begin();

    // Configure MCP23S17 Interrupts (IOCON.INTPOL=0 for active-low, IOCON.ODR=1 for open-drain)
    //ioExpander.configureInterruptOutput(true, false); // Open-drain, Active-low
    ioExpander.configureInterruptOutput(false, true); // Active-driver, Active-high
    //ioExpander.configureInterruptOutput(true, true); // Open-drain, Active-high

    // Configure MCP23S17 pins for inputs with pull-ups and enable interrupts on change
    ioExpander.pinMode(0, _INPUT_PULLUP);    // Pitot Heat on GPA0
    ioExpander.enableInterruptPin(0, true); // Enable interrupt on GPA0 for Pitot Heat

    ioExpander.pinMode(1, _INPUT_PULLUP);    // Encoder A on GPA1
    ioExpander.pinMode(2, _INPUT_PULLUP);    // Encoder B on GPA2
    ioExpander.enableInterruptPin(1, true); // Enable interrupt on GPA1 for Encoder A
    ioExpander.enableInterruptPin(2, true); // Enable interrupt on GPA2 for Encoder B
    
    // Set interrupt compare mode to compare against previous value (INTCON=0)
    // This generates an interrupt on any pin state change.
    ioExpander.setInterruptControlMode(0, true); // For GPA0
    ioExpander.setInterruptControlMode(1, true); // For GPA1
    ioExpander.setInterruptControlMode(2, true); // For GPA2

    // Configure Pico's GPIO for MCP23S17 interrupt
    gpio_init(MCP23S17_INT_PIN);
    gpio_set_dir(MCP23S17_INT_PIN, GPIO_IN);
    gpio_pull_up(MCP23S17_INT_PIN); // Ensure pull-up if MCP23S17 INT is open-drain

    // Attach the interrupt callback to the Pico GPIO
    gpio_set_irq_enabled_with_callback(MCP23S17_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &mcp23s17_interrupt_callback);


    // Initialize DCS-BIOS components that use the MCP23S17
    // Create local instances and assign their addresses to global pointers
    // Use 'static' if you want these objects to have static storage duration
    // so their addresses are valid for the global pointers throughout execution.
    static DcsBios::MCP23S17Switch2Pos local_pltPitotHeat("PLT_PITOT_HEAT", &ioExpander, 0);
    g_pltPitotHeat = &local_pltPitotHeat;

    // MODIFIED: This local variable is already declared with the correct template arguments
static DcsBios::RotaryEncoderT<POLL_EVERY_TIME, DcsBios::TWO_STEPS_PER_DETENT> local_pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", "-3200", "+3200", &ioExpander, 1, 2, 4);    // Create a local instance of the RotaryEncoder with the correct template parameters - the IO expander, pins, debounce delay.
    // This assignment line (which was line 102) will now work correctly
    g_pltIntLightConsoleBrightness = &local_pltIntLightConsoleBrightness;

    // Example for a 3-position switch using MCP23S17 on GPA3, GPA4, GPA5
     //const uint8_t multiPosSwitchPins[] = {3, 4, 5};
     //static DcsBios::MCP23S17Switch3Pos local_gearLever("GEAR_LEVER", &ioExpander, multiPosSwitchPins);
     //g_gearLever = &local_gearLever; // Assign to a global pointer if you make one for it

     DcsBios::Switch2Pos pltWpnMasterArm("PLT_WPN_MASTER_ARM", 3);


    //printf("MCP23S17 initialized, components configured.\n");
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
    
    // Launch core1 for DCS-BIOS RS485/USB task
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");

    // DCS-BIOS setup must be called after launching core1 and setting up components.
    DcsBios::setup();
    
    // The main loop now primarily handles DCS-BIOS communication and other background tasks.
    // Individual component pollInput() calls for MCP23S17-connected components are
    // handled by the interrupt service routine (ISR).
    while (true)
    {
        DcsBios::loop();
        pltWpnMasterArm.pollInput();                          // Main DCS-BIOS handler
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        // Check if an MCP23S17 interrupt occurred
        if (mcpInterruptOccurred) {
            mcpInterruptOccurred = false; // Clear the flag

            // Read the interrupt flags to know which pins caused the interrupt
            uint16_t interruptFlags = ioExpander.getInterruptFlags();
            
            // Reading INTCAP registers clears the interrupt on the MCP23S17
            uint16_t capturedValues = ioExpander.getInterruptCapturedValues(); 

            // Process components based on interrupt flags
            if (interruptFlags & (1 << 0)) { // GPA0 for Pitot Heat
                if (g_pltPitotHeat) {
                    g_pltPitotHeat->pollInput();
                }
            }
            if (interruptFlags & ((1 << 1) | (1 << 2))) { // GPA1 or GPA2 for Rotary Encoder
                if (g_pltIntLightConsoleBrightness) {
                    g_pltIntLightConsoleBrightness->pollInput();
                }
            }
            // Add more conditions for other interrupt-driven components here 
        }
    }

    return 0;
    
}