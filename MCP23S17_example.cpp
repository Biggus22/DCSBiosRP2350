#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/pwm.h" // Corrected: Added 'h'
#include "hardware/adc.h"

#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/MCP23S17.h" // Your MCP23S17 header
#include "internal/Switches.h" // Your modified Switches.h (now includes MCP23S17-specific switch classes)
#include "internal/Encoders.h" // Your modified Encoders.h (now includes MCP23S17-specific encoder classes)
// #include "internal/AnalogInputs.h" // Removed as per user's instruction

// SPI Configuration
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19

// MCP23S17 Configuration
#define MCP23S17_ADDRESS 0x20   // Hardware address (A2=A1=A0=0)
#define MCP23S17_ADDRESS_2 0x21 // Hardware address for the second MCP23S17

// MCP23S17 instances
MCP23S17 ioExpander(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, MCP23S17_ADDRESS);
MCP23S17 ioExpander2(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, MCP23S17_ADDRESS_2); // Second expander

// For Console Lighting Potentiometer (If you remove analog, remove this section)
#define CONSOLE_LIGHTING_POT_PIN 26 // ADC0 input for potentiometer
#define CONSOLE_DIMMING_DEADZONE 10 // Deadzone for brightness changes

unsigned int lastBrightness = 0;

// DCS-BIOS output callback for console brightness LED
void onPltIntLightConsoleBrightnessChange(unsigned int newValue)
{
    if (abs((int)newValue - (int)lastBrightness) >= CONSOLE_DIMMING_DEADZONE)
    {
        lastBrightness = newValue;
        uint16_t pwmVal = newValue; // newValue is already 0-65535 for PWM
        pwm_set_gpio_level(CONSOLE_LIGHTING_POT_PIN, pwmVal);
    }
}

// DCS-BIOS Inputs using MCP23S17
// Example: 2-position switch for Pitot Heat
// Using pollIntervalMs = 0 (POLL_EVERY_TIME)
DcsBios::Switch2PosT<0> pltPitotHeat("PLT_PITOT_HEAT", &ioExpander, 0, false); // Pin GPA0 on MCP23S17_ADDRESS 0x20

// Example: Rotary Encoder for Console Lighting Brightness
// Using pollIntervalMs = 0 (POLL_EVERY_TIME) and ONE_STEP_PER_DETENT
// Corrected constructor arguments: Reordered to match MCP23S17 constructor
DcsBios::RotaryEncoderT<0, DcsBios::ONE_STEP_PER_DETENT> pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", "INC", "DEC", &ioExpander, 1, 2); // Pins GPA1, GPA2 on MCP23S17_ADDRESS 0x20

// Example: 3-position switch for AFCS Autopilot
// Using pollIntervalMs = 0 (POLL_EVERY_TIME) and 2 positions (0, 1)
const uint8_t afcsAutopilotPins[] = {4, 5}; // Pins GPA4, GPA5 on MCP23S17_ADDRESS 0x20
// Corrected constructor arguments: Added message string "PLT_AFCS_AUTOPILOT"
DcsBios::SwitchMultiPosT<0, 2> pltAfcsAutopilot("PLT_AFCS_AUTOPILOT", &ioExpander, afcsAutopilotPins, 50); // Using 2 positions for "Engage" (0) and "Off" (1)

// Example: Rotary Encoder for CM Chaff Burst Count on a second MCP23S17
// Using pollIntervalMs = 0 (POLL_EVERY_TIME) and ONE_STEP_PER_DETENT
// Corrected constructor arguments: Reordered to match MCP23S17 constructor
DcsBios::RotaryEncoderT<0, DcsBios::ONE_STEP_PER_DETENT> pltCmChaffBurstCount("PLT_CM_CHAFF_BURST_COUNT", "INC", "DEC", &ioExpander2, 1, 8); // Pins GPB1, GPA1 on MCP23S17_ADDRESS 0x21 (GPA1 is pin 8)

// Analog Input for Console Lighting Potentiometer (Keep or remove based on your needs)
// Using pollIntervalMs = 0 (POLL_EVERY_TIME)
// DcsBios::PotentiometerEWMA<0> consoleLightingPot("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", CONSOLE_LIGHTING_POT_PIN, 0.1);

int main()
{
    // Initialize standard I/O for debugging
    stdio_init_all();

    // Init PWM for LED output (If you remove analog, remove this section)
    gpio_set_function(CONSOLE_LIGHTING_POT_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(CONSOLE_LIGHTING_POT_PIN);
    pwm_set_wrap(slice_num, 65535);   // Set PWM wrap value to 65535 (16-bit)
    pwm_set_enabled(slice_num, true); // Enable PWM

    // Init ADC for analog pot (If you remove analog, remove this section)
    adc_init();
    adc_gpio_init(CONSOLE_LIGHTING_POT_PIN); // ADC0

    // Initialize MCP23S17s
    printf("Initializing MCP23S17s...\n");
    ioExpander.begin();
    ioExpander2.begin();

    // Pin configurations are now handled by the constructors of the DcsBios switch and encoder classes.
    // Explicit ioExpander.pinMode() calls are generally not needed here for pins used by those objects.

    // Enable interrupts on MCP23S17 0x20 for pins 0, 1, 2, 4, 5
    // Note: If you add more DcsBios::MCP23S17* objects, you might need to enable interrupts for their pins here.
    ioExpander.enableInterrupt(0);
    ioExpander.enableInterrupt(1);
    ioExpander.enableInterrupt(2);
    ioExpander.enableInterrupt(4);
    ioExpander.enableInterrupt(5);
    printf("MCP23S17 0x20 pins 0,1,2,4,5 configured for interrupts.\n");

    // Enable interrupts on MCP23S17 0x21 for pins 1 and 8
    ioExpander2.enableInterrupt(1);
    ioExpander2.enableInterrupt(8);
    printf("MCP23S17 0x21 pins 1 and 8 configured for interrupts.\n");

    printf("MCP23S17s initialized and pins configured (by DcsBios objects).\n");

    // Launch core1 for DCS-BIOS RS485/USB task
    multicore_launch_core1(DcsBios::core1_task);

    // DCS-BIOS init
    DcsBios::setup();
    printf("DCS-BIOS setup complete\n");

    while (true)
    {
        DcsBios::loop();            // Main DCS-BIOS handler, polls all DcsBios::PollingInput objects
        DcsBios::updateHeartbeat(); // Blink status LED (if connected)
        sleep_us(10);               // Slight delay
    }

    return 0;
}