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

//#define afcsCoilPin 10       // Output to hold coil - AP Hold Coil
//define altHoldCoilPin 7     // Output to hold coil - AFCS Coil
//#define FULL_POWER 255       // Full 12V power (PWM 255)
//#define HOLD_POWER_AFCS 125  // Reduced power for AFCS Hold (Increased to 50% duty cycle)
//#define HOLD_POWER_ALTHLD 130 // Reduced power for ALT HOLD (Increased to 50% duty cycle)
//#define HOLD_TIME 3000       // Time in milliseconds to hold full power (3 sec)
#define NUM_LEDS 18          // Total number of SK6812 LEDs
#define ONBOARD_NEOPIXEL_PIN 16 // Define the pin for the onboard NeoPixel


WS2812 externalLeds(pio0, 0, 14, false); // Global WS2812 object for external NeoPixels on pin 5
WS2812 onboardLed(pio1, 0, ONBOARD_NEOPIXEL_PIN, false); // WS2812 object for the onboard NeoPixel on pin 16



uart_inst_t *rs485_uart = uart0; // Control UART in main

// PWM slice and channel variables
uint slice_num_afcs, slice_num_althold;
uint channel_afcs, channel_althold;

// Global variables for non-blocking timing
/*uint32_t afcs_hold_start_time = 0;
uint32_t althold_hold_start_time = 0;
bool afcs_is_on = false;
bool althold_is_on = false;
*/
/*
void setup_pwm(uint gpio_pin)
{
    // Set the GPIO function to PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    // Get PWM slice and channel for this GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint channel = pwm_gpio_to_channel(gpio_pin);

    // Store slice and channel info for each pin
    if (gpio_pin == afcsCoilPin)
    {
        slice_num_afcs = slice_num;
        channel_afcs = channel;
    }
    else if (gpio_pin == altHoldCoilPin)
    {
        slice_num_althold = slice_num;
        channel_althold = channel;
    }

    // Set PWM frequency
    pwm_set_clkdiv(slice_num, 500.0f); // Changed from 32.0f to reduce frequency
    pwm_set_wrap(slice_num, 255); // 8-bit resolution (0-255)

    // Enable PWM
    pwm_set_enabled(slice_num, true);
}

void analog_write(uint gpio_pin, uint16_t value)
{
    // Set PWM duty cycle (0-255) for the correct pin
    if (gpio_pin == afcsCoilPin)
    {
        pwm_set_chan_level(slice_num_afcs, channel_afcs, value);
    }
    else if (gpio_pin == altHoldCoilPin)
    {
        pwm_set_chan_level(slice_num_althold, channel_althold, value);
    }
}

// Function to check hold times and change PWM levels if necessary
void check_hold_times()
{
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Check AFCS hold time
    if (afcs_is_on && afcs_hold_start_time > 0 && (current_time - afcs_hold_start_time >= HOLD_TIME))
    {
        analog_write(afcsCoilPin, HOLD_POWER_AFCS);
        afcs_hold_start_time = 0; // Reset the timer
    }

    // Check ALT HOLD time
    if (althold_is_on && althold_hold_start_time > 0 && (current_time - althold_hold_start_time >= HOLD_TIME))
    {
        analog_write(altHoldCoilPin, HOLD_POWER_ALTHLD);
        althold_hold_start_time = 0; // Reset the timer
    }
}
*/
/*
void onPltAfcsAutopilotChange(unsigned int newafcsValue)
{
    if (newafcsValue == 1)
    {
        analog_write(afcsCoilPin, FULL_POWER);
        afcs_hold_start_time = to_ms_since_boot(get_absolute_time());
        afcs_is_on = true;
    }
    else
    {
        analog_write(afcsCoilPin, 0);
        afcs_is_on = false;
        afcs_hold_start_time = 0;
    }
}
DcsBios::IntegerBuffer pltAfcsAutopilotBuffer(F_4E_PLT_AFCS_AUTOPILOT, onPltAfcsAutopilotChange);

void onPltAfcsAltHoldChange(unsigned int newAltHoldValue)
{
    if (newAltHoldValue == 1)
    {
        analog_write(altHoldCoilPin, FULL_POWER);
        althold_hold_start_time = to_ms_since_boot(get_absolute_time());
        althold_is_on = true;
    }
    else
    {
        analog_write(altHoldCoilPin, 0);
        althold_is_on = false;
        althold_hold_start_time = 0;
    }
}

DcsBios::IntegerBuffer pltAfcsAltHoldBuffer(F_4E_PLT_AFCS_ALT_HOLD, onPltAfcsAltHoldChange);
*/

// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0)); // Red for F-4
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// DCS-BIOS F-4E INPUT FUNCTIONS HERE


DcsBios::Switch3PosLatchedMomentary pltAfcsAltHold("PLT_AFCS_ALT_HOLD", 9, 8);
DcsBios::Switch3PosLatchedMomentary pltAfcsAutopilot("PLT_AFCS_AUTOPILOT", 4, 5);


// On-off toggle for magnetically held momentary toggles
//const uint8_t pltAfcsAltHoldPin = 9;
//DcsBios::Switch2Pos pltAfcsAltHold("PLT_AFCS_ALT_HOLD", pltAfcsAltHoldPin);
//const uint8_t pltAfcsAutopilotPin = 12;
//DcsBios::Switch2Pos pltAfcsAutopilot("PLT_AFCS_AUTOPILOT", pltAfcsAutopilotPin, true);

//const uint8_t pltAfcsStabAugPitchPins[2] = {15, 13};
//DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsStabAugPitch("PLT_AFCS_STAB_AUG_PITCH", pltAfcsStabAugPitchPins);

const uint8_t pltAfcsStabAugPitchPins[2] = {15, 13};
DcsBios::SyncingSwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsStabAugPitch("PLT_AFCS_STAB_AUG_PITCH", pltAfcsStabAugPitchPins,
0x2a70, 0x0400, 10, 50);
const uint8_t pltAfcsStabAugRollPins[2] = {27, 26};
DcsBios::SyncingSwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsStabAugRoll("PLT_AFCS_STAB_AUG_ROLL", pltAfcsStabAugRollPins,
0x2a70, 0x0200, 9, 50);
const uint8_t pltAfcsStabAugYawPins[2] = {28, 29};
DcsBios::SyncingSwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsStabAugYaw("PLT_AFCS_STAB_AUG_YAW", pltAfcsStabAugYawPins,
0x2a70, 0x0100, 8, 50);

// DCS-BIOS F-14A/B FUNCTIONS HERE
DcsBios::Switch3PosLatchedMomentary pltAutopltAlt("PLT_AUTOPLT_ALT", 9, 8);
DcsBios::Switch3PosLatchedMomentary pltAutopltEngage("PLT_AUTOPLT_ENGAGE", 4, 5);

//const uint8_t pltAutopltAltPin = 8;
//DcsBios::Switch2Pos pltAutopltAlt("PLT_AUTOPLT_ALT", pltAutopltAltPin);
//const uint8_t pltAutopltEngagePin = 12;
//DcsBios::Switch2Pos pltAutopltEngage("PLT_AUTOPLT_ENGAGE", pltAutopltEngagePin, true);

const uint8_t pltAfcsPitchPins[2] = {15, 13};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsPitch("PLT_AFCS_YAW", pltAfcsPitchPins);
const uint8_t pltAfcsRollPins[2] = {27, 26};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsRoll("PLT_AFCS_ROLL", pltAfcsRollPins);
const uint8_t pltAfcsYawPins[2] = {28, 29};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsYaw("PLT_AFCS_PITCH", pltAfcsYawPins);

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


// Updated callback functions


int main()
{
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);

    // Initialize the onboard NeoPixel
  //  onboardLed.begin(1);
/*
    setup_pwm(afcsCoilPin);
    setup_pwm(altHoldCoilPin);
*/
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
    // Visual debug: Flash onboard LED to signal board mode is determined
    /*onboardLed.setPixel(0, onboardLed.rgbw(0, 0, 255, 0)); // Blue flash
    onboardLed.show();
    sleep_ms(500);
    onboardLed.clear();
    onboardLed.show();
*/
    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);

    DcsBios::setup();

    while (true)
    {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        //check_hold_times(); // <-- new non-blocking check
        sleep_us(10);
    }
}