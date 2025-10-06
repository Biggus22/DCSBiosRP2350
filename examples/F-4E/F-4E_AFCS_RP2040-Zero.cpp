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


WS2812 externalLeds(pio0, 0, 5, false); // Global WS2812 object for external NeoPixels on pin 14 (Changed to 24-bit RGB mode)
WS2812 onboardLed(pio1, 0, ONBOARD_NEOPIXEL_PIN, false); // WS2812 object for the onboard NeoPixel on pin 16
unsigned int lastBrightness = 0;         // Last brightness value for console lighting

char currentAircraftName[24] = "";       // Current aircraft name for identification

bool isF4Active();
bool isF14Active();

// Function to check if the current aircraft is an F-4
bool isF4Active() {
    return (strlen(currentAircraftName) > 0 && 
            (strstr(currentAircraftName, "F-4") != NULL || 
             strstr(currentAircraftName, "F4") != NULL));
}

// Function to check if the current aircraft is an F-14
bool isF14Active() {
    return (strlen(currentAircraftName) > 0 && 
            (strstr(currentAircraftName, "F-14") != NULL || 
             strstr(currentAircraftName, "F14") != NULL));
}
// Enhanced LED state tracking
enum LedState {
    LED_OFF,              // LEDs completely off (DCS not running or crashed)
    LED_RAINBOW_LOADING,  // Rainbow effect (DCS running, no aircraft or loading)
    LED_NORMAL_F4,        // F-4 console lighting mode
    LED_NORMAL_F14,       // F-14 console lighting mode  
    LED_FADE_TO_BLUE,     // Fading to blue before shutdown
    LED_BLUE_HOLD,        // Holding blue color
    LED_FADE_OUT          // Fading out to off
};

LedState currentLedState = LED_OFF;
uint32_t ledStateStartTime = 0;
uint32_t lastDcsBiosDataTime = 0;
uint32_t lastModTimeUpdate = 0;
char lastModTime[7] = "";  // Store the last mod time string
bool aircraftNameReceived = false; // Flag to track if we've received any aircraft name data



// Timing constants (in milliseconds)
const uint32_t DCS_ALIVE_TIMEOUT = 10000;      // 10 seconds with no data = DCS probably dead
const uint32_t MOD_TIME_STALE_TIMEOUT = 60000;  // 1 minute with same mod time = game paused/menu
const uint32_t FADE_TO_BLUE_DURATION = 2000;    // 2 seconds to fade to blue
const uint32_t BLUE_HOLD_DURATION = 3000;       // 3 seconds holding blue
const uint32_t FADE_OUT_DURATION = 30000;       // 30 seconds to fade out completely
const uint32_t RAINBOW_CYCLE_SPEED = 50;        // Rainbow color cycle speed (lower = faster)



// Function to determine what LED state we should be in
LedState determineLedState() {
    uint32_t currentTime = to_ms_since_boot(get_absolute_time());
    uint32_t timeSinceLastData = currentTime - lastDcsBiosDataTime;
    uint32_t timeSinceModTimeUpdate = currentTime - lastModTimeUpdate;
    
    // Check if DCS appears to be completely dead (no data at all for a while)
    if (timeSinceLastData > DCS_ALIVE_TIMEOUT) {
        aircraftNameReceived = false; // Reset the flag if we lose DCS connection
        return LED_OFF;
    }
    
    // Check if we have received any aircraft name data
    if (!aircraftNameReceived) {
        // We haven't received any aircraft name data yet, so DCS is probably not running
        return LED_OFF;
    }
    
    // We have received aircraft name data, so DCS is running
    // Check if we have a valid aircraft loaded
    if (strlen(currentAircraftName) > 0) {
        if (isF4Active()) {
            // Check if mod time is still updating (game running vs paused/menu)
            if (timeSinceModTimeUpdate < MOD_TIME_STALE_TIMEOUT) {
                return LED_NORMAL_F4;
            } else {
                // Game is paused or in menu - start shutdown sequence
                return LED_FADE_TO_BLUE;
            }
        } else if (isF14Active()) {
            if (timeSinceModTimeUpdate < MOD_TIME_STALE_TIMEOUT) {
                return LED_NORMAL_F14;
            } else {
                return LED_FADE_TO_BLUE;
            }
        }
    }
    
    // DCS is running but no aircraft loaded (or loading) - rainbow mode
    return LED_RAINBOW_LOADING;
}

// Enhanced rainbow effect with smoother animation
void updateRainbowEffect(uint32_t elapsedTime, float brightnessMultiplier = 1.0f) {
    uint32_t colorOffset = (elapsedTime / RAINBOW_CYCLE_SPEED) % 360;
    
    for (int i = 0; i < NUM_LEDS; i++) {
        // Create smooth rainbow across the strip with animation
        float hue = ((i * 360.0f / NUM_LEDS) + colorOffset) / 360.0f;
        hue = hue - floor(hue);  // Normalize to 0-1
        
        // Convert HSV to RGB with proper saturation and value
        float h = hue * 6.0f;
        float c = 255.0f * brightnessMultiplier;  // Chroma (intensity)
        float x = c * (1.0f - fabs(fmod(h, 2.0f) - 1.0f));
        float m = 0;
        
        float r, g, b;
        int h_i = (int)h;
        
        switch (h_i) {
            case 0: r = c; g = x; b = 0; break;
            case 1: r = x; g = c; b = 0; break;
            case 2: r = 0; g = c; b = x; break;
            case 3: r = 0; g = x; b = c; break;
            case 4: r = x; g = 0; b = c; break;
            case 5: r = c; g = 0; b = x; break;
            default: r = 0; g = 0; b = 0; break;
        }
        
        externalLeds.setPixel(i, externalLeds.rgbw((uint8_t)r, (uint8_t)g, (uint8_t)b, 0));
    }
    externalLeds.show();
}

// Enhanced LED management function
void manageLedStates() {
    uint32_t currentTime = to_ms_since_boot(get_absolute_time());
    LedState targetState = determineLedState();
    
    // Handle state transitions
    if (currentLedState != targetState) {
        // State change detected
        switch (targetState) {
            case LED_OFF:
                // Immediate transition to off
                externalLeds.clear();
                externalLeds.show();
                currentLedState = LED_OFF;
                break;
                
            case LED_RAINBOW_LOADING:
                currentLedState = LED_RAINBOW_LOADING;
                ledStateStartTime = currentTime;
                break;
                
            case LED_NORMAL_F4:
            case LED_NORMAL_F14:
                currentLedState = targetState;
                // LED updates will be handled by console brightness callbacks
                break;
                
            case LED_FADE_TO_BLUE:
                if (currentLedState == LED_NORMAL_F4 || currentLedState == LED_NORMAL_F14) {
                    currentLedState = LED_FADE_TO_BLUE;
                    ledStateStartTime = currentTime;
                }
                break;
        }
    }
    
    // Handle current state updates
    uint32_t elapsedTime = currentTime - ledStateStartTime;
    
    switch (currentLedState) {
        case LED_OFF:
            // LEDs already off, nothing to do
            break;
            
        case LED_RAINBOW_LOADING:
            updateRainbowEffect(elapsedTime);
            break;
            
        case LED_NORMAL_F4:
        case LED_NORMAL_F14:
            // Console brightness callbacks handle these states
            break;
            
        case LED_FADE_TO_BLUE:
            if (elapsedTime < FADE_TO_BLUE_DURATION) {
                // Fade from current color to blue
                float fadeProgress = (float)elapsedTime / (float)FADE_TO_BLUE_DURATION;
                uint8_t blueIntensity = (uint8_t)(255 * fadeProgress);
                uint8_t otherIntensity = (uint8_t)(255 * (1.0f - fadeProgress));
                
                for (int i = 0; i < NUM_LEDS; i++) {
                    // Assume current color was red for F-4, green for F-14
                    uint8_t red = isF4Active() ? otherIntensity : 0;
                    uint8_t green = isF14Active() ? otherIntensity : 0;
                    
                    externalLeds.setPixel(i, externalLeds.rgbw(red, green, blueIntensity, 0));
                }
                externalLeds.show();
            } else {
                // Transition to blue hold
                currentLedState = LED_BLUE_HOLD;
                ledStateStartTime = currentTime;
            }
            break;
            
        case LED_BLUE_HOLD:
            if (elapsedTime < BLUE_HOLD_DURATION) {
                // Hold blue color
                for (int i = 0; i < NUM_LEDS; i++) {
                    externalLeds.setPixel(i, externalLeds.rgbw(0, 0, 255, 0));
                }
                externalLeds.show();
            } else {
                // Transition to fade out
                currentLedState = LED_FADE_OUT;
                ledStateStartTime = currentTime;
            }
            break;
            
        case LED_FADE_OUT:
            if (elapsedTime < FADE_OUT_DURATION) {
                // Fade blue to off
                float fadeProgress = 1.0f - ((float)elapsedTime / (float)FADE_OUT_DURATION);
                uint8_t blueIntensity = (uint8_t)(255 * fadeProgress);
                
                for (int i = 0; i < NUM_LEDS; i++) {
                    externalLeds.setPixel(i, externalLeds.rgbw(0, 0, blueIntensity, 0));
                }
                externalLeds.show();
            } else {
                // Fade complete, turn off
                currentLedState = LED_OFF;
                externalLeds.clear();
                externalLeds.show();
            }
            break;
    }
}

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

void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    if (isF4Active() && currentLedState == LED_NORMAL_F4) {
        uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
        
        for (int i = 0; i < NUM_LEDS; i++) {
            externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0));
        }
        externalLeds.show();
    }
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// DCS-BIOS F-4E INPUT FUNCTIONS HERE


DcsBios::Switch3PosLatchedMomentary pltAfcsAltHold("PLT_AFCS_ALT_HOLD", 9, 8);
DcsBios::Switch3PosLatchedMomentary pltAfcsAutopilot("PLT_AFCS_AUTOPILOT", 5, 4);


// On-off toggle for magnetically held momentary toggles
//const uint8_t pltAfcsAltHoldPin = 9;
//DcsBios::Switch2Pos pltAfcsAltHold("PLT_AFCS_ALT_HOLD", pltAfcsAltHoldPin);
//const uint8_t pltAfcsAutopilotPin = 12;
//DcsBios::Switch2Pos pltAfcsAutopilot("PLT_AFCS_AUTOPILOT", pltAfcsAutopilotPin, true);

const uint8_t pltAfcsStabAugPitchPins[2] = {15, 13};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsStabAugPitch("PLT_AFCS_STAB_AUG_PITCH", pltAfcsStabAugPitchPins);
const uint8_t pltAfcsStabAugRollPins[2] = {27, 26};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsStabAugRoll("PLT_AFCS_STAB_AUG_ROLL", pltAfcsStabAugRollPins);
const uint8_t pltAfcsStabAugYawPins[2] = {28, 29};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltAfcsStabAugYaw("PLT_AFCS_STAB_AUG_YAW", pltAfcsStabAugYawPins);

// DCS-BIOS F-14A/B FUNCTIONS HERE
DcsBios::Switch3PosLatchedMomentary pltAutopltAlt("PLT_AUTOPLT_ALT", 9, 8);
DcsBios::Switch3PosLatchedMomentary pltAutopltEngage("PLT_AUTOPLT_ENGAGE", 5, 4);

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

// DCS-BIOS callback function for F-14 console lighting
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    if (isF14Active() && currentLedState == LED_NORMAL_F14) {
        uint8_t brightness = 0;
        
        if (consoleBrightness <= 8) {
            uint8_t percentages[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};
            brightness = (percentages[consoleBrightness] * 255) / 100;
        } else {
            brightness = (uint8_t)((consoleBrightness * 255) / 65535);
        }
        
        for (int i = 0; i < NUM_LEDS; i++) {
            externalLeds.setPixel(i, externalLeds.rgbw(0, brightness, 0, 0));
        }
        externalLeds.show();
    }
}

// Declare the IntegerBuffer for F-14 console lighting
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);


// Updated callback functions
void onAcftNameChange(char* newValue) {
    lastDcsBiosDataTime = to_ms_since_boot(get_absolute_time());
    strncpy(currentAircraftName, newValue, sizeof(currentAircraftName) - 1);
    currentAircraftName[sizeof(currentAircraftName) - 1] = '\0';
    aircraftNameReceived = true; // Set the flag when we receive any aircraft name data
}
DcsBios::StringBuffer<24> AcftNameBuffer(MetadataStart_ACFT_NAME_A, onAcftNameChange);

void onModTimeChange(char* newValue) {
    uint32_t currentTime = to_ms_since_boot(get_absolute_time());
    lastDcsBiosDataTime = currentTime;
    aircraftNameReceived = true; // Also set the flag when we receive mod time data (indicates DCS is running)
    
    // Check if mod time actually changed
    if (strcmp(lastModTime, newValue) != 0) {
        lastModTimeUpdate = currentTime;
        strncpy(lastModTime, newValue, sizeof(lastModTime) - 1);
        lastModTime[sizeof(lastModTime) - 1] = '\0';
    }
}
DcsBios::StringBuffer<6> modTimeBuffer(CommonData_MOD_TIME_A, onModTimeChange);

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
        manageLedStates(); // Check if LEDs should be turned off due to timeout
        sleep_us(10);
    }
}