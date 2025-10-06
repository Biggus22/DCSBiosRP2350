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
#define NUM_LEDS 33 // Total number of SK6812 LEDs
#define CMS_LED 13 // CMS ON LED
#define FLARES_LED 12 // FLARES ON LED
#define ONBOARD_NEOPIXEL_PIN 16 // Define the pin for the onboard NeoPixel

WS2812 externalLeds(pio0, 0, 3, false); // Global WS2812 object for external NeoPixels on pin 3
unsigned int lastBrightness = 0;

char currentAircraftName[24] = "";       // Current aircraft name for identification
bool aircraftNameReceived = false;       // Flag to track if we've received any aircraft name data

bool isF4Active();
bool isF14Active();

// Function to check if the current aircraft is an F-4
bool isF4Active() {
    if (strlen(currentAircraftName) == 0) return false;
    
    // Create a lowercase version for case-insensitive comparison
    char lowerName[24];
    for (int i = 0; i < 24 && currentAircraftName[i] != '\0'; i++) {
        lowerName[i] = (currentAircraftName[i] >= 'A' && currentAircraftName[i] <= 'Z') ? 
                       currentAircraftName[i] + 32 : currentAircraftName[i];
    }
    lowerName[23] = '\0'; // Ensure null termination
    
    return (strstr(lowerName, "f-4") != NULL || 
            strstr(lowerName, "f4") != NULL);
}

// Function to check if the current aircraft is an F-14
bool isF14Active() {
    if (strlen(currentAircraftName) == 0) return false;
    
    // Create a lowercase version for case-insensitive comparison
    char lowerName[24];
    for (int i = 0; i < 24 && currentAircraftName[i] != '\0'; i++) {
        lowerName[i] = (currentAircraftName[i] >= 'A' && currentAircraftName[i] <= 'Z') ? 
                       currentAircraftName[i] + 32 : currentAircraftName[i];
    }
    lowerName[23] = '\0'; // Ensure null termination
    
    return (strstr(lowerName, "f-14") != NULL || 
            strstr(lowerName, "f14") != NULL);
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

uart_inst_t *rs485_uart = uart0;

// DCS-BIOS F-4E INPUT FUNCTIONS HERE
//const uint8_t pltFuelAirRefuelPins[2] = {7, 8};
DcsBios::Switch2PosT<> pltFuelAirRefuel("PLT_FUEL_AIR_REFUEL", 7);
DcsBios::Switch3Pos2Pin pltFuelExternalTanksFeed("PLT_FUEL_EXTERNAL_TANKS_FEED", 29, 28);
DcsBios::SwitchWithCover2PosT<> pltFuelRefuelSelector("PLT_FUEL_REFUEL_SELECTOR", "PLT_FUEL_REFUEL_SELECTOR_COVER", 14);

//const uint8_t pltFuelWingFuelDumpPins[2] = {9, 10};
DcsBios::Switch2PosT<> pltFuelWingFuelDump("PLT_FUEL_WING_FUEL_DUMP", 9);

DcsBios::Switch2PosT<> pltFuelBoostPumpLCheck("PLT_FUEL_BOOST_PUMP_L_CHECK", 6);
DcsBios::Switch2PosT<> pltFuelBoostPumpRCheck("PLT_FUEL_BOOST_PUMP_R_CHECK", 5);

const uint8_t pltFuelWingInternalFeedPins[2] = {27, 26};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 2> pltFuelWingInternalFeed("PLT_FUEL_WING_INTERNAL_FEED", pltFuelWingInternalFeedPins);

DcsBios::Switch2Pos pltCmFlareNormal("PLT_CM_FLARE_NORMAL", 11, true);

void onPltCmFlareLightChange(unsigned int newValue) {
    lastDcsBiosDataTime = to_ms_since_boot(get_absolute_time()); // Update data time when receiving console brightness
    if (newValue == 1) {
        gpio_put(FLARES_LED, 1); // Set the pin high (on)
    } 
    // Otherwise, turn the LED off
    else {
        gpio_put(FLARES_LED, 0); // Set the pin low (off)
    }
}

DcsBios::IntegerBuffer pltCmFlareLightBuffer(F_4E_PLT_CM_FLARE_LIGHT, onPltCmFlareLightChange);

void onPltCmOnLightChange(unsigned int newValue) {
    lastDcsBiosDataTime = to_ms_since_boot(get_absolute_time()); // Update data time when receiving CMS ON light data
    // Check if newValue is 1 to turn the LED on
    if (newValue == 1) {
        gpio_put(CMS_LED, 1); // Set the pin high (on)
    } 
    // Otherwise, turn the LED off
    else {
        gpio_put(CMS_LED, 0); // Set the pin low (off)
    }
}
DcsBios::IntegerBuffer pltCmOnLightBuffer(F_4E_PLT_CM_ON_LIGHT, onPltCmOnLightChange);

// DCS-BIOS callback function for F-4E console lighting
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    lastDcsBiosDataTime = to_ms_since_boot(get_absolute_time()); // Update data time when receiving console brightness
    if (isF4Active() && currentLedState == LED_NORMAL_F4) {
        uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
        
        for (int i = 0; i < NUM_LEDS; i++) {
            externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0));
        }
        externalLeds.show();
    }
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// DCS-BIOS F-14A/B FUNCTIONS HERE
//const uint8_t pltIcsAmpSelPins[3] = {5, 3, 4};
//DcsBios::SwitchMultiPosT<3> pltIcsAmpSel("PLT_ICS_AMP_SEL", pltIcsAmpSelPins);
//const uint8_t pltIcsFuncSelPins[3] = {8, 9};
//DcsBios::SwitchMultiPosT<3> pltIcsFuncSel("PLT_ICS_FUNC_SEL", pltIcsFuncSelPins);

// DCS-BIOS callback function for F-14 console lighting
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    lastDcsBiosDataTime = to_ms_since_boot(get_absolute_time()); // Update data time when receiving F-14 console brightness
    if (isF14Active() && currentLedState == LED_NORMAL_F14) {
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
            externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0)); // Using rgbw for SK6812
        }
        externalLeds.show();
    }
}
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
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready
    externalLeds.begin(NUM_LEDS); // Initialize with 10 pixels

    // Initialize the GPIO pin for the LED
    gpio_init(CMS_LED);
    gpio_init(FLARES_LED);
    // Set the LED pin as an output
    gpio_set_dir(CMS_LED, GPIO_OUT);
    gpio_set_dir(FLARES_LED, GPIO_OUT);

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
        manageLedStates(); // Check if LEDs should be turned off due to timeout
        sleep_us(10);
    }
}
