#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
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
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "internal/TM1637.h"


#define NUM_LEDS 10          // Total number of SK6812 LEDs

WS2812 externalLeds(pio0, 0, 5, false); // Global WS2812 object for external NeoPixels on pin 5
// TM1637 display pins - adjust as needed for your hardware
#define TM1637_CLK_PIN 23
#define TM1637_DATA_PIN 21

uart_inst_t *rs485_uart = uart0;

// TM1637 display object
TM1637 tm1637(TM1637_CLK_PIN, TM1637_DATA_PIN);

char lastFrequency[7] = {0}; // Store last displayed frequency to avoid unnecessary updates

void onPltVorIlsFrequencyChange(char *newValue)
{
    printf("VOR/ILS Frequency: %s\n", newValue);
    
    // Only update display if frequency has changed
    if (strcmp(newValue, lastFrequency) != 0) {
        strcpy(lastFrequency, newValue);
        
        char digitsOnly[6] = {0}; // To store only the digits from the frequency
        int decimalPositionInOriginal = -1;
        int digitIndex = 0;
        
        // Iterate through the input string to extract digits and find the decimal position
        for (int i = 0; i < strlen(newValue); i++) {
            if (newValue[i] == '.') {
                // Store the position of the decimal point in the original string
                decimalPositionInOriginal = i;
            } else {
                digitsOnly[digitIndex++] = newValue[i];
            }
        }
        
        char displayValue[7]; // 6 display characters + 1 for null terminator
        
        // Pad the entire buffer with spaces
        memset(displayValue, ' ', sizeof(displayValue));
        displayValue[6] = '\0'; // Ensure it is null-terminated
        
        // Calculate the starting position to shift digits left by one position
        int digitsOnlyLen = strlen(digitsOnly);
        int startIndex = 5 - digitsOnlyLen; // Changed from 6 to 5 to shift one position left
        if (startIndex < 0) startIndex = 0; // Prevent negative index if too many digits
        
        // Copy the characters from the new value into the padded string
        for (int i = 0; i < digitsOnlyLen && startIndex + i < 6; ++i) {
            displayValue[startIndex + i] = digitsOnly[i];
        }

        // Calculate the correct decimal position in the final padded string
        int finalDecimalPosition = -1;
        if (decimalPositionInOriginal != -1) {
            // The position is the start index plus the number of digits before the decimal
            finalDecimalPosition = startIndex + (decimalPositionInOriginal) -1;
        }
        
        // Display the formatted digits and the decimal point. The TM1637 class handles the decimal.
        tm1637.displayWithDecimalPoint(displayValue, finalDecimalPosition, false, true);
        printf("Displayed on TM1637: %s with decimal at position %d\n", displayValue, finalDecimalPosition);
    }
}
DcsBios::StringBuffer<6> pltVorIlsFrequencyBuffer(F_4E_PLT_VOR_ILS_FREQUENCY_A, onPltVorIlsFrequencyChange);

// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0)); // Red for F-4
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

DcsBios::Switch2Pos pltVorIlsTest("PLT_VOR_ILS_TEST", 26);
DcsBios::Potentiometer pltVorIlsMarkerVolume("PLT_VOR_ILS_MARKER_VOLUME", 28);
DcsBios::Potentiometer pltVorIlsVolume("PLT_VOR_ILS_VOLUME", 27, true);
//DcsBios::RotaryEncoder1Step pltVorIlsFrequencyHundreds("PLT_VOR_ILS_FREQUENCY_HUNDREDS", "DEC", "INC", 3, 4);
//DcsBios::RotaryEncoder1Step pltVorIlsFrequencyDecimals("PLT_VOR_ILS_FREQUENCY_DECIMALS", "DEC", "INC", 7, 8);

// Example of how to use the EmulatedConcentricRotaryEncoder
// This creates an encoder that behaves differently based on the state of a push button
// When the button is not pressed (HIGH), it sends commands for VOR/ILS frequency hundreds
// When the button is pressed (LOW), it sends commands for VOR/ILS frequency decimals
 DcsBios::EmulatedConcentricRotaryEncoder4Step emulatedEncoder(
     "PLT_VOR_ILS_FREQUENCY_HUNDREDS", "DEC", "INC",           // Function 1 (default): VOR/ILS hundreds
     "PLT_VOR_ILS_FREQUENCY_DECIMALS", "DEC", "INC",          // Function 2: VOR/ILS decimals  
     3, 4, 20);                                              // Encoder pins A=3, B=4, Button=20

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
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");


    // Initialize the TM1637 display
    printf("Initializing TM1637 display...\n");
    tm1637.begin();
    tm1637.setBrightness(1); // Set maximum brightness
    tm1637.display("108.00", false, true); // Display shifted left to indicate startup
    sleep_ms(1000); // Show startup message for 1 second
    tm1637.clear();
    DcsBios::setup(); // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}