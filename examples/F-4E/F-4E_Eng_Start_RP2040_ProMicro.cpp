#ifndef RP2040_BOARD
#define RP2040_BOARD
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


#define NUM_LEDS 10          // Total number of SK6812 LEDs


WS2812 externalLeds(pio0, 0, 16, false); // Global WS2812 object for external NeoPixels on pin 16 (Changed to 24-bit RGB mode)
unsigned int lastBrightness = 0;         // Last brightness value for console lighting

uart_inst_t *rs485_uart = uart0; // Control UART in main

// PWM slice and channel variables
uint slice_num_afcs, slice_num_althold;
uint channel_afcs, channel_althold;

void setup_pwm(uint gpio_pin)
{
    // Set the GPIO function to PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    // Get PWM slice and channel for this GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint channel = pwm_gpio_to_channel(gpio_pin);



    // Set PWM frequency (optional - adjust as needed)
    pwm_set_clkdiv(slice_num, 4.0f);
    pwm_set_wrap(slice_num, 255); // 8-bit resolution (0-255)

    // Enable PWM
    pwm_set_enabled(slice_num, true);
}


// DCS-BIOS callback function for PLT_INT_LIGHT_CONSOLE
void onPltIntLightConsoleChange(unsigned int consoleBrightness)
{
    uint8_t brightness = (uint8_t)((consoleBrightness * 255) / 65535);
    externalLeds.setBrightness(brightness);

    // Set all 18 external LEDs to green with no white component
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Using rgbw for SK6812
    }
    externalLeds.show();
}
// Declare the IntegerBuffer
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);



const uint8_t pltEngineMasterLPins[2] = {7, 6};
DcsBios::SwitchMultiPos<2> pltEngineMasterL("PLT_ENGINE_MASTER_L", pltEngineMasterLPins);

const uint8_t pltEngineMasterRPins[2] = {14, 15};
DcsBios::SwitchMultiPos<2> pltEngineMasterR("PLT_ENGINE_MASTER_R", pltEngineMasterRPins);

const uint8_t pltEngineStartRPins[2] = {12, 13};
DcsBios::SwitchMultiPos<3> pltEngineStart("PLT_ENGINE_START", pltEngineStartRPins);

const uint8_t pltControlsRudderTrimPins[2] = {9, 8};
DcsBios::SwitchMultiPos<3> pltControlsRudderTrim("PLT_CONTROLS_RUDDER_TRIM", pltControlsRudderTrimPins);

const uint8_t pltCadcCorrectionPins[2] = {10, 11};
DcsBios::SwitchMultiPos<3> pltCadcCorrection("PLT_CADC_CORRECTION", pltCadcCorrectionPins);

int main()
{
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready


    // Initialize the WS2812 strip for the power-on flash
    externalLeds.begin(NUM_LEDS); // Initialize with correct number of pixels

    // Power-on Green Flash for 1 second
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(201, 0, 0, 0)); // Set all pixels to green (R=0, G=255, B=0, W=0)
    }
    externalLeds.show();
    sleep_ms(5000); // Display green for 1 second

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

    DcsBios::setup(); // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    printf("Minimal testing program running\n");

    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}
