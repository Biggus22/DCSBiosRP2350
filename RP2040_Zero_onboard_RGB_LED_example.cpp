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
#include "internal/ws2812.h" // Include the WS2812 header

// Global WS2812 object
WS2812 onboardLed(pio0, 0, NEO_DRIVE_PIN);

void init_i2c0() {
    i2c_init(i2c0, 400 * 1000);  // 400kHz
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
}

// DCS-BIOS callback function for PLT_INT_LIGHT_CONSOLE
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    // newValue is expected to be a value from 0 to 65535 based on DCS-BIOS integer buffers.
    // Scale this value to a brightness from 0 to 255 for the WS2812 LED.
    uint8_t brightness = (uint8_t)((consoleBrightness * 255) / 65535);

    // Set the brightness of the onboard WS2812 LED
    onboardLed.setBrightness(brightness);

    // Set the LED color to red (GRB format for WS2812)
    // Red color is 0x00FF0000 in GRB format (Green 0, Red 255, Blue 0)
    onboardLed.setPixel(0, onboardLed.rgb(201, 0, 0)); // Set pixel 0 to red

    // Update the LED
    onboardLed.show();
}

// Declare the IntegerBuffer
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);


int main() {
    stdio_init_all();  // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
    sleep_ms(5000);   // Wait for USB CDC to be ready


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



    init_i2c0();  // Initialize I2C0
    sleep_ms(100); 
    
    // Initialize the WS2812 LED
    if (onboardLed.begin(1)) { // 1 pixel
        printf("WS2812 onboard LED initialized successfully!\n");
        // Set initial color to off
        onboardLed.clear();
        onboardLed.show();
    } else {
        printf("Failed to initialize WS2812 onboard LED!\n");
    }

    DcsBios::Switch2Pos emerJettBtn("EMER_JETT_BTN", 6);

    DcsBios::setup();  // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true) {
        DcsBios::loop(); // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}