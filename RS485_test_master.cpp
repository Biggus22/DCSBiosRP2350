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

#define CONSOLE_LIGHTING_LED 25     // Console LED on GPIO 25
#define CONSOLE_DIMMING_DEADZONE 0  // Deadzone for brightness changes


uart_inst_t* rs485_uart = uart0; // Control UART in main

unsigned int lastBrightness = 0; // Last brightness value for console lighting

int main() {
    stdio_init_all();  // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
    sleep_ms(5000);   // Wait for USB CDC to be ready

    DcsBios::initDeviceAddressPins(ADDR0, ADDR1, ADDR2, ADDR3);
    uint8_t boardAddress = DcsBios::readDeviceAddress();

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
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

        // Initialize PWM for console lighting LED
    gpio_set_function(CONSOLE_LIGHTING_LED, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(CONSOLE_LIGHTING_LED);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);         // Slow down PWM frequency
    pwm_config_set_wrap(&config, 255);           // Set wrap to 255 for 8-bit resolution
    pwm_init(slice_num, &config, true);          // Initialize and enable PWM
    pwm_set_gpio_level(CONSOLE_LIGHTING_LED, 0); // Start with LED off

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);  
    printf("Core 1 task launched!\n");


    DcsBios::setup();  // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true) {
        DcsBios::loop(); // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}

//DCS-BIOS FUNCTIONS HERE
// DCS-BIOS input callback for console brightness LED
void onPltIntLightConsoleBrightnessChange(unsigned int newValue)
{
    extern unsigned int lastBrightness;
    if (abs((int)newValue - (int)lastBrightness) >= CONSOLE_DIMMING_DEADZONE)
    {
        lastBrightness = newValue;
        uint16_t pwmVal = newValue >> 8; // Map 0–65535 to 0–255
        pwm_set_gpio_level(CONSOLE_LIGHTING_LED, pwmVal);
    }
}
DcsBios::IntegerBuffer pltIntLightConsoleBrightnessBuffer(0x2d6e, 0xffff, 0, onPltIntLightConsoleBrightnessChange);
