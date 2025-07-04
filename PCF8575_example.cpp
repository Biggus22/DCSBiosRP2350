#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/PCF8575.h"

void init_i2c0()
{
    i2c_init(i2c0, 400 * 1000); // 400kHz
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA);
    gpio_pull_up(I2C0_SCL);
}

PCF8575 pcf8575(i2c0, 0x20); // Default address, can be changed via A0, A1, A2 pins

int main()
{
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000); // Wait for USB to be ready


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

    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");

    init_i2c0();
    sleep_ms(500); // Allow time for I2C initialization
    pcf8575.begin(); // Initialize the PCF8575

    if (!pcf8575.begin())
    {
        printf("Failed to initialize PCF8575!\n");
       
        while (1)
        {
            sleep_ms(1000);
        }
    }
    else
    {
        printf("PCF8575 initialized successfully.\n");
    }

    //Define all your DCS-BIOS controls.
    DcsBios::Switch2Pos pltCockpitHelmet("PLT_COCKPIT_HELMET", 15, true); // switch on GPIO 15, reversed logic
    DcsBios::PCF8575Switch2Pos pltAdiReferenceSystem("PLT_ADI_REFERENCE_SYSTEM", &pcf8575, 0, true, 5); // switch on PCF8575 pin 0, reversed logic, debounce delay of 5ms
 
    DcsBios::setup();
    printf("DCS-BIOS setup complete!\n");

    while (true)
    {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        sleep_us(10);
    }
}