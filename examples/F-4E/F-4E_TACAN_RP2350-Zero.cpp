#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/ws2812.h"
#include "internal/SSD1306.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

#define NUM_LEDS 6

WS2812 externalLeds(pio0, 0, 3, false);
SSD1306 tacanDisplay(i2c1, 0x3C);
uart_inst_t *rs485_uart = uart0;

const uint8_t pltTacanModePins[5] = {11, 8, 9, 10, 12};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 5> pltTacanMode("PLT_TACAN_MODE", pltTacanModePins);

DcsBios::RotaryEncoderT<POLL_EVERY_TIME, DcsBios::TWO_STEPS_PER_DETENT, 5> pltTacanSetOnes("PLT_TACAN_SET_ONES", "DEC", "INC", 26, 13);
DcsBios::RotaryEncoderT<POLL_EVERY_TIME, DcsBios::TWO_STEPS_PER_DETENT, 5> pltTacanSetTens("PLT_TACAN_SET_TENS", "DEC", "INC", 27, 28);

const uint8_t pltTacanSetXyPins[2] = {6, 7};
DcsBios::Switch3Pos2Pin pltTacanSetXy("PLT_TACAN_SET_XY", pltTacanSetXyPins[0], pltTacanSetXyPins[1]);

DcsBios::Switch2Pos pltTacanTest("PLT_TACAN_TEST", 5);

DcsBios::Potentiometer pltTacanVolume("PLT_TACAN_VOLUME", 29, false, 0, 4095);

DcsBios::LED pltTacanTestLight(0x2BBC, 0x8000, 4);

void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0));
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t brightness = 0;
    if (consoleBrightness <= 8) {
        uint8_t percentages[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};
        brightness = (percentages[consoleBrightness] * 255) / 100;
    } else {
        brightness = (uint8_t)((consoleBrightness * 255) / 65535);
    }
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0));
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);

void onPltTacanChannelChange(char* newValue) {
    tacanDisplay.clear();
    tacanDisplay.drawString(38, 15, newValue, 2, 2);
    tacanDisplay.display();
}
DcsBios::StringBuffer<4> pltTacanChannelBuffer(F_4E_PLT_TACAN_CHANNEL_A, onPltTacanChannelChange);

int main()
{
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);

    adc_init();

    i2c_init(i2c1, 400000);
    gpio_set_function(14, GPIO_FUNC_I2C);
    gpio_set_function(15, GPIO_FUNC_I2C);
    gpio_pull_up(14);
    gpio_pull_up(15);

    tacanDisplay.init();
    tacanDisplay.clear();
    tacanDisplay.drawString(38, 20, "----", 3, 3);
    tacanDisplay.display();

    externalLeds.begin(NUM_LEDS);

    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0));
    }
    externalLeds.show();
    sleep_ms(1000);

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

    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");

    DcsBios::setup();
    printf("DCS-BIOS setup complete!\n");
    while (true)
    {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        sleep_us(10);
    }
}
