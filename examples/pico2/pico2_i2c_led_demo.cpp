#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/I2cBusHwMaster.h"
#include "internal/I2cProtocol.h"
#include "internal/Addresses.h"

using namespace DcsBios;

// I2C slave configuration
static constexpr uint8_t SLAVE_ADDR_LED_GAUGE = 0x08;  // Nano I2C address for the LED gauge slave
static constexpr uint8_t REG_ID_CONSOLE_LIGHT = 0x01;  // register ID for console lighting gauge data

static I2cBusHwMaster i2cBus;

static I2cOutputListener consoleGauge(
    F_4E_PLT_INT_LIGHT_CONSOLE_A,
    &i2cBus,
    SLAVE_ADDR_LED_GAUGE,
    REG_ID_CONSOLE_LIGHT,
    I2cOutputListener::LED_BRIGHTNESS
);

int main() {
    stdio_init_all();
    initHeartbeat(HEARTBEAT_LED);

    uint8_t boardAddress = 0xF;
    currentBoardMode = determineBoardMode(boardAddress);

    init_rs485_uart(uart0, UART0_TX, UART0_RX, RS485_EN, 250000);
    multicore_launch_core1(core1_task);
    setup();

    while (true) {
        loop();
        updateHeartbeat();
        sleep_us(10);
    }
}
