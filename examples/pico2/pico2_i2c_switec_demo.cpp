#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/I2cBusHwMaster.h"
#include "internal/Addresses.h"

using namespace DcsBios;

static I2cBusHwMaster i2cBus;

// DCS-BIOS: F-4E pilot O2 flow → remote SwitecX25 gauge slave.
// F_4E_PLT_O2_FLOW_A (0x2B32) → slave 0x08, reg 0x01, 16-bit step count.
static I2cOutputListener o2Gauge(
    F_4E_PLT_O2_FLOW_A,
    &i2cBus,
    0x08,
    0x01,
    I2cOutputListener::STEPS_16BIT
);

// DCS-BIOS: F-4E console lighting → gauge backlight (red PWM LED on PB1).
// F_4E_PLT_INT_LIGHT_CONSOLE_A (0x2D8A) → slave 0x08, reg 0x01, 8-bit brightness.
static I2cOutputListener consoleBacklight(
    F_4E_PLT_INT_LIGHT_CONSOLE_A,
    &i2cBus,
    0x08,
    0x01,
    I2cOutputListener::BACKLIGHT
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