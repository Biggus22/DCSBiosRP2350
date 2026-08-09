#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/I2cBusHwMaster.h"
#include "internal/Addresses.h"

using namespace DcsBios;

// I2C slave configuration
static constexpr uint8_t SLAVE_ADDR_LEFT_GAUGE = 0x08;   // I2C address of left fuel flow gauge slave
static constexpr uint8_t SLAVE_ADDR_RIGHT_GAUGE = 0x09;  // I2C address of right fuel flow gauge slave
static constexpr uint8_t REG_ID_GAUGE = 0x01;            // register ID for gauge data

static I2cBusHwMaster i2cBus;

// DCS-BIOS: F-4E pilot fuel flow L → remote SwitecX25 gauge slave (left).
// F_4E_PLT_FUEL_FLOW_L (0x2C8E) → 16-bit step count.
static I2cOutputListener flowGaugeLeft(
    F_4E_PLT_FUEL_FLOW_L_A,
    &i2cBus,
    SLAVE_ADDR_LEFT_GAUGE,
    REG_ID_GAUGE,
    I2cOutputListener::STEPS_16BIT
);

// DCS-BIOS: F-4E pilot fuel flow R → remote SwitecX25 gauge slave (right).
// F_4E_PLT_FUEL_FLOW_R (0x2C90) → 16-bit step count.
static I2cOutputListener flowGaugeRight(
    F_4E_PLT_FUEL_FLOW_R_A,
    &i2cBus,
    SLAVE_ADDR_RIGHT_GAUGE,
    REG_ID_GAUGE,
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