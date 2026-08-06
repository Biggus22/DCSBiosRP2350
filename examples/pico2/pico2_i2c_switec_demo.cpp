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
static constexpr uint8_t SLAVE_ADDR_SWITEC_GAUGE = 0x08;  // Nano I2C address for the Switec gauge slave
static constexpr uint8_t REG_ID_O2_FLOW = 0x01;            // register ID for O2 flow gauge data

static I2cBusHwMaster i2cBus;

// DCS-BIOS: F-4E pilot O2 flow → remote SwitecX25 gauge slave.
// F_4E_PLT_O2_FLOW_A (0x2B32) → 16-bit step count.
static I2cOutputListener o2Gauge(
    F_4E_PLT_O2_FLOW_A,
    &i2cBus,
    SLAVE_ADDR_SWITEC_GAUGE,
    REG_ID_O2_FLOW,
    I2cOutputListener::STEPS_16BIT
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