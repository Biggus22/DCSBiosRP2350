#include "pico/stdlib.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"

// Test pins for the 3-position momentary switch
// Connect your switch between these pins and ground
const uint8_t testSwitchPins[2] = {8, 9}; // Pin 8 = Down, Pin 9 = Up

// Create an instance of the 3-position momentary to 2-position latching switch
DcsBios::Switch3PosMomentary2PosLatching testSwitch("TEST_SWITCH", testSwitchPins);

uart_inst_t *rs485_uart = uart0;

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    // Set up board mode (using default address)
    uint8_t boardAddress = 0xF;
    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    DcsBios::currentBoardMode = board;
    
    // Initialize RS485 communication
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);
    
    // Launch core 1 task
    multicore_launch_core1(DcsBios::core1_task);
    
    // Initialize DCS-BIOS
    DcsBios::setup();
    
    // Main loop
    while (true) {
        DcsBios::loop();
        sleep_ms(10); // 100Hz update rate
    }
    
    return 0;
}