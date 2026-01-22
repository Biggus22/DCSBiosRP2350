#include "DcsBios.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "internal/FoxConfig.h"

// Example: Pico as RS485 master compatible with Arduino DCS-BIOS slaves.
//
// Key points:
// - Enable Arduino framing by defining DCSBIOS_RS485_ARDUINO at compile time.
// - RS485 UART is set to 250000 8N1.
// - Core 1 runs the RS485/USB bridge and polling loop.
//
// This example assumes:
// - UART0_TX/UART0_RX/RS485_EN pins are defined in FoxConfig.h for your board.
// - RS485 transceiver /RE+DE is driven by RS485_EN.

#ifndef DCSBIOS_RS485_ARDUINO
#warning "This example expects DCSBIOS_RS485_ARDUINO to be defined at compile time."
#endif

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for USB to enumerate

    printf("DCS-BIOS Arduino RS485 Master Starting...\n");
    printf("RS485: UART0 TX=GPIO%d RX=GPIO%d EN=GPIO%d @ 250000 baud\n", UART0_TX, UART0_RX, RS485_EN);

    // Initialize RS485 UART @ 250000 baud (Arduino DCS-BIOS standard)
    DcsBios::init_rs485_uart(uart0, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Start core 1 for DCS-BIOS host/RS485 tasks
    multicore_launch_core1(DcsBios::core1_task);

    // Initialize DCS-BIOS framework on core 0
    DcsBios::setup();
    printf("Core 0 ready. Polling Arduino slaves...\n");

    while (true) {
        DcsBios::loop();
        sleep_ms(1);
    }

    return 0;
}
