#include "DcsBios.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "internal/FoxConfig.h"

// =============================================================================
// Arduino DCS-BIOS RS485 Master Example for Raspberry Pi Pico/Pico 2
// =============================================================================
//
// This example demonstrates using a Pico/Pico 2 as a DCS-BIOS RS485 master
// (host) that communicates with Arduino-based DCS-BIOS slaves.
//
// WHAT THIS DOES:
// ---------------
// 1. Receives DCS-BIOS data from PC via USB CDC
// 2. Broadcasts DCS-BIOS updates to all Arduino slaves via RS485
// 3. Polls Arduino slaves for button presses, encoder changes, etc.
// 4. Forwards slave responses back to PC via USB CDC
//
// HARDWARE REQUIREMENTS:
// ----------------------
// - Raspberry Pi Pico or Pico 2
// - RS485 transceiver module (e.g., MAX485, MAX487, SP485, SN65HVD72)
// - Arduino slave(s) running DcsBios::RS485Slave sketches
// - Twisted pair cable for RS485 bus (A/B differential pair)
// - 120Ω termination resistors at both ends of RS485 bus
//
// WIRING (Example for Pico 2):
// ----------------------------
//   Pico GPIO 0 (UART0 TX)  -> RS485 transceiver DI (Driver Input)
//   Pico GPIO 1 (UART0 RX)  -> RS485 transceiver RO (Receiver Output)
//   Pico GPIO 2             -> RS485 transceiver DE+/RE (Driver/Receiver Enable)
//   Pico VBUS (5V)          -> RS485 transceiver VCC
//   Pico GND                -> RS485 transceiver GND
//
//   RS485 transceiver A     -> RS485 bus A (twisted pair +)
//   RS485 transceiver B     -> RS485 bus B (twisted pair -)
//
// Note: Pin assignments are defined in FoxConfig.h as UART0_TX, UART0_RX, RS485_EN
//
// RS485 BUS TOPOLOGY:
// -------------------
//   [120Ω] -- Arduino Slave 1 -- Arduino Slave 2 -- ... -- Pico Master -- [120Ω]
//             (address 1)        (address 2)                 (master)
//
// - Each Arduino slave must have a unique address (1-126)
// - Termination resistors (120Ω) at both ends of bus
// - Star topology is NOT recommended (use daisy-chain)
// - Maximum cable length: ~1200m @ 250kbaud (shorter is better)
//
// ARDUINO SLAVE SETUP:
// --------------------
// Each Arduino slave must:
// 1. Include DcsBios.h from Arduino DCS-BIOS library
// 2. Call DcsBios::setup() in setup()
// 3. Call DcsBios::loop() in loop()
// 4. Use DcsBios::RS485Slave rs485(address, TXENABLE_PIN) as serial backend
//
// Example Arduino slave sketch:
//   #define DCSBIOS_RS485_SLAVE 1  // Slave address (1-126)
//   #define TXENABLE_PIN 2         // Pin connected to DE+/RE of MAX485
//   #include <DcsBios.h>
//
//   void setup() {
//     DcsBios::setup();
//   }
//
//   void loop() {
//     DcsBios::loop();
//   }
//
//   // Add your DCS-BIOS controls here (buttons, encoders, etc.)
//   DcsBios::Switch2Pos masterArm("MASTER_ARM_SW", 3);
//   // ... etc ...
//
// COMPILE-TIME CONFIGURATION:
// ---------------------------
// This example requires DCSBIOS_RS485_ARDUINO to be defined at compile time.
// In CMakeLists.txt:
//   add_compile_definitions(DCSBIOS_RS485_ARDUINO)
//
// Optional tuning (in CMakeLists.txt or before including DcsBios.h):
//   DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US=25000    (polling rate)
//   DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US=8000  (response timeout)
//   DCSBIOS_RS485_ARDUINO_MAX_MISSES=3              (timeout threshold)
//   DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY=1             (active-only polling)
//
// BAUD RATE:
// ----------
// Arduino DCS-BIOS library uses 250000 baud 8N1 for RS485.
// DO NOT change this unless you also modify the Arduino library.
//
// TROUBLESHOOTING:
// ----------------
// - No slaves responding: Check wiring, termination, baud rate, slave addresses
// - Intermittent errors: Check cable quality, length, termination
// - Lag on Arduino slave: Increase poll interval (25ms -> 50ms)
// - Bus collisions: Ensure all slaves have unique addresses
//
// =============================================================================

#ifndef DCSBIOS_RS485_ARDUINO
#warning "This example expects DCSBIOS_RS485_ARDUINO to be defined at compile time."
#warning "Add 'add_compile_definitions(DCSBIOS_RS485_ARDUINO)' to CMakeLists.txt"
#endif

int main() {
    // Initialize USB CDC for communication with PC
    stdio_init_all();
    sleep_ms(2000); // Wait for USB enumeration

    // Print startup banner with configuration details
    printf("\n");
    printf("==============================================\n");
    printf("  DCS-BIOS Arduino RS485 Master\n");
    printf("  Raspberry Pi Pico/Pico 2\n");
    printf("==============================================\n");
    printf("RS485 Configuration:\n");
    printf("  UART: UART0\n");
    printf("  TX Pin: GPIO%d\n", UART0_TX);
    printf("  RX Pin: GPIO%d\n", UART0_RX);
    printf("  TXEN Pin: GPIO%d\n", RS485_EN);
    printf("  Baud Rate: 250000 (8N1)\n");
    printf("==============================================\n");

    // Initialize RS485 UART
    // Baud rate: 250000 (Arduino DCS-BIOS standard)
    // Format: 8 data bits, no parity, 1 stop bit
    DcsBios::init_rs485_uart(uart0, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Launch Core 1 task (RS485/USB bridge and polling loop)
    // Core 1 handles:
    // - USB CDC RX -> RS485 broadcast to slaves
    // - Polling Arduino slaves
    // - RS485 RX -> USB CDC TX to PC
    multicore_launch_core1(DcsBios::core1_task);

    // Initialize DCS-BIOS framework on Core 0
    // Core 0 handles:
    // - DCS-BIOS protocol parsing
    // - Local controls (if any)
    DcsBios::setup();

    printf("Core 0: DCS-BIOS initialized\n");
    printf("Core 1: RS485 polling active\n");
    printf("\nReady. Polling Arduino slaves...\n");
    printf("==============================================\n");

    // Main loop: run DCS-BIOS framework
    while (true) {
        DcsBios::loop();
        sleep_ms(1); // Brief yield to allow USB CDC processing
    }

    return 0;
}
