/*
 * I2C Hall Switch Test — LED Flash by Position
 *
 * RP2350 master reads a CH32V003 hall switch slave and flashes
 * the onboard LED for N seconds where N = active switch number (1-8).
 * No switch active (0) = LED off.
 *
 * Uses raw Pico SDK I2C (no DCS-BIOS frame protocol).
 *
 * Wiring:
 *   GP6 (SDA) → CH32V003 D3 (PC1)
 *   GP7 (SCL) → CH32V003 D4 (PC2)
 *   GND       → CH32V003 GND
 *   4.7kΩ pull-ups to 3.3V on both sides
 */

#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/Addresses.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"

using namespace DcsBios;

// I2C configuration
static constexpr uint I2C_SDA = 6;   // GP6
static constexpr uint I2C_SCL = 7;   // GP7
static constexpr uint32_t I2C_BAUD = 100000;  // 100 kHz
static constexpr uint8_t SLAVE_ADDR = 0x08;

// LED
static constexpr uint STATUS_LED = 15;  // GP15 — external LED

// Timing
static constexpr uint32_t POLL_INTERVAL_MS = 50;
static constexpr uint32_t I2C_TIMEOUT_MS = 10;

static void initI2c() {
    i2c_init(i2c1, I2C_BAUD);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

static void i2c_scan() {
    printf("I2C scan (100 kHz, GP6/GP7):\n");
    uint8_t found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        uint8_t rxbuf;
        absolute_time_t deadline = make_timeout_time_ms(10);
        int ret = i2c_read_blocking_until(i2c1, addr, &rxbuf, 1, false, deadline);
        if (ret == PICO_ERROR_TIMEOUT) {
            // NAK — device not present, retry once
            deadline = make_timeout_time_ms(10);
            ret = i2c_read_blocking_until(i2c1, addr, &rxbuf, 1, false, deadline);
        }
        if (ret >= 0) {
            printf("  0x%02X ACK\n", addr);
            found++;
        }
    }
    if (found == 0) {
        printf("  No devices found\n");
    } else {
        printf("  %d device(s) found\n", found);
    }
}

static int readSlave(uint8_t *out) {
    uint8_t buf[1];
    absolute_time_t deadline = make_timeout_time_ms(I2C_TIMEOUT_MS);
    int ret = i2c_read_blocking_until(i2c1, SLAVE_ADDR, buf, 1, false, deadline);
    if (ret == 1) {
        *out = buf[0];
        return 0;
    }
    return -1;
}

int main() {
    stdio_init_all();
    initHeartbeat(HEARTBEAT_LED);

    // Wait for USB serial
    sleep_ms(5000);
    printf("\n=== I2C Hall Switch Test ===\n");
    printf("Pico 2 | I2C1 @ 100 kHz | GP6=SDA, GP7=SCL\n");
    printf("Expected slave: CH32V003 at 0x08\n\n");

    initI2c();
    i2c_scan();
    printf("\nPolling started...\n");

    // LED setup
    gpio_init(STATUS_LED);
    gpio_set_dir(STATUS_LED, GPIO_OUT);

    uint8_t boardAddress = 0xF;
    currentBoardMode = determineBoardMode(boardAddress);

    init_rs485_uart(uart0, UART0_TX, UART0_RX, RS485_EN, 250000);
    multicore_launch_core1(core1_task);
    setup();

    // State tracking for transition detection
    uint8_t prevSwitch = 0;
    uint32_t eventCount = 0;

    while (true) {
        loop();
        updateHeartbeat();

        uint8_t switchNum = 0;
        readSlave(&switchNum);

        // LED on when any switch active, off when none
        gpio_put(STATUS_LED, (switchNum >= 1 && switchNum <= 8) ? 1 : 0);

        if (switchNum != prevSwitch) {
            uint32_t t = time_us_32() / 1000000;
            if (switchNum >= 1 && switchNum <= 8) {
                printf("[%u] +%u HALLSW%d ON\n", t, ++eventCount, switchNum);
            } else if (prevSwitch >= 1 && prevSwitch <= 8) {
                printf("[%u] #%u HALLSW%d OFF\n", t, eventCount, prevSwitch);
            }
            prevSwitch = switchNum;
        }

        sleep_ms(POLL_INTERVAL_MS);
    }
}
