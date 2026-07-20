/*
 * Shared I2C slave definitions — protocol constants, CRC-8 utilities
 *
 * Included by all Arduino I2C slave sketches that communicate with the
 * DCS-BIOS RP2350 I2C master. Must stay in sync with src/internal/I2cFrame.h
 * and src/internal/I2cCommand.h in the main project.
 *
 * Frame wire format: [reg:1][cmd:1][len:1][data:len][crc8:1]
 * CRC-8: Dallas/Maxim 1-Wire, poly 0x07, init 0x00
 */
#ifndef I2C_SLAVE_DEFS_H
#define I2C_SLAVE_DEFS_H

#include <stdint.h>

// --- I2C frame field offsets (matches I2cFrame.h) ---
#define FRAME_IDX_REG     0
#define FRAME_IDX_CMD     1
#define FRAME_IDX_LEN     2
#define FRAME_IDX_DATA    3

// --- I2C commands (matches I2cCmd enum in I2cCommand.h) ---
#define CMD_SET_POSITION  0x01
#define CMD_HOME_SWEEP    0x03

// --- Register IDs ---
#define REG_GAUGE         0x01

// --- I2C slave address (default; override in sketch if needed) ---
#define I2C_SLAVE_ADDRESS 0x08

// --- CRC-8 (Dallas, poly 0x07, init 0x00) ---
static uint8_t crc8_table[256];

static void init_crc8_table() {
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
        crc8_table[i] = crc;
    }
}

static inline uint8_t crc8_calc(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

#endif // I2C_SLAVE_DEFS_H
