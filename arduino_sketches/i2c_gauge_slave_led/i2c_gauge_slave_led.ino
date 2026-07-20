/*
 * I2C Gauge Slave — LED Brightness
 *
 * Receives [reg][cmd][len][data][crc8] frames from Pico 2 master.
 * reg=0x01, cmd=0x01 (SET_POSITION), data=[0-255 brightness] → PWM on LED_BUILTIN.
 *
 * Wiring:
 *   A4 (SDA) → Pico 2 GP6 (level shift if Nano at 5V)
 *   A5 (SCL) → Pico 2 GP7
 *   GND      → Pico 2 GND
 *   4.7kΩ pull-ups to 3.3V on Pico side
 *
 * Voltage: RP2350 is 3.3V. Nano at 5V needs level shifter on SDA/SCL.
 */

#include <Wire.h>
#include <Arduino.h>

// I2C frame field offsets — matches the master's I2cFrame wire format
#define FRAME_IDX_REG     0
#define FRAME_IDX_CMD     1
#define FRAME_IDX_LEN     2
#define FRAME_IDX_DATA    3

// I2C commands — must match I2cCmd enum in I2cCommand.h
#define CMD_SET_POSITION  0x01

// Register ID for this gauge (matches master's regId)
#define REG_GAUGE         0x01

#define I2C_SLAVE_ADDRESS 0x08
#define LED_PIN LED_BUILTIN

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

static uint8_t frame[254];
static uint8_t frameLen = 0;

static void onReceive(int howMany) {
    frameLen = 0;
    while (Wire.available() && frameLen < sizeof(frame)) {
        frame[frameLen++] = (uint8_t)Wire.read();
    }
    if (frameLen < 4) return;

    uint8_t reg = frame[0];
    uint8_t cmd = frame[1];
    uint8_t len = frame[2];
    if (frameLen != (uint8_t)(len + 4)) return;

    uint8_t crc = 0;
    for (uint8_t i = 0; i < frameLen - 1; i++) {
        crc = crc8_table[crc ^ frame[i]];
    }
    if (crc != frame[frameLen - 1]) return;

    if (reg == REG_GAUGE && cmd == CMD_SET_POSITION && len == 1) {
        analogWrite(LED_PIN, frame[FRAME_IDX_DATA]);
    }
}

void setup() {
    init_crc8_table();
    pinMode(LED_PIN, OUTPUT);
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
}

void loop() {
}
