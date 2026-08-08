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
 *
 * ATtiny804 note: frame buffer sized to the Wire rx buffer (32 bytes),
 * not 254 — the ATtiny804 has 512 bytes SRAM.
 */

#include <Wire.h>
#include <Arduino.h>
#include "i2c_slave_defs.h"

#define LED_PIN LED_BUILTIN

static uint8_t frame[32];
static uint8_t frameLen = 0;

static void onReceive(int howMany) {
    frameLen = 0;
    while (Wire.available() && frameLen < sizeof(frame)) {
        frame[frameLen++] = (uint8_t)Wire.read();
    }
    if (frameLen < 4) return;

    uint8_t reg = frame[FRAME_IDX_REG];
    uint8_t cmd = frame[FRAME_IDX_CMD];
    uint8_t len = frame[FRAME_IDX_LEN];
    if (frameLen != (uint8_t)(len + 4)) return;

    if (crc8_calc(frame, frameLen - 1) != frame[frameLen - 1]) return;

    if (reg == REG_GAUGE && cmd == CMD_SET_POSITION && len == 1) {
        analogWrite(LED_PIN, frame[FRAME_IDX_DATA]);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
}

void loop() {
}
