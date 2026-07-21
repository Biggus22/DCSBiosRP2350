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
#include "i2c_slave_defs.h"

#define LED_PIN LED_BUILTIN

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
    if (len > FRAME_MAX_PAYLOAD) return;
    if (frameLen != (uint8_t)(len + FRAME_OVERHEAD)) return;

    uint8_t crc = i2cFrame_crc8(frame, frameLen - 1);
    if (crc != frame[frameLen - 1]) return;

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
