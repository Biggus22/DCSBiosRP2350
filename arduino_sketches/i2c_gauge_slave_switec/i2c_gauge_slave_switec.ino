/*
 * I2C Gauge Slave — SwitecX25 Stepper
 *
 * Receives [reg][cmd][len][data][crc8] frames from Pico 2 master.
 * reg=0x01, cmd=0x01 (SET_POSITION), data=[uint16_le steps] → motor1.setPosition(target).
 *
 * Uses SwitecX25 library: https://github.com/clearwater/SwitecX25
 *
 * Wiring:
 *   A4 (SDA) → Pico 2 GP6 (level shift if Nano at 5V)
 *   A5 (SCL) → Pico 2 GP7
 *   GND      → Pico 2 GND
 *   4.7kΩ pull-ups to 3.3V on Pico side
 *   X27 coils on pins 4,5,6,7
 *
 * Voltage: RP2350 is 3.3V. Nano at 5V needs level shifter on SDA/SCL.
 */

#include <Wire.h>
#include <SwitecX25.h>

#define I2C_SLAVE_ADDRESS 0x08
#define STEPS (315*3)  // 945 steps, 315° at 1/3 resolution

SwitecX25 motor1(STEPS, 4, 5, 6, 7);

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

static uint8_t frame[254];
static uint8_t frameLen = 0;

// Deferred homing flag — motor1.zero() is blocking, so we run it in loop()
// instead of the I2C ISR. Master sends cmd 0x03 (HOME_SWEEP) to trigger.
static bool homeRequested = false;

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

    if (reg == 0x01 && cmd == 0x01 && len == 2) {
        // SET_POSITION: data = uint16_le DCS-BIOS raw value (0-65535)
        uint16_t rawValue = frame[3] | (frame[4] << 8);
        uint16_t target = (uint16_t)(((uint32_t)rawValue * STEPS) / 65535);
        if (target > STEPS) target = STEPS;
        motor1.setPosition(target);
    } else if (reg == 0x01 && cmd == 0x03 && len == 0) {
        // HOME_SWEEP: defer to loop() — zero() is blocking
        homeRequested = true;
    }
}

void setup() {
    init_crc8_table();
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
    motor1.zero();  // home against mechanical stops on power-up
}

void loop() {
    if (homeRequested) {
        motor1.zero();
        homeRequested = false;
    }
    motor1.update();
}