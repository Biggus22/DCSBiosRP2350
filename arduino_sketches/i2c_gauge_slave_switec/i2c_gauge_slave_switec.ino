/*
 * I2C Gauge Slave — SwitecX25 Stepper
 *
 * Receives [reg][cmd][len][data][crc8] frames from Pico 2 master.
 * reg=0x01, cmd=0x01 (SET_POSITION), data=[uint16_le steps] → motor1.setPosition(target).
 * reg=0x01, cmd=0x09 (SET_BACKLIGHT), data=[0-255] → analogWrite(BACKLIGHT_PIN).
 *
 * Uses SwitecX25 library: https://github.com/clearwater/SwitecX25
 *
 * ATtiny1614 wiring (megaTinyCore):
 *   PA1 (SDA) → Pico 2 GP6   (TWI0 pinswap-1)
 *   PA2 (SCL) → Pico 2 GP7
 *   GND       → Pico 2 GND
 *   4.7kΩ pull-ups to 3.3V on Pico side
 *   X27 coils: coil1=PA7, coil2=PA6, coil3=PA4, coil4=PA5
 *   Backlight (red PWM LED): PB1
 *
 * Voltage: RP2350 is 3.3V. ATtiny1614 at 3.3V connects directly, no level shifter.
 * Swapping any two coil pins reverses motor direction.
 */

#include <Wire.h>
#include <SwitecX25.h>

#define I2C_SLAVE_ADDRESS 0x08
#define I2C_CMD_SET_BACKLIGHT 0x09
#define STEPS (315*3)  // 945 steps, 315° at 1/3 resolution
#define BACKLIGHT_PIN PIN_PB1
#define BACKLIGHT_DEFAULT 127

SwitecX25 motor1(STEPS, PIN_PA7, PIN_PA6, PIN_PA4, PIN_PA5);

// --- Direction filter ---
// Ignores small direction reversals to prevent SwitecX25 acceleration
// ramp interruptions from DCS-BIOS value jitter. A reversal of up to
// DIR_FILTER_THRESHOLD steps is treated as noise and ignored; larger
// reversals are accepted as genuine direction changes.
#define DIR_FILTER_THRESHOLD 10

static int lastDirection = 0;   // 0=unknown, 1=forward, -1=backward
static uint16_t lastTarget = 0;

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
        uint16_t rawValue = frame[3] | (frame[4] << 8);
        uint16_t target = (uint16_t)(((uint32_t)rawValue * STEPS) / 65535);
        if (target > STEPS) target = STEPS;

        // Direction-locked filter: prevent small direction reversals
        // from jittering the motor against the acceleration ramp.
        if (lastDirection >= 0 && target >= lastTarget) {
            lastDirection = 1;
        } else if (lastDirection <= 0 && target <= lastTarget) {
            lastDirection = -1;
        } else if (abs((int16_t)target - (int16_t)lastTarget) > DIR_FILTER_THRESHOLD) {
            lastDirection = (target > lastTarget) ? 1 : -1;
        } else {
            return; // small reversal — ignore
        }
        lastTarget = target;
        motor1.setPosition(target);
    } else if (reg == 0x01 && cmd == 0x03 && len == 0) {
        // HOME_SWEEP: defer to loop() — motor1.zero() is blocking
        homeRequested = true;
    } else if (reg == 0x01 && cmd == I2C_CMD_SET_BACKLIGHT && len == 1) {
        analogWrite(BACKLIGHT_PIN, frame[3]);
    }
}

void setup() {
    init_crc8_table();
    pinMode(BACKLIGHT_PIN, OUTPUT);
    analogWrite(BACKLIGHT_PIN, BACKLIGHT_DEFAULT);
    Wire.swap(1);  // TWI0 pinswap-1: SDA=PA1, SCL=PA2
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
    motor1.zero();  // sweep to lower stop and reset counter
}

void loop() {
    if (homeRequested) {
        motor1.zero();
        homeRequested = false;
    }
    motor1.update();
}