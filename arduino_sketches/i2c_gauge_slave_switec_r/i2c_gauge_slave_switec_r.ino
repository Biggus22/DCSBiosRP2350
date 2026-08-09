/*
 * I2C Gauge Slave — SwitecX25 Stepper (RIGHT engine fuel flow)
 *
 * Receives [reg][cmd][len][data][crc8] frames from Pico 2 master.
 * reg=REG_GAUGE, cmd=CMD_SET_POSITION, data=[uint16_le steps] → motor1.setPosition(target).
 * reg=REG_GAUGE, cmd=CMD_SET_BACKLIGHT, data=[0-255] → analogWrite(BACKLIGHT_PIN).
 *
 * Uses SwitecX25 library: https://github.com/clearwater/SwitecX25
 *
 * This board is the second gauge on the bus: I2C address 0x09. The first
 * (left fuel flow) slave uses address 0x08. Both share the same SDA/SCL lines
 * and each has its own ATtiny1614 + X27 motor + hall sensor.
 *
 * ATtiny1614 wiring (megaTinyCore):
 *   PA1 (SDA) → Pico 2 GP6   (TWI0 pinswap-1)
 *   PA2 (SCL) → Pico 2 GP7
 *   GND       → Pico 2 GND
 *   4.7kΩ pull-ups to 3.3V on Pico side
 *   X27 coils: coil1=PA7, coil2=PA6, coil3=PA4, coil4=PA5
 *   Backlight (red PWM LED): PB1
 *   Hall zero sensor (A3144, open-collector, active-low): PA3
 *
 * Voltage: RP2350 is 3.3V. ATtiny1614 at 3.3V connects directly, no level shifter.
 * Swapping any two coil pins reverses motor direction.
 *
 * Homing: there is no physical stop — the A3144 hall switch is the only
 * position reference. The motor sweeps across the full ~345° travel until the
 * sensor triggers; that point becomes step 0 (or STEPS when GAUGE_SCALE_REVERSED).
 * Runs at boot and on CMD_HOME_SENSOR (0x04) / CMD_HOME_SWEEP (0x03). Searches
 * the opposite direction if the first sweep finds nothing.
 *
 * Frame buffer sized to the Wire rx buffer (32 bytes).
 */

#include <Wire.h>
#include <SwitecX25.h>

#define I2C_SLAVE_ADDRESS 0x09
#include "i2c_slave_defs.h"

#define STEPS 1035  // ~345° travel at 3 steps/° (1/3 step resolution)
#define BACKLIGHT_PIN PIN_PB1
#define BACKLIGHT_DEFAULT 127

// Hall zero sensor (A3144) on PA3.
// A3144 output is open-collector: pulled LOW when a south pole is nearby.
#define HALL_PIN PIN_PA3

// Homing sweep direction: 1 = stepDown toward scale zero, -1 = stepUp.
// Runtime variable so homing can retry in the opposite direction if the
// first sweep does not cross the sensor.
static int8_t homeDir = 1;

// Homing step period (us) — mirrors SwitecX25 zero()'s RESET_STEP_MICROSEC.
#define HOME_STEP_US 800

// PCB coil wiring is fixed and stepUp() swings the needle the wrong way on the
// dial. When 1: increasing fuel flow drives stepDown() (opposite physical
// rotation) and the hall zero is assigned to STEPS (top of range) so zero flow
// still rests on the magnet. Flip to 0 if a gauge is wired the correct way.
#define GAUGE_SCALE_REVERSED 1

SwitecX25 motor1(STEPS, PIN_PA7, PIN_PA6, PIN_PA4, PIN_PA5);

// A3144 is active-low (open-collector + pull-up): LOW = magnet present.
static bool hallTriggered() {
    return digitalRead(HALL_PIN) == LOW;
}

// --- Direction filter ---
// Ignores small direction reversals to prevent SwitecX25 acceleration
// ramp interruptions from DCS-BIOS value jitter. A reversal of up to
// DIR_FILTER_THRESHOLD steps is treated as noise and ignored; larger
// reversals are accepted as genuine direction changes.
#define DIR_FILTER_THRESHOLD 10

static int lastDirection = 0;   // 0=unknown, 1=forward, -1=backward
static uint16_t lastTarget = 0;

static uint8_t frame[32];
static uint8_t frameLen = 0;

// Deferred homing flag — hall homing is blocking, so we run it in loop()
// instead of the I2C ISR. Master sends cmd 0x03 (HOME_SWEEP) or 0x04
// (HOME_SENSOR) to trigger. There is no physical stop on this gauge, so both
// commands home against the A3144 hall switch.
static bool homeRequested = false;

// --- Hall-sensor homing (non-blocking, run from loop()) ---
// No physical stop on this gauge, so the A3144 is the only zero reference.
// Sweeps the full travel in one direction; if the sensor is not found by the
// time the counter clamps at an end, it sweeps back the other way. On trigger,
// that point becomes step 0 (or STEPS when GAUGE_SCALE_REVERSED). If neither
// direction finds the magnet, the motor is left stopped without a reference.
static void setHomePosition() {
#if GAUGE_SCALE_REVERSED
    motor1.currentStep = STEPS;
    motor1.targetStep = STEPS;
#else
    motor1.currentStep = 0;
    motor1.targetStep = 0;
#endif
    motor1.stopped = true;
    motor1.dir = 0;
    motor1.vel = 0;
}

static void homeToHallSensor() {
    // Already triggered (needle resting on the magnet) — reset immediately.
    if (hallTriggered()) {
        setHomePosition();
        return;
    }

    // Try the primary direction, then the reverse, until the magnet is found.
    for (int attempt = 0; attempt < 2; attempt++) {
        // Start from the far end so the sweep covers the full travel.
        if (homeDir > 0) {
            motor1.currentStep = STEPS;  // sweep down toward scale zero
        } else {
            motor1.currentStep = 0;      // sweep up toward scale zero
        }

        unsigned long t0 = millis();
        unsigned long timeout = (unsigned long)STEPS * HOME_STEP_US / 1000UL + 1000UL;
        while (!hallTriggered()) {
            if (homeDir > 0) {
                motor1.stepDown();
            } else {
                motor1.stepUp();
            }
            delayMicroseconds(HOME_STEP_US);

            // Counter clamped at an end without triggering — stop this sweep.
            if ((homeDir > 0 && motor1.currentStep == 0) ||
                (homeDir < 0 && motor1.currentStep == STEPS)) {
                break;
            }
            if (millis() - t0 >= timeout) break;
        }

        if (hallTriggered()) {
            setHomePosition();
            return;
        }
        homeDir = -homeDir;  // reverse direction for the next attempt
    }
}

// --- SET_POSITION handler (split out of ISR for clarity) ---
static void handleSetPosition(uint16_t rawValue) {
#if GAUGE_SCALE_REVERSED
    uint16_t target = STEPS - (uint16_t)(((uint32_t)rawValue * STEPS) / 65535);
#else
    uint16_t target = (uint16_t)(((uint32_t)rawValue * STEPS) / 65535);
#endif
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
}

static void onReceive(int howMany) {
    frameLen = 0;
    while (Wire.available() && frameLen < sizeof(frame)) {
        frame[frameLen++] = (uint8_t)Wire.read();
    }
    if (frameLen < 4) return;

    uint8_t reg = frame[FRAME_IDX_REG];
    uint8_t cmd = frame[FRAME_IDX_CMD];
    uint8_t len = frame[FRAME_IDX_LEN];
    if (len > FRAME_MAX_PAYLOAD) return;
    if (frameLen != (uint8_t)(len + FRAME_OVERHEAD)) return;

    if (crc8_calc(frame, frameLen - 1) != frame[frameLen - 1]) return;

    if (reg == REG_GAUGE && cmd == CMD_SET_POSITION && len == 2) {
        // Reassemble 16-bit little-endian value from two bytes (low byte, high byte)
        uint16_t rawValue = frame[FRAME_IDX_DATA] | (frame[FRAME_IDX_DATA + 1] << 8);
        handleSetPosition(rawValue);
    } else if (reg == REG_GAUGE && cmd == CMD_HOME_SWEEP && len == 0) {
        // HOME_SWEEP: defer to loop() — hall homing is blocking
        homeRequested = true;
    } else if (reg == REG_GAUGE && cmd == CMD_HOME_SENSOR) {
        // HOME_SENSOR: defer to loop() — hall homing is blocking
        homeRequested = true;
    } else if (reg == REG_GAUGE && cmd == CMD_SET_BACKLIGHT && len == 1) {
        analogWrite(BACKLIGHT_PIN, frame[FRAME_IDX_DATA]);
    }
}

void setup() {
    pinMode(BACKLIGHT_PIN, OUTPUT);
    analogWrite(BACKLIGHT_PIN, BACKLIGHT_DEFAULT);
    pinMode(HALL_PIN, INPUT_PULLUP);
    Wire.swap(1);  // TWI0 pinswap-1: SDA=PA1, SCL=PA2
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
    homeRequested = true;  // hall-zero at boot
}

void loop() {
    if (homeRequested) {
        homeToHallSensor();
        homeRequested = false;
        lastTarget = 0;
        lastDirection = 0;
    }
    motor1.update();
}
