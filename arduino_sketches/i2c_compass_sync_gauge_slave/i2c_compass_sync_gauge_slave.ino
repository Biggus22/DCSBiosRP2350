/*
 * I2C Compass Sync Gauge Slave — SwitecX25 (X27) stepper
 *
 * F-4E compass control panel gauge slave. Receives
 * [reg][cmd][len][data][crc8] frames from the RP2350-Zero master.
 * reg=REG_GAUGE, cmd=CMD_SET_POSITION, data=[uint16_le steps] -> setPosition(steps).
 *
 * This is a dumb executor: the DCS value -> step mapping (full-scale span,
 * direction and jitter deadband) lives on the RP2350 master, so the gauge can
 * be tuned over the master's USB link without another UPDI session. The slave
 * only homes against the mechanical stop and clamps to the physical travel.
 *
 * Uses SwitecX25 library: https://github.com/clearwater/SwitecX25
 *
 * ATtiny1614 wiring (megaTinyCore):
 *   PA1 (SDA) -> RP2350-Zero GP6   (TWI0 pinswap-1)
 *   PA2 (SCL) -> RP2350-Zero GP7
 *   GND       -> RP2350-Zero GND
 *   4.7k pull-ups to 3.3V on the RP2350 side
 *   X27 coils: PA4-PA7
 *   PA0: UPDI programming (onboard inline resistor)
 *
 *   PB0 is wired for analog input but unused on this panel; the pin config
 *   is kept commented out below.
 *
 * Voltage: RP2350 is 3.3V. ATtiny1614 at 3.3V connects directly, no level shifter.
 * Swapping any two coil pins reverses the motor direction.
 *
 * Homing: no hall sensor on this gauge. At boot (or on CMD_HOME_SWEEP) the
 * motor runs SwitecX25::zero(), which sweeps the full travel down onto the
 * X27's mechanical limiter; that stop is the position reference.
 *
 * Frame buffer sized to the Wire rx buffer (32 bytes).
 */

#include <Wire.h>
#include <SwitecX25.h>
#include "i2c_slave_defs.h"

#define STEPS 945  // standard X27.168 with travel limiter (~315 deg at 3 steps/deg)

// Dumb executor: the DCS value -> step mapping (span, direction, jitter
// deadband) is done on the RP2350 master. CMD_SET_POSITION carries an absolute
// step target, clamped here to the physical travel.
SwitecX25 motor1(STEPS, PIN_PA7, PIN_PA6, PIN_PA4, PIN_PA5);

static uint16_t lastTarget = 0;

static uint8_t frame[32];
static uint8_t frameLen = 0;

// Deferred homing flag — the sweep is blocking, so we run it in loop()
// instead of the I2C ISR. The master sends CMD_HOME_SWEEP (0x03) to re-run it.
static bool homeRequested = false;

// --- Mechanical-stop homing (run from loop()) ---
// No hall sensor on this gauge. SwitecX25::zero() starts its counter at the far
// end and steps all the way down, so the needle is guaranteed to be resting on
// the mechanical limiter when the sweep ends. That stop is step 0.
static void homeToMechanicalStop() {
    motor1.zero();       // sweep down onto the mechanical stop
    motor1.currentStep = 0;
    motor1.targetStep = 0;
    motor1.stopped = true;
    motor1.dir = 0;
    motor1.vel = 0;
    lastTarget = 0;
}

// --- SET_POSITION handler ---
static void handleSetPosition(uint16_t target) {
    if (target > STEPS) target = STEPS;  // never drive past the physical travel
    if (target == lastTarget) return;
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
        // HOME_SWEEP: defer to loop() — the sweep is blocking
        homeRequested = true;
    }
}

void setup() {
    // PB0 is wired for analog input but unused on this panel:
    // pinMode(PIN_PB0, INPUT);
    Wire.swap(1);  // TWI0 pinswap-1: SDA=PA1, SCL=PA2
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
    homeRequested = true;  // mechanical-stop home at boot
}

void loop() {
    if (homeRequested) {
        homeToMechanicalStop();
        homeRequested = false;
    }
    motor1.update();
}
