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

// === Homing configuration ===
//
// The SwitecX25 library's zero() function only resets the internal step
// counter — it does NOT move the motor. A homing sweep against a known
// mechanical stop is required to synchronise the counter with physical
// position.
//
// HOME_TO_UPPER = 1: home against the upper (CW/forward) stop at STEPS.
// HOME_TO_UPPER = 0: home against the lower (CCW/backward) stop at 0.
#define HOME_TO_UPPER 1

// Maximum time (ms) to wait for the motor to stall against a stop.
// STEPS = 945 with a ~2000 us step interval → ~1.9 s per full sweep.
// 3000 ms gives generous margin for mechanical settling.
#define HOME_SWEEP_MS 3000

// Uncomment HOME_BOTH_ENDS to add a second sweep across the full gauge
// range after the initial homing. This verifies that the motor can reach
// both mechanical stops and synchronises the counter to the reference
// stop as chosen by HOME_TO_UPPER above.
// Startup delay increases from ~3 s (one stop) to ~9 s (both ends).
// #define HOME_BOTH_ENDS 1


// Sweep the motor against a mechanical stop and reset the position
// counter so that step 0 = the reference stop (upper or lower per
// HOME_TO_UPPER). Call once in setup() before accepting I2C commands.
static void homeMotor() {
#if HOME_TO_UPPER
    // Homing target at the upper end of the step range.
    const unsigned int refTarget = STEPS;
    const unsigned int oppTarget = 0;
#else
    // Homing target at the lower end.
    const unsigned int refTarget = 0;
    const unsigned int oppTarget = STEPS;
#endif

    // Pass 1: drive toward the reference stop.
    // When the motor hits the physical stop the library continues
    // counting steps internally. This is expected — we reset the
    // counter at the stop after the movement settles.
    motor1.setPosition(refTarget);
    {
        unsigned long start = millis();
        while (millis() - start < HOME_SWEEP_MS) {
            motor1.update();
        }
    }

#ifdef HOME_BOTH_ENDS
    // Pass 2: drive to the opposite stop (verifies full range).
    motor1.setPosition(oppTarget);
    {
        unsigned long start = millis();
        while (millis() - start < HOME_SWEEP_MS) {
            motor1.update();
        }
    }
    // Pass 3: drive back to the reference stop.
    motor1.setPosition(refTarget);
    {
        unsigned long start = millis();
        while (millis() - start < HOME_SWEEP_MS) {
            motor1.update();
        }
    }
#endif

    // Reset the internal counter. Step 0 now = the reference stop.
    motor1.zero();
}


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

// Deferred homing flag — homeMotor() is blocking, so we run it in loop()
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
        // HOME_SWEEP: defer to loop() — homeMotor() is blocking
        homeRequested = true;
    }
}

void setup() {
    init_crc8_table();
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
    homeMotor();  // sweep to stop, then zero the counter
}

void loop() {
    if (homeRequested) {
        homeMotor();
        homeRequested = false;
    }
    motor1.update();
}