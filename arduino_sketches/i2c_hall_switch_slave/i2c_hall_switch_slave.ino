/*
 * I2C Hall Switch Slave — 8-Position Thumbwheel Reader
 *
 * CH32V003 I2C slave that reads 8 A3144 hall switches and reports
 * the active switch number (1-8) or 0 if none.
 *
 * Protocol: raw byte — master reads 1 byte, value is 0-8.
 * No frame protocol, no CRC. Simple and minimal.
 *
 * Wiring (CH32V003 SOP16 / Arduino D-numbers):
 *   D3  (PC1) = SDA  → RP2350 GP6
 *   D4  (PC2) = SCL  → RP2350 GP7
 *   D5  (PC3) = HALLSW1
 *   D6  (PC4) = HALLSW2
 *   D8  (PC6) = HALLSW3
 *   D9  (PC7) = HALLSW4
 *   D14 (PD4) = HALLSW5
 *   D15 (PD5) = HALLSW6
 *   D16 (PD6) = HALLSW7
 *   D17 (PD7) = HALLSW8
 *   D0  (PA1) = LIMIT_SW (defined, unused)
 *   GND       → RP2350 GND
 *   VCC       → 3.3V
 *
 * A3144: open-collector, active LOW. Internal pull-ups enabled.
 * I2C bus: 4.7kΩ pull-ups to 3.3V on RP2350 side.
 */

#include <Wire.h>
#include <Arduino.h>

#define I2C_SLAVE_ADDRESS 0x08

// Hall switch pins (Arduino D-numbers, matching CH32V003 SOP16 package)
#define HALLSW1  5   // PC3
#define HALLSW2  6   // PC4
#define HALLSW3  8   // PC6
#define HALLSW4  9   // PC7
#define HALLSW5  14  // PD4
#define HALLSW6  15  // PD5
#define HALLSW7  16  // PD6
#define HALLSW8  17  // PD7

#define LIMIT_SW 0   // PA1 (unused for now)

static const uint8_t hallPins[8] = {
    HALLSW1, HALLSW2, HALLSW3, HALLSW4,
    HALLSW5, HALLSW6, HALLSW7, HALLSW8
};

/*
 * Read hall switches and return active switch number.
 * Returns 0 if none active, 1-8 for active switch.
 * A3144 is active LOW (magnet present = LOW).
 * No two switches should be active simultaneously (mechanical constraint).
 */
static uint8_t readHallSwitches() {
    for (uint8_t i = 0; i < 8; i++) {
        if (digitalRead(hallPins[i]) == LOW) {
            return i + 1;
        }
    }
    return 0;
}

/*
 * I2C request handler — master reads 1 byte.
 * Called by the Wire library when the master requests data.
 */
static void onRequest() {
    uint8_t switchNum = readHallSwitches();
    Wire.write(switchNum);
}

/*
 * I2C receive handler — master writes data.
 * Not used for this simple slave (master only reads).
 * Required by Wire library to avoid NAK on write phase.
 */
static void onReceive(int howMany) {
    // Discard any data sent by master
    while (Wire.available()) {
        Wire.read();
    }
}

void setup() {
    // Configure all hall switch pins as inputs with pull-ups
    for (uint8_t i = 0; i < 8; i++) {
        pinMode(hallPins[i], INPUT_PULLUP);
    }

    // Initialize I2C slave
    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onRequest(onRequest);
    Wire.onReceive(onReceive);
}

void loop() {
    // Nothing to do — I2C is interrupt-driven
    delay(10);
}
