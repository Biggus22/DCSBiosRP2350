/*
 * I2C Weapons Panel Slave — CH32V003 input panel for the F-4E weapons panel
 *
 * Reports the gun arm switch, the 8-position weapon select and the bomb
 * quantity selector to the RP2350 master over I2C. This device drives no
 * outputs: every pin below is an input, so the data direction is slave ->
 * master.
 *
 * Wiring (CH32V003A4M6, 3.3V, direct to the RP2350 — no level shifting):
 *   PC1 = SDA -> RP2350 GP6
 *   PC2 = SCL -> RP2350 GP7
 *   PC0 = gun arm switch (active LOW to GND) - 0 = pressed = armed
 *   PA2 = bomb quantity selector (analog, 12 positions)
 *   weapon select, one-hot, switch common to GND:
 *     position 0 = PD6, 1 = PC3, 2 = PC4, 3 = PC6,
 *     position 4 = PC7, 5 = PD7, 6 = PD4, 7 = PD5
 *   PD1 = SWDIO, reserved for programming
 *   GND -> RP2350 GND
 *
 * 4.7kOhm pull-ups to 3.3V on the RP2350 side only; the I2C pins are
 * open-drain. I2C1 is hardwired to PC1/PC2 on this part and the core exposes
 * no alternate mapping, so Wire.begin() takes an address and nothing else.
 *
 * PD5 and PD6 are the USART1 pins of the debug UART. They are used here as
 * weapon select positions, so this sketch must never use Serial.
 *
 * Protocol: master writes [reg=0x01][cmd=0x05][len=0x00][crc8] with a repeated
 * START, then reads exactly 11 bytes:
 *
 *   [reg=0x81][cmd=0x05][len=0x07][state0][bomb_L][bomb_H][spare_L][spare_H][state2][value0][crc8]
 *
 *   state0 bit0 = PC0 pin level (1 = HIGH/released, 0 = LOW/pressed),
 *            bits 1-7 reserved (zero).
 *   state2      bit i = weapon select position i pin level (1 = HIGH, 0 = LOW).
 *                The selected position is the one whose pin is LOW; the master
 *                decodes the one-hot pattern and reports the position.
 *   bomb_L/H    raw 10-bit ADC of PA2, little-endian.
 *   spare_L/H   unused (zero).
 *   value0      bomb quantity position 0..11, thresholded from the ADC here.
 *
 * The wire convention is pin levels - bit SET = pin HIGH (released) - which
 * mirrors the RP2350's gpio_get() so the master's active-LOW logic holds
 * without inversion.
 *
 * All debouncing happens on the master (DcsBios::I2cInputSwitch* /
 * I2cInputValue); this side only reduces bus traffic. If a frame is rebuilt
 * while the master is reading it the CRC will fail and the master simply
 * retries, so no locking is needed.
 *
 * --- Prerequisites -------------------------------------------------------
 *
 * PD7 is NRST, not GPIO, out of reset. The Arduino core assigns D17 no
 * reset function, but the chip's option bytes still default PD7 to the NRST
 * ball, so weapon select position 5 reads nothing until it is overridden once:
 *
 *   minichlink -D     disable NRST on PD7 (USER byte e817 -> 00ff)
 *   minichlink -d     re-enable NRST on PD7 if you want the pin back
 *
 * This is a persistent, per-chip option-byte change, not a runtime setting.
 * Full flashing procedure (minichlink commands, driver setup) is documented in
 * arduino_sketches/README.md under "CH32V003 via minichlink". Do NOT use
 * arduino-cli upload for this part: OpenOCD needs the WCH driver while
 * minichlink needs WinUSB on the probe's Interface 0 - the two are mutually
 * exclusive, and minichlink is the working path here.
 *
 * Flashing needs SWD on PD1 (SWDIO) + GND. PD0/SWCLK is not broken out on
 * SOP16 - the CH32V003 debug interface is the 1-wire SDI on PD1 alone.
 *
 * Build: CH32_Arduino:ch32v:CH32V00x_EVT (pnum=CH32V003F4). There is no A4M6
 * variant - the FQBN always resolves to the CH32V003F4 pinout, which is the
 * same die. Do NOT build against the WCH:ch32v package: its Wire library has
 * the slave path compiled out and cannot do this at all.
 */

#include <Wire.h>
#include <Arduino.h>
#include "i2c_slave_defs.h"

#ifndef I2C_SLAVE_ADDRESS
#define I2C_SLAVE_ADDRESS 0x08
#endif

// --- Pin map ---------------------------------------------------------------
// Arduino D-numbers for the CH32V003 SOP16 (CH32V003A4M6, 16 KB flash, 2 KB
// SRAM, 14 I/O), for wiring: PC1=1 PC2=2 PC3=3 PC4=4 PC6=5 PC7=6 PD1=7 (SWDIO)
// PD4=8 PD5=9 (TX1) PD6=10 (RX1) PD7=11 PA1=12 PA2=13 PC0=16
//
// IMPORTANT: the core overloads these port names, and the two families need
// different kinds of number. PA1/PA2 expand to PIN_A1/PIN_A0, which are
// A-numbers, because analogRead() runs them through analogInputToPinName().
// PCx/PDx expand to plain D-numbers, because digitalRead() and pinMode() run
// them through digitalPinToPinName(). Do not swap the spellings between the
// defines below - they are correct as written.
#define PIN_GUN_ARM     PC0   // D2,  digitalRead
#define PIN_WPN_SEL_0   PD6   // D16, digitalRead - weapon select position 0
#define PIN_WPN_SEL_1   PC3   // D5,  digitalRead - weapon select position 1
#define PIN_WPN_SEL_2   PC4   // D6,  digitalRead - weapon select position 2
#define PIN_WPN_SEL_3   PC6   // D8,  digitalRead - weapon select position 3
#define PIN_WPN_SEL_4   PC7   // D9,  digitalRead - weapon select position 4
#define PIN_WPN_SEL_5   PD7   // D17, digitalRead - weapon select position 5
#define PIN_WPN_SEL_6   PD4   // D14, digitalRead - weapon select position 6
#define PIN_WPN_SEL_7   PD5   // D15, digitalRead - weapon select position 7
#define PIN_ADC_BOMB    PA2   // A0,  analogRead (ADC1_IN0)

// I2C1 is fixed at PC1/PC2 by the variant's PIN_WIRE_SDA/PIN_WIRE_SCL, so
// there are no pin arguments to pass and none are declared here.

// --- Protocol additions (mirror src/internal/I2cInputPanel.h) -------------
#define REG_PANEL_INPUTS  0x01
#define REG_FLAG_RESPONSE 0x80
#define PAYLOAD_LEN       7
#define RESP_FRAME_LEN    (PAYLOAD_LEN + FRAME_OVERHEAD)  // 11

// Payload byte offsets, relative to the start of the data field. Expressing
// these against FRAME_IDX_DATA rather than as bare 0/1/3 keeps them clear of
// the reg/cmd/len header and the trailing CRC.
#define IDX_STATE0     (FRAME_IDX_DATA + 0)
#define IDX_ADC_BOMB   (FRAME_IDX_DATA + 1)  // 2 bytes, little-endian
#define IDX_ADC_SPARE  (FRAME_IDX_DATA + 3)  // 2 bytes, unused
#define IDX_STATE2     (FRAME_IDX_DATA + 5)
#define IDX_VALUE0     (FRAME_IDX_DATA + 6)

// --- Sampling --------------------------------------------------------------
#define SAMPLE_INTERVAL_MS 10
#define ADC_OVERSAMPLES    4
#define ADC_SETTLE_US      100  // sample-and-hold cap settling between reads

// --- Bomb quantity ---------------------------------------------------------
//
// The selector produces one of 12 analog levels. The thresholds below are the
// default uniform split of the 10-bit ADC range, which is only correct for a
// ladder with equal steps. Read the raw ADC values (the master prints them)
// at every detent and replace this table with the measured values.
#define BOMB_QTY_LEVELS     12
#define BOMB_QTY_HYSTERESIS 8  // ADC counts the raw value must clear to change
static const uint16_t BOMB_QTY_THRESHOLD[BOMB_QTY_LEVELS - 1] = {
    85, 170, 256, 341, 426, 512, 597, 682, 768, 853, 938
};

// Master request state, written by onReceive().
static uint8_t reqFrame[FRAME_OVERHEAD];
static uint8_t reqReg = 0xFF;
static uint8_t reqCmd = 0xFF;
static bool reqValid = false;

// Response frame, rebuilt by loop() and streamed by onRequest().
static uint8_t respFrame[RESP_FRAME_LEN];

static uint8_t state0 = 0x01;  // gun arm released
static uint8_t state2 = 0xFF;  // no weapon select pin low
static uint16_t adcBomb = 0;
static uint8_t bombQty = 0;

static unsigned long lastSampleMs = 0;

/*
 * Read the gun arm pin as a level: bit SET = pin HIGH (released). LOW
 * (pressed to GND) clears the bit, and the master reports 0 = pressed =
 * armed for that control.
 */
static uint8_t readGunArm() {
    uint8_t byte = 0;
    if (digitalRead(PIN_GUN_ARM) == LOW) byte |= 0x01;  // inverted: LOW = armed
    return byte;
}

/*
 * Read the eight weapon select pins as levels: bit i = position i pin HIGH.
 * The selected position is the one whose pin is LOW, so a one-hot pattern
 * with exactly one clear bit is the selected position. The master does the
 * decoding and holds the last position while the pattern is not valid.
 */
static uint8_t readWeaponSelect() {
    uint8_t byte = 0;
    if (digitalRead(PIN_WPN_SEL_0) == HIGH) byte |= 0x01;
    if (digitalRead(PIN_WPN_SEL_1) == HIGH) byte |= 0x02;
    if (digitalRead(PIN_WPN_SEL_2) == HIGH) byte |= 0x04;
    if (digitalRead(PIN_WPN_SEL_3) == HIGH) byte |= 0x08;
    if (digitalRead(PIN_WPN_SEL_4) == HIGH) byte |= 0x10;
    if (digitalRead(PIN_WPN_SEL_5) == HIGH) byte |= 0x20;
    if (digitalRead(PIN_WPN_SEL_6) == HIGH) byte |= 0x40;
    if (digitalRead(PIN_WPN_SEL_7) == HIGH) byte |= 0x80;
    return byte;
}

/*
 * Oversample one ADC channel. The CH32V003 sample-and-hold cap needs a moment
 * between conversions, especially right after a channel switch.
 */
static uint16_t readAdcAverage(uint8_t pin) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < ADC_OVERSAMPLES; i++) {
        sum += (uint16_t)analogRead(pin);
        delayMicroseconds(ADC_SETTLE_US);
    }
    return (uint16_t)(sum / ADC_OVERSAMPLES);
}

/*
 * Bucket the raw ADC value into one of the twelve selector positions.
 */
static uint8_t bombQtyBucket(uint16_t raw) {
    uint8_t index = 0;
    while (index < (BOMB_QTY_LEVELS - 1) && raw >= BOMB_QTY_THRESHOLD[index]) {
        index++;
    }
    return index;
}

/*
 * Update the reported bomb quantity. The raw value has to clear the
 * threshold between two positions by BOMB_QTY_HYSTERESIS counts before the
 * reported position changes, so a value sitting on a threshold does not flap.
 */
static void updateBombQuantity(uint16_t raw) {
    const uint8_t bucket = bombQtyBucket(raw);
    if (bucket == bombQty) return;

    const uint16_t boundary = (bucket > bombQty)
        ? BOMB_QTY_THRESHOLD[bombQty]
        : BOMB_QTY_THRESHOLD[bucket];
    const uint16_t distance = (raw > boundary)
        ? (uint16_t)(raw - boundary)
        : (uint16_t)(boundary - raw);
    if (distance < BOMB_QTY_HYSTERESIS) return;

    bombQty = bucket;
}

/*
 * Pack the sampled inputs into the response frame. I2C_REG_RESPONSE (0x80) is
 * OR'd into the register so the master can tell slave frames from its own.
 */
static void buildResponse() {
    respFrame[FRAME_IDX_REG] = REG_PANEL_INPUTS | REG_FLAG_RESPONSE;
    respFrame[FRAME_IDX_CMD] = CMD_STATUS_REQ;
    respFrame[FRAME_IDX_LEN] = PAYLOAD_LEN;

    respFrame[IDX_STATE0] = state0;

    respFrame[IDX_ADC_BOMB]     = (uint8_t)(adcBomb & 0xFF);
    respFrame[IDX_ADC_BOMB + 1] = (uint8_t)((adcBomb >> 8) & 0xFF);
    respFrame[IDX_ADC_SPARE]     = 0;
    respFrame[IDX_ADC_SPARE + 1] = 0;

    respFrame[IDX_STATE2] = state2;
    respFrame[IDX_VALUE0] = bombQty;

    respFrame[RESP_FRAME_LEN - 1] = crc8_calc(respFrame, RESP_FRAME_LEN - 1);
}

/*
 * Master request handler. The frame is validated and remembered, but the
 * response does NOT depend on it (see onRequest): some slave stacks do not
 * deliver onReceive before onRequest on a repeated-START combined transaction,
 * and gating the response on reqValid then deadlocks the link (master retries
 * forever, panel stays offline).
 */
static void onReceive(int howMany) {
    (void)howMany;

    uint8_t n = 0;
    while (Wire.available() && n < FRAME_OVERHEAD) {
        reqFrame[n++] = (uint8_t)Wire.read();
    }
    if (n < FRAME_OVERHEAD) {
        reqValid = false;
        return;
    }
    if (crc8_calc(reqFrame, (uint8_t)(n - 1)) != reqFrame[n - 1]) {
        reqValid = false;
        return;
    }

    reqReg = reqFrame[FRAME_IDX_REG];
    reqCmd = reqFrame[FRAME_IDX_CMD];
    reqValid = true;
}

/*
 * Master read handler. Always streams the latest response frame: the master
 * requests exactly RESP_FRAME_LEN bytes, so a short write would NAK and stall
 * the bus. The frame is primed in setup() and rebuilt every sample, so even a
 * master that reads before the first sample - or whose request frame was not
 * latched (repeated-START delivery) - gets a well-formed, CRC-valid answer
 * instead of hanging the link offline.
 */
static void onRequest() {
    Wire.write(respFrame, RESP_FRAME_LEN);
}

void setup() {
    pinMode(PIN_GUN_ARM, INPUT_PULLUP);

    pinMode(PIN_WPN_SEL_0, INPUT_PULLUP);
    pinMode(PIN_WPN_SEL_1, INPUT_PULLUP);
    pinMode(PIN_WPN_SEL_2, INPUT_PULLUP);
    pinMode(PIN_WPN_SEL_3, INPUT_PULLUP);
    pinMode(PIN_WPN_SEL_4, INPUT_PULLUP);
    pinMode(PIN_WPN_SEL_5, INPUT_PULLUP);
    pinMode(PIN_WPN_SEL_6, INPUT_PULLUP);
    pinMode(PIN_WPN_SEL_7, INPUT_PULLUP);

    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    // Prime the frame so a master that reads before the first sample still gets
    // a well-formed response.
    buildResponse();
}

void loop() {
    const unsigned long now = millis();

    if ((now - lastSampleMs) >= SAMPLE_INTERVAL_MS) {
        lastSampleMs = now;

        state0 = readGunArm();
        state2 = readWeaponSelect();

        adcBomb = readAdcAverage(PIN_ADC_BOMB);
        updateBombQuantity(adcBomb);

        buildResponse();
    }
}
