/*
 * I2C Interior Panel Slave — CH32V003 remote input panel
 *
 * Reads the switches and pots on a remote interior-lighting sub-panel and
 * reports them to the RP2350 master over I2C. This device drives no outputs:
 * every pin below is an input, so the data direction is slave -> master.
 *
 * Wiring (CH32V003, 3.3V, direct to the RP2350 — no level shifting):
 *   PC1 = SDA -> RP2350 GP6
 *   PC2 = SCL -> RP2350 GP7
 *   PC3 = formation-light MODE toggle, throw A   (active LOW to GND)
 *   PC4 = formation-light MODE toggle, throw B   (active LOW to GND)
 *   PA1 = indexer brightness pot (ADC)
 *   PA2 = standby SAI brightness pot (ADC)
 *   PD7 = reset button (active LOW to GND) -> local software reset
 *   PD1 = SWDIO, reserved for programming
 *   GND -> RP2350 GND
 *
 * 4.7kOhm pull-ups to 3.3V on the RP2350 side only; the I2C pins are
 * open-drain. I2C1 is hardwired to PC1/PC2 on this part and the core exposes
 * no alternate mapping, so Wire.begin() takes an address and nothing else.
 * Note the CH32V003 has I2C1 only — there is no I2C2. PD0/PD1 are TIM1_CH1N
 * and TIM1_CH3N, and PD1 is the SWDIO debug pin, so I2C cannot be moved there.
 *
 * Protocol: master writes [reg=0x01][cmd=0x05][len=0x00][crc8] with a repeated
 * START, then reads exactly 9 bytes:
 *
 *   [reg=0x81][cmd=0x05][len=0x05][state][idx_L][idx_H][sai_L][sai_H][crc8]
 *
 *   state bit0 = PC3 pin level (1 = HIGH/released, 0 = LOW/thrown),
 *   bit1 = PC4 pin level, bits 2-7 reserved (zero).
 *   This mirrors the RP2350's gpio_get() convention so the master's active-LOW
 *   I2cInputSwitch3Pos2Bit logic holds without inversion.
 *   idx/sai    = raw 10-bit ADC, little-endian
 *
 * All debouncing and ADC filtering happens on the master (DcsBios::
 * I2cInputSwitch* / I2cInputPotentiometer); this side only reduces bus
 * traffic. If a frame is rebuilt while the master is reading it the CRC will
 * fail and the master simply retries, so no locking is needed.
 *
 * --- Prerequisites -------------------------------------------------------
 *
 * PD7 is NRST, not GPIO, out of reset. The Arduino core assigns D17 no
 * reset function, but the chip's option bytes still default PD7 to the NRST
 * ball, so the reset button reads nothing until it is overridden once:
 *
 *   minichlink -D     disable NRST on PD7 (USER byte e817 -> 00ff)
 *   minichlink -d     re-enable NRST on PD7 if you want the pin back
 *
 * This is a persistent, per-chip option-byte change, not a runtime setting.
 * Full flashing procedure (minichlink commands, driver setup) is documented in
 * arduino_sketches/README.md under "CH32V003 via minichlink". Do NOT use
 * arduino-cli upload for this part: OpenOCD needs the WCH driver while
 * minichlink needs WinUSB on the probe's Interface 0 — the two are mutually
 * exclusive, and minichlink is the working path here.
 *
 * Flashing needs SWD on PD1 (SWDIO) + GND. PD0/SWCLK is not broken out on
 * SOP16 — the CH32V003 debug interface is the 1-wire SDI on PD1 alone.
 *
 * Build: CH32_Arduino:ch32v:CH32V00x_EVT (pnum=CH32V003F4). There is no A4M6
 * variant — the FQBN always resolves to the CH32V003F4 pinout, which is the
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
// PC3/PD7 expand to plain D-numbers, because digitalRead() and pinMode() run
// them through digitalPinToPinName(). Do not swap the spellings between the
// defines below — they are correct as written.
#define PIN_MODE_A      PC3   // D5,  digitalRead
#define PIN_MODE_B      PC4   // D6,  digitalRead
#define PIN_ADC_INDEXER PA1   // A1,  analogRead (ADC1_IN1)
#define PIN_ADC_SAI     PA2   // A0,  analogRead (ADC1_IN0)
#define PIN_RESET_BTN   PD7   // D17, digitalRead — see the NRST note above

// I2C1 is fixed at PC1/PC2 by the variant's PIN_WIRE_SDA/PIN_WIRE_SCL, so
// there are no pin arguments to pass and none are declared here.

// --- Protocol additions (mirror src/internal/I2cInputPanel.h) -------------
#define REG_PANEL_INPUTS  0x01
#define REG_FLAG_RESPONSE 0x80
#define PAYLOAD_LEN       5
#define RESP_FRAME_LEN    (PAYLOAD_LEN + FRAME_OVERHEAD)  // 9

// Payload byte offsets, relative to the start of the data field. Expressing
// these against FRAME_IDX_DATA rather than as bare 0/1/3 keeps them clear of
// the reg/cmd/len header and the trailing CRC.
#define IDX_STATE        (FRAME_IDX_DATA + 0)
#define IDX_ADC_INDEXER  (FRAME_IDX_DATA + 1)  // 2 bytes, little-endian
#define IDX_ADC_SAI      (FRAME_IDX_DATA + 3)  // 2 bytes, little-endian

// --- Sampling --------------------------------------------------------------
#define SAMPLE_INTERVAL_MS 10
#define DEBOUNCE_MS        50
#define RESET_DEBOUNCE_MS  50
#define ADC_OVERSAMPLES    4
#define ADC_SETTLE_US      100  // sample-and-hold cap settling between reads
#define STATE_MASK         0x03 // bits 2-7 are reserved and go out as zero

// Master request state, written by onReceive().
static uint8_t reqFrame[FRAME_OVERHEAD];
static uint8_t reqReg = 0xFF;
static uint8_t reqCmd = 0xFF;
static bool reqValid = false;

// Response frame, rebuilt by loop() and streamed by onRequest().
static uint8_t respFrame[RESP_FRAME_LEN];

// Debounced toggle state. pairStable is what goes on the wire. Pin-level
// convention: bit SET = pin HIGH (released). Starts at centre (both HIGH,
// 0x03) so the first DEBOUNCE_MS before a real sample lands reports centre
// instead of a spurious thrown position.
static uint8_t pairCandidate = 0x03;
static uint8_t pairStable = 0x03;
static unsigned long pairChangedMs = 0;

static uint8_t stateByte = 0x03;
static uint16_t adcIndexer = 0;
static uint16_t adcSai = 0;

static bool resetLevel = false;
static bool resetFired = false;
static unsigned long resetChangedMs = 0;

static unsigned long lastSampleMs = 0;

/*
 * Read the two MODE toggle pins as pin levels: bit0 = PC3 HIGH, bit1 = PC4
 * HIGH. LOW (thrown to GND) clears the bit. Centre = both HIGH = 0x03.
 * Neither low is the centre position; both low is the in-between transit state.
 */
static uint8_t readModePair() {
    uint8_t pair = 0;
    if (digitalRead(PIN_MODE_A) == HIGH) pair |= 0x01;
    if (digitalRead(PIN_MODE_B) == HIGH) pair |= 0x02;
    return pair;
}

/*
 * Debounce the toggle. The master debounces again, but doing it here keeps
 * repeated transits off the I2C bus.
 */
static void sampleToggle(unsigned long now) {
    const uint8_t raw = readModePair();
    if (raw != pairCandidate) {
        pairCandidate = raw;
        pairChangedMs = now;
    }
    if ((now - pairChangedMs) >= DEBOUNCE_MS) {
        pairStable = pairCandidate;
    }
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
 * Edge-triggered local reset. Waits for a release before firing again so a held
 * button cannot put the part into a reset loop.
 */
static void serviceResetButton(unsigned long now) {
    const bool pressed = (digitalRead(PIN_RESET_BTN) == LOW);

    if (pressed != resetLevel) {
        resetLevel = pressed;
        resetChangedMs = now;
    }
    if (pressed && !resetFired && ((now - resetChangedMs) >= RESET_DEBOUNCE_MS)) {
        resetFired = true;
        NVIC_SystemReset();
        // Fallback if the WCH core helper is unavailable:
        //   NVIC->CFGR = (1<<7) | (0xA05F << 16);
        for (;;) {
        }
    }
    if (!pressed) {
        resetFired = false;
    }
}

/*
 * Pack the sampled inputs into the response frame. I2C_REG_RESPONSE (0x80) is
 * OR'd into the register so the master can tell slave frames from its own.
 */
static void buildResponse() {
    respFrame[FRAME_IDX_REG] = REG_PANEL_INPUTS | REG_FLAG_RESPONSE;
    respFrame[FRAME_IDX_CMD] = CMD_STATUS_REQ;
    respFrame[FRAME_IDX_LEN] = PAYLOAD_LEN;

    respFrame[IDX_STATE] = stateByte;

    respFrame[IDX_ADC_INDEXER]     = (uint8_t)(adcIndexer & 0xFF);
    respFrame[IDX_ADC_INDEXER + 1] = (uint8_t)((adcIndexer >> 8) & 0xFF);
    respFrame[IDX_ADC_SAI]         = (uint8_t)(adcSai & 0xFF);
    respFrame[IDX_ADC_SAI + 1]     = (uint8_t)((adcSai >> 8) & 0xFF);

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
 * master that reads before the first sample — or whose request frame was not
 * latched (repeated-START delivery) — gets a well-formed, CRC-valid answer
 * instead of hanging the link offline.
 */
static void onRequest() {
    Wire.write(respFrame, RESP_FRAME_LEN);
}

void setup() {
    pinMode(PIN_MODE_A, INPUT_PULLUP);
    pinMode(PIN_MODE_B, INPUT_PULLUP);
    pinMode(PIN_RESET_BTN, INPUT_PULLUP);

    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    // Prime the frame so a master that reads before the first sample still gets
    // a well-formed response.
    buildResponse();
}

void loop() {
    const unsigned long now = millis();

    serviceResetButton(now);

    if ((now - lastSampleMs) >= SAMPLE_INTERVAL_MS) {
        lastSampleMs = now;

        sampleToggle(now);

        adcIndexer = readAdcAverage(PIN_ADC_INDEXER);
        delayMicroseconds(ADC_SETTLE_US);
        adcSai = readAdcAverage(PIN_ADC_SAI);

        stateByte = pairStable & STATE_MASK;
        buildResponse();
    }
}