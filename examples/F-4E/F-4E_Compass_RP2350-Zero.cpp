// F-4E Compass Control Panel — RP2350-Zero + ATtiny1614 I2C gauge slave
//
// The RP2350 carries the compass mode rotary, the gyro mode (BCD-coded)
// rotary, the SET HDG knob and the LATITUDE knob, plus a 10-pixel SK6812
// console-lighting strip. The ATtiny1614 on the same PCB drives the compass
// sync X27 gauge and receives it over i2c1 (see
// arduino_sketches/i2c_compass_sync_gauge_slave/).
//
// The ATtiny also has PB0 wired for analog input; that input is unused for
// now, so it is commented out in the slave sketch.
//
// Board mode is a compile-time constant: 0xF is USB-only (bench), 0x1-0xD is
// RS485 slave. GP36-39 are the RP2350-Zero's QSPI flash pins, so the
// framework's default ADDR0-3 strap scheme is unavailable on this board.

#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdio.h>
#include <cstdio>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/ws2812.h"
#include "internal/I2cBusHwMaster.h"
#include "internal/I2cOutputListener.h"
#include "hardware/adc.h"

#define NUM_LEDS 10
#define LED_PIN 3

// 0xF = USB-only. 0x0 = RS485 host, 0x1-0xD = RS485 slave, 0xE = RS485 terminal.
static constexpr uint8_t kBoardAddress = 0xF;

// Compass sync gauge slave (ATtiny1614, 7-bit I2C address).
static constexpr uint8_t kGaugeSlaveAddr = 0x08;
static constexpr uint8_t kGaugeRegId = 0x01;  // REG_GAUGE

// Gauge mapping. The ATtiny is only a "setPosition(steps)" executor, so all of
// this is tuned here, over USB - no UPDI session needed:
//   kGaugeSpan      full-scale step count; must be <= the slave's STEPS (945).
//                   This gauge's limiter gives 90 deg of travel; at the X27's
//                   3 steps/deg that is 270 steps.
//   kGaugeReversed  flips the needle direction (use when DCS 0 lands at the
//                   wrong end of the dial)
//   kGaugeDeadband  steps of DCS jitter ignored before a new position is sent
static constexpr uint16_t kGaugeSpan = 270;
static constexpr bool kGaugeReversed = true;
static constexpr uint16_t kGaugeDeadband = 3;

// Bench bring-up mode. When 1 the DCS listener, the parser and core 1 are all
// left out and the firmware becomes an interactive jog tool on the USB serial
// console: step the needle from the mechanical zero and read the exact step
// count back. Use it to find the offset between the homed stop and the gauge's
// real zero, then apply that offset to the flight mapping and set this to 0.
// DCS must be disconnected in this mode - the USB link carries console text,
// not DCS-BIOS messages.
//
// This must be a macro (not constexpr): the branches below use #if, which the
// preprocessor evaluates before the compiler sees any C++ constant.
#define GAUGE_BENCH_MODE 0

WS2812 externalLeds(pio0, 0, LED_PIN, false);

uart_inst_t *rs485_uart = uart0;

// ---------------------------------------------------------------------------
// I2C master -> ATtiny1614 gauge slave
// ---------------------------------------------------------------------------

static DcsBios::I2cBusHwMaster i2cBus;  // i2c1, GP6 SDA / GP7 SCL, 100 kHz

#if !GAUGE_BENCH_MODE
// Compass sync indicator needle: DCS 0..65535 is scaled to 0..kGaugeSpan steps
// (optionally reversed) on the master and sent as a raw step target.
static DcsBios::I2cOutputListener compassSyncGauge(
    F_4E_PLT_COMPASS_SYNC_A,
    &i2cBus,
    kGaugeSlaveAddr,
    kGaugeRegId,
    DcsBios::I2cOutputListener::STEPS_SCALED,
    kGaugeSpan,
    kGaugeReversed,
    kGaugeDeadband
);
#else
// Drive one absolute step target to the gauge slave.
static void sendGaugeSteps(uint16_t steps) {
    const uint8_t data[2] = { (uint8_t)(steps & 0xFF), (uint8_t)(steps >> 8) };
    i2cBus.sendFrame(kGaugeSlaveAddr, kGaugeRegId, DcsBios::I2C_CMD_SET_POSITION,
                     data, 2, 10);
}

// Interactive jog tool. The ATtiny has already homed onto its mechanical stop,
// so step 0 here is that stop. Move the needle until it points at the gauge's
// real zero, then read the printed step count.
static void gaugeJogLoop() {
    uint16_t pos = 0;
    sendGaugeSteps(pos);

    printf("\nCompass gauge jog (step 0 = mechanical stop)\n");
    printf("  +  -       one step\n");
    printf("  ]  [       ten steps\n");
    printf("  }  {       fifty steps\n");
    printf("  h          go to 0 (mechanical stop)\n");
    printf("  e          go to full travel (%u)\n", (unsigned)kGaugeSpan);
    printf("  s          sweep 0 -> full -> 0\n");
    printf("  ?          report position\n");
    printf("steps=0\n");

    while (true) {
        const int c = getchar_timeout_us(20000);
        if (c == PICO_ERROR_TIMEOUT) {
            DcsBios::updateHeartbeat();
            continue;
        }

        bool moved = false;
        switch (c) {
            case '+': if (pos < kGaugeSpan) { pos++; moved = true; } break;
            case '-': if (pos > 0) { pos--; moved = true; } break;
            case ']': pos = (pos + 10 > kGaugeSpan) ? kGaugeSpan : (uint16_t)(pos + 10); moved = true; break;
            case '[': pos = (pos > 10) ? (uint16_t)(pos - 10) : 0; moved = true; break;
            case '}': pos = (pos + 50 > kGaugeSpan) ? kGaugeSpan : (uint16_t)(pos + 50); moved = true; break;
            case '{': pos = (pos > 50) ? (uint16_t)(pos - 50) : 0; moved = true; break;
            case 'h': pos = 0; moved = true; break;
            case 'e': pos = kGaugeSpan; moved = true; break;
            case 's':
                for (uint16_t p = 0; p <= kGaugeSpan; p += (kGaugeSpan / 90) + 1) {
                    sendGaugeSteps(p);
                    sleep_ms(20);
                }
                for (uint16_t p = kGaugeSpan; ; ) {
                    sendGaugeSteps(p);
                    sleep_ms(20);
                    if (p == 0) break;
                    p = (p > (kGaugeSpan / 90) + 1) ? (uint16_t)(p - (kGaugeSpan / 90) - 1) : 0;
                }
                pos = 0;
                moved = true;
                break;
            case '?': break;
            case '\r': case '\n': continue;
            default: continue;
        }

        if (moved) sendGaugeSteps(pos);
        printf("steps=%u\n", (unsigned)pos);
    }
}
#endif

// ---------------------------------------------------------------------------
// Panel inputs — RP2350 GPIOs (active LOW, internal pull-ups)
// ---------------------------------------------------------------------------

// Compass mode rotary. The pin array order is the value the control receives:
// kModePins[i] pulled LOW reports position i. Confirmed order:
// COMP=0, DG=1, SLAVED=2, SYNC=3. The NEXT detents of the switch are wired to
// GP14/GP15 as hardware spares but deliberately left out of this program.
static const uint8_t kModePins[4] = {13, 12, 11, 10};
DcsBios::SwitchMultiPosT<POLL_EVERY_TIME, 4> pltCompassModeSync(
    "PLT_COMPASS_MODE_SYNC", kModePins, 100);

// Gyro mode: BCD-coded rotary (1/2/4/8 weight lines, common to GND, active
// low), of which only two detents are used. The detent the switch rests in at
// boot is defined as the SAFE position and reported as "0" at startup, so the
// cockpit aligns with the physical switch at spawn. The first different stable
// detent afterwards becomes GYRO ("1"); any other detent is ignored.
//
// Power up with the knob at the SAFE detent - the knob/dial face is aligned to
// whatever detent that is.
//
// The DCS control is PLT_COMPASS_GYRO_MODE, a plain two-state toggle: send
// "0"/"1" on change only.
class CompassGyroToggle : public DcsBios::PollingInput, public DcsBios::ResettableInput {
private:
    const char* msg_;
    uint pinA_, pinB_, pinC_, pinD_;
    int learnedSafeCode_ = -1;   // first stable detent at boot
    int learnedGyroCode_ = -1;   // first different stable detent seen
    int lastReported_ = -1;      // 0 = safe, 1 = gyro, -1 = nothing yet
    int debounceCandidate_ = -1;
    unsigned long debounceDelay_;
    unsigned long lastDebounceTime_ = 0;

    int readRawCode() const {
        int total = 0;
        if (gpio_get(pinA_) == 0) total += 1;
        if (gpio_get(pinB_) == 0) total += 2;
        if (gpio_get(pinC_) == 0) total += 4;
        if (gpio_get(pinD_) == 0) total += 8;
        return total;
    }

    void resetState() {
        // Keep the learned detents; only forget the reported state.
        lastReported_ = -1;
        debounceCandidate_ = -1;
        lastDebounceTime_ = 0;
    }

    void pollInput() {
        const int code = readRawCode();

        const unsigned long now = to_ms_since_boot(get_absolute_time());
        if (code != debounceCandidate_) {
            debounceCandidate_ = code;
            lastDebounceTime_ = now;
        }
        if ((now - lastDebounceTime_) < debounceDelay_) return;

        if (learnedSafeCode_ < 0) {
            learnedSafeCode_ = code;   // boot detent = SAFE
            // Report the position once at startup so the cockpit matches the
            // physical switch at spawn (a toggle is a state, not an action).
            if (DcsBios::tryToSendDcsBiosMessage(msg_, "0")) lastReported_ = 0;
            return;
        }
        if (code == learnedSafeCode_) {
            if (lastReported_ != 0) {
                if (DcsBios::tryToSendDcsBiosMessage(msg_, "0")) lastReported_ = 0;
            }
            return;
        }
        if (code != learnedGyroCode_) {
            if (learnedGyroCode_ < 0) {
                learnedGyroCode_ = code;  // first other stable detent = GYRO
            } else {
                return;  // a third detent - not one of the two used
            }
        }
        if (lastReported_ != 1) {
            if (DcsBios::tryToSendDcsBiosMessage(msg_, "1")) lastReported_ = 1;
        }
    }

public:
    CompassGyroToggle(const char* msg, uint pinA, uint pinB, uint pinC, uint pinD,
                      unsigned long debounceDelay = 100)
        : PollingInput(POLL_EVERY_TIME), msg_(msg),
          pinA_(pinA), pinB_(pinB), pinC_(pinC), pinD_(pinD),
          debounceDelay_(debounceDelay) {
        gpio_init(pinA_); gpio_pull_up(pinA_);
        gpio_init(pinB_); gpio_pull_up(pinB_);
        gpio_init(pinC_); gpio_pull_up(pinC_);
        gpio_init(pinD_); gpio_pull_up(pinD_);
        resetState();
    }

    void resetThisState() { this->resetState(); }

    // Raw 4-bit code of the current detent, for bench diagnostics.
    int getRawCode() const { return readRawCode(); }
};

// Gyro mode: 4 = BCD 1, 5 = BCD 2, 8 = BCD 4, 9 = BCD 8.
CompassGyroToggle pltCompassGyroMode("PLT_COMPASS_GYRO_MODE", 4, 5, 8, 9);

// ---------------------------------------------------------------------------
// Bench diagnostics. Production USB must carry ONLY "TAG VALUE\n" lines, so
// any bare printf breaks DCS parsing. Set to 1 for bench debugging ONLY, with
// DCS disconnected: prints the raw gyro-mode BCD code every 500 ms.
#define PANEL_DEBUG 0

#if PANEL_DEBUG
static unsigned long lastDiagMs = 0;

static void reportBcdLines() {
    unsigned long now = to_ms_since_boot(get_absolute_time());
    if ((now - lastDiagMs) < 500) return;
    lastDiagMs = now;

    printf("bcd: p4=%d p5=%d p8=%d p9=%d code=%d\n",
           gpio_get(4), gpio_get(5), gpio_get(8), gpio_get(9),
           pltCompassGyroMode.getRawCode());
}
#endif

// GP26-29 are the only four ADC pins, and PotentiometerEWMA hard-maps
// 26/27/28/29 to ADC0/1/2/3.
// SET HDG: the DCS control range is -1..1; DCS-BIOS maps 0..65535 across it,
// so mid-scale pot = centre. GP27/GP28 are pinned for analog input but
// unused on this panel - adc_gpio_init calls kept for when they are wired:
// DcsBios::Potentiometer pltCompassUnused27("<CONTROL>", 27, false, 0, 4095);
// DcsBios::Potentiometer pltCompassUnused28("<CONTROL>", 28, false, 0, 4095);
DcsBios::Potentiometer pltCompassSetHeadingRotate("PLT_COMPASS_SET_HEADING_ROTATE", 26, true, 0, 4095);
DcsBios::Potentiometer pltCompassLatitude("PLT_COMPASS_LATITUDE", 29, true, 0, 4095);

// ---------------------------------------------------------------------------
// Output — console lighting strip
// ---------------------------------------------------------------------------

void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)(((uint64_t)consoleBrightness * 255) / 65535);

    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgb(intensity, 0, 0));
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// ---------------------------------------------------------------------------

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);  // Wait for USB CDC to be ready

    // adc_gpio_init(27);
    // adc_gpio_init(28);

    if (!externalLeds.begin(NUM_LEDS)) {
        // The strip stays dark. There is no output to report this on: the USB
        // link carries DCS-BIOS messages only.
    }

    // Power-on green flash.
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgb(0, 201, 0));
    }
    externalLeds.show();
    sleep_ms(5000);  // Long power-on green flash, as on the other panels

    // Clear LEDs after the flash.
    externalLeds.clear();
    externalLeds.show();

#if GAUGE_BENCH_MODE
    // Bench jog tool: no parser, no core 1, no RS485 - just the console and the
    // I2C gauge. Never returns.
    gaugeJogLoop();
#endif

    DcsBios::BoardMode board = DcsBios::determineBoardMode(kBoardAddress);
    DcsBios::currentBoardMode = board;

    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);

    DcsBios::setup();

    while (true) {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
#if PANEL_DEBUG
        reportBcdLines();
#endif
        sleep_us(10);
    }
}
