// F-4E Environmental Control Panel — RP2350-Zero
//
// A second build of the compass panel PCB, used for the environmental
// control panel. It carries the same 10-pixel SK6812 console-lighting strip
// driven from DCS-BIOS, but its two inputs — a temperature control pot and
// a three-position latching toggle — have no DCS-BIOS controls, so they are
// exposed on a composite USB HID gamepad interface and bound in DCS like
// any joystick.
//
// USB interface layout (see src/internal/usb_descriptors.c):
//   0/1  CDC-ACM  - DCS-BIOS protocol
//   2    vendor   - picotool soft reset
//   3    HID gamepad - X axis (pot) + 2 buttons (toggle)
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
#include "internal/UsbHidGamepad.h"
#include "internal/UsbIdentity.h"
#include "hardware/adc.h"

// Host-visible USB identity (Device Manager, joy.cpl) — see UsbIdentity.h.
DCSBIOS_USB_ID("DCSBiosRP2350", "F-4E ECS Panel")
DCSBIOS_USB_HID_NAME("ECS Gamepad")

#define NUM_LEDS 10
#define LED_PIN 3

// 0xF = USB-only. 0x0 = RS485 host, 0x1-0xD = RS485 slave, 0xE = RS485 terminal.
static constexpr uint8_t kBoardAddress = 0xF;

WS2812 externalLeds(pio0, 0, LED_PIN, false);

uart_inst_t *rs485_uart = uart0;

// ---------------------------------------------------------------------------
// USB HID gamepad inputs
// ---------------------------------------------------------------------------
// Deliberately NOT DcsBios::PollingInput subclasses: these controls don't
// exist in DCS-BIOS, and the USB serial line must carry nothing but genuine
// DCS-BIOS messages.

// Three-position latching toggle. Both legs are pulled up; a thrown position
// grounds exactly one of them, centre grounds neither.
// Swap these two constants if the switch reports up/down backwards.
static constexpr uint kToggleUpPin = 11;    // HID button 1
static constexpr uint kToggleDownPin = 12;  // HID button 2

class HidToggle3Pos {
private:
    static const unsigned long kDebounceMs = 20;

    uint pinUp_, pinDown_;
    int candidate_ = -1;          // last raw reading, for debounce
    int stable_ = 0;              // last stable reading: bit0 = up, bit1 = down
    unsigned long lastChangeMs_ = 0;

public:
    HidToggle3Pos(uint pinUp, uint pinDown) : pinUp_(pinUp), pinDown_(pinDown) {
        gpio_init(pinUp_);
        gpio_pull_up(pinUp_);
        gpio_init(pinDown_);
        gpio_pull_up(pinDown_);
    }

    void poll() {
        int code = 0;
        if (gpio_get(pinUp_) == 0) code |= 1;
        if (gpio_get(pinDown_) == 0) code |= 2;

        const unsigned long now = to_ms_since_boot(get_absolute_time());
        if (code != candidate_) {
            candidate_ = code;
            lastChangeMs_ = now;
        }
        if ((now - lastChangeMs_) < kDebounceMs || code == stable_) return;

        stable_ = code;
        UsbHidGamepad::setButton(0, stable_ & 1);
        UsbHidGamepad::setButton(1, stable_ & 2);
    }
};

HidToggle3Pos hidToggle(kToggleUpPin, kToggleDownPin);

// Temperature control pot on GP29 (ADC3). Flip kPotInverted if DCS sees the
// axis travel backwards.
static constexpr uint kPotPin = 29;
static constexpr uint kPotAdcInput = 3;
static constexpr bool kPotInverted = true;

class HidPotAxis {
private:
    static const unsigned long kSampleMs = 5;
    static const int kEwmaShift = 3;   // alpha = 1/8
    static const int32_t kDeadband = 64;  // 16-bit counts (~4 ADC counts)

    int32_t filtered_ = -1;       // -1 = uninitialised
    int32_t lastSent_ = -1;
    unsigned long lastSampleMs_ = 0;

public:
    void poll() {
        const unsigned long now = to_ms_since_boot(get_absolute_time());
        if ((now - lastSampleMs_) < kSampleMs) return;
        lastSampleMs_ = now;

        adc_select_input(kPotAdcInput);
        const int32_t raw = adc_read();  // 0..4095

        if (filtered_ < 0) filtered_ = raw;
        filtered_ += (raw - filtered_) >> kEwmaShift;  // EWMA toward raw

        // ADC 0..4095 -> signed 16-bit axis -32768..32767
        int32_t scaled = (filtered_ * 65535 + 2047) / 4095 - 32768;
        if (kPotInverted) scaled = -1 - scaled;

        if (lastSent_ < 0 || scaled - lastSent_ > kDeadband || lastSent_ - scaled > kDeadband) {
            lastSent_ = scaled;
            UsbHidGamepad::setAxis((int16_t)scaled);
        }
    }
};

HidPotAxis hidPot;

// ---------------------------------------------------------------------------
// Output — console lighting strip
// ---------------------------------------------------------------------------

void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)(((uint64_t)consoleBrightness * 255) / 65535);
#ifdef ECS_DIAGNOSTIC
    printf("[DIAG] CONSOLE callback value=%u\n", consoleBrightness);
#endif

    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgb(intensity, 0, 0));
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// ---------------------------------------------------------------------------
// TEMPORARY diagnostics (define ECS_DIAGNOSTIC to enable, remove when fixed):
//  - core1_diag_task replaces DcsBios::core1_task: identical USB_ONLY behaviour
//    (getchar -> FIFO push, FIFO pop -> putchar) plus an RX byte counter and
//    periodic printf so the host can see whether the CDC RX path delivers
//    bytes to the parser.
// ---------------------------------------------------------------------------
#ifdef ECS_DIAGNOSTIC

volatile uint32_t g_dbg_rx_bytes = 0;
volatile uint32_t g_dbg_rx_words = 0;   // 0x55 bytes seen (sync detection aid)

void core1_diag_task() {
    if (watchdog_caused_reboot()) {
        sleep_ms(100);
    }
    watchdog_enable(WATCHDOG_TIMEOUT, true);

    while (true) {
        watchdog_update();

        // Alive marker, also rate-limited to 5 s.
        static absolute_time_t lastAlive = { 0 };
        static uint32_t aliveTicks = 0;
        const absolute_time_t nowA = get_absolute_time();
        if (absolute_time_diff_us(lastAlive, nowA) > 5000000) {
            lastAlive = nowA;
            printf("[DIAG] alive t=%us rx=%u\n", ++aliveTicks * 5, g_dbg_rx_bytes);
        }

        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            g_dbg_rx_bytes++;
            if ((uint8_t)ch == 0x55) g_dbg_rx_words++;
            multicore_fifo_push_timeout_us((uint32_t)ch, 1000);
            // Rate-limited to once per 5 s so it cannot flood the CDC line
            // while a DCS client streams binary data.
            static absolute_time_t lastPrint = { 0 };
            const absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(lastPrint, now) > 5000000) {
                lastPrint = now;
                printf("[DIAG] rx=%u 0x55count=%u\n", g_dbg_rx_bytes, g_dbg_rx_words);
            }
        }

        if (multicore_fifo_rvalid()) {
            putchar((char)multicore_fifo_pop_blocking());
            sleep_us(10);
        }
    }
}

#endif // ECS_DIAGNOSTIC

// ---------------------------------------------------------------------------

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);  // Wait for USB CDC to be ready

    adc_init();
    adc_gpio_init(kPotPin);
    UsbHidGamepad::begin();

    bool ledsOk = externalLeds.begin(NUM_LEDS);
    if (!ledsOk) {
        printf("[BOOT] LED begin FAILED\n");
    } else {
        printf("[BOOT] LED begin OK\n");
    }

#ifdef ECS_DIAGNOSTIC
    // Strip hardware walk: red, green, blue — proves the LED chain works
    // regardless of DCS. Also marks the diagnostics build.
    static const uint8_t walk[3][3] = { {201, 0, 0}, {0, 201, 0}, {0, 0, 201} };
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < NUM_LEDS; i++) {
            externalLeds.setPixel(i, externalLeds.rgb(walk[c][0], walk[c][1], walk[c][2]));
        }
        externalLeds.show();
        printf("[DIAG] boot walk color %d\n", c);
        sleep_ms(1000);
    }
    printf("[DIAG] boot complete, entering main loop\n");
#else
    // Power-on green flash.
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgb(0, 201, 0));
    }
    externalLeds.show();
    sleep_ms(5000);  // Long power-on green flash, as on the other panels

    // Clear LEDs after the flash.
    externalLeds.clear();
    externalLeds.show();
#endif

    DcsBios::BoardMode board = DcsBios::determineBoardMode(kBoardAddress);
    DcsBios::currentBoardMode = board;

    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

#ifdef ECS_DIAGNOSTIC
    multicore_launch_core1(core1_diag_task);
    printf("[DIAG] core1 diag task launched, board mode %d\n", (int)board.mode);
#else
    multicore_launch_core1(DcsBios::core1_task);
#endif

    DcsBios::setup();

    while (true) {
        hidToggle.poll();
        hidPot.poll();
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        UsbHidGamepad::task();
        sleep_us(10);
    }
}
