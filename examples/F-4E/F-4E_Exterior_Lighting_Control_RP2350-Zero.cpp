// F-4E Exterior Lighting Control — RP2350-Zero
//
// Panel inputs on the RP2350 (active LOW, internal pull-ups to 3.3V) plus a
// 7-pixel SK6812 console-lighting strip, with a CH32V003 input sub-panel on
// i2c1 for the formation-light MODE toggle and the indexer / SAI
// brightness pots. See exterior_lighting_control_panel_plan.md for the full pin map.
//
// Board mode is a compile-time constant: 0xF is USB-only (bench), 0x1-0xD is
// RS485 slave. GP36-39 are the RP2350-Zero's QSPI flash pins, so the framework's
// default ADDR0-3 strap scheme is unavailable on this board.

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
#include "internal/I2cInputPanel.h"
#include "hardware/adc.h"

#define NUM_LEDS 7
#define LED_PIN 3

// 0xF = USB-only. 0x0 = RS485 host, 0x1-0xD = RS485 slave, 0xE = RS485 terminal.
static constexpr uint8_t kBoardAddress = 0xF;

static constexpr uint8_t kPanelSlaveAddr = 0x08;  // CH32V003 7-bit I2C address
static constexpr uint8_t kPanelRegId = 0x01;       // panel register
static constexpr uint8_t kPanelCmd = 0x05;         // CMD_STATUS_REQ
static constexpr uint8_t kPanelPayloadLen = 5;     // state byte + 2 x uint16 ADC
static constexpr uint32_t kPanelPollMs = 20;

// Bit positions in the CH32's state byte. The wire convention is pin levels:
// bit SET = pin HIGH (released/inactive), bit CLEAR = pin LOW (thrown/active).
// This mirrors gpio_get() so I2cInputSwitch3Pos2Bit's active-LOW logic holds.
static constexpr uint8_t kBitFormationModeA = 1 << 0;  // CH32 PC3 level
static constexpr uint8_t kBitFormationModeB = 1 << 1;  // CH32 PC4 level

// Production USB must carry ONLY "TAG VALUE\n" lines (the DCS-BIOS format).
// Any bare printf (including the panel reporter and boot banners below) breaks
// DCS parsing. Set to 1 for bench debugging ONLY, with DCS disconnected.
#define PANEL_DEBUG 0

WS2812 externalLeds(pio0, 0, LED_PIN, false);

uart_inst_t *rs485_uart = uart0;

// ---------------------------------------------------------------------------
// I2C master -> CH32V003 input sub-panel
// ---------------------------------------------------------------------------

static DcsBios::I2cBusHwMaster i2cBus;  // i2c1, GP6 SDA / GP7 SCL, 100 kHz

static DcsBios::I2cInputPanel remotePanel(
    &i2cBus, kPanelSlaveAddr, kPanelRegId, kPanelCmd, kPanelPayloadLen, kPanelPollMs);

// ---------------------------------------------------------------------------
// Local panel inputs — RP2350 GPIOs
// ---------------------------------------------------------------------------

DcsBios::Switch2Pos pltWcaWarnLightTest("PLT_WCA_WARN_LIGHT_TEST", 9);

// Three-position toggles: A thrown = 0, centre = 1, B thrown = 2.
// Swap the two pins if a switch reports 0 and 2 the wrong way round.
DcsBios::Switch3Pos2Pin pltExtLightFuselage("PLT_EXT_LIGHT_ANTI_COLL", 4, 5);
DcsBios::Switch3Pos2Pin pltExtLightFlashSteady("PLT_EXT_LIGHT_FLASH_MODE", 8, 15);
DcsBios::Switch3Pos2Pin pltExtLightTail("PLT_EXT_LIGHT_TAIL_BRIGHTNESS", 13, 14);
DcsBios::Switch3Pos2Pin pltExtLightWing("PLT_EXT_LIGHT_WING_BRIGHTNESS", 12, 11);

// GP26-29 are the only four ADC pins, and PotentiometerEWMA hard-maps
// 26/27/28/29 to ADC0/1/2/3.
// Analog inputs floating on this panel — commented out.
// DcsBios::Potentiometer pltAoaStallVolume("PLT_AOA_STALL_VOLUME", 26, false, 0, 4095);
// DcsBios::Potentiometer pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", 27, false, 0, 4095);
// DcsBios::Potentiometer pltIntLightInstrumentPanelBrightness("PLT_INT_LIGHT_INSTRUMENT_PANEL_BRIGHTNESS", 28, false, 0, 4095);
// DcsBios::Potentiometer pltExtLightFormationBrightness("PLT_EXT_LIGHT_FORMATION_BRIGHTNESS", 29, false, 0, 4095);

// ---------------------------------------------------------------------------
// Sub-panel inputs — CH32V003 over I2C
// ---------------------------------------------------------------------------

DcsBios::I2cInputSwitch3Pos2Bit pltExtLightFormationMode(
    &remotePanel, kBitFormationModeA, kBitFormationModeB, "PLT_EXT_LIGHT_FORMATION_MODE");

// Analog inputs floating on this panel — commented out.
// DcsBios::I2cInputPotentiometer pltAoaIndexerBrightness(
//     &remotePanel, DcsBios::I2C_INPUT_ADC_0, "PLT_INT_LIGHT_AOA_INDEXER_BRIGHTNESS", false, 0, 1023);

// NOTE: this is the LEFT-wall SAI brightness knob. The right-wall knob is
// non-functional (superseded by the left-wall one in the real aircraft) but
// its hardware stays in place for now.
// DcsBios::I2cInputPotentiometer pltIntLightSaiBrightness(
//     &remotePanel, DcsBios::I2C_INPUT_ADC_1, "PLT_INT_LIGHT_SAI_BRIGHTNESS", false, 0, 1023);

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

#if PANEL_DEBUG
static unsigned long lastPanelPrintMs = 0;
static uint32_t lastReportedFails = 0;

static void reportPanelState() {
    unsigned long now = to_ms_since_boot(get_absolute_time());
    if ((now - lastPanelPrintMs) < 500) return;
    lastPanelPrintMs = now;

    uint16_t adc0 = 0;
    uint16_t adc1 = 0;
    remotePanel.readAdc(DcsBios::I2C_INPUT_ADC_0, &adc0);
    remotePanel.readAdc(DcsBios::I2C_INPUT_ADC_1, &adc1);

    printf("panel: online=%d state=0x%02X adcIndexer=%u adcSai=%u fails=%lu\n",
           (int)remotePanel.online(), remotePanel.readState(), adc0, adc1,
           (unsigned long)remotePanel.failCount());

    // Warn once when a previously healthy link drops, instead of every 500ms.
    if (lastReportedFails == 0 && remotePanel.failCount() > 0) {
        printf("panel: I2C NACKs on slave 0x%02X — check pull-ups, wiring and the CH32\n",
               kPanelSlaveAddr);
    }
    lastReportedFails = remotePanel.failCount();
}
#endif

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);  // Wait for USB CDC to be ready

    adc_init();

    if (!externalLeds.begin(NUM_LEDS)) {
#if PANEL_DEBUG
        printf("WS2812 init FAILED on GPIO%d\n", LED_PIN);
#endif
    }

    // Power-on green flash.
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgb(0, 201, 0));
    }
    externalLeds.show();
    sleep_ms(5000);

    // Clear LEDs after the flash.
    externalLeds.clear();
    externalLeds.show();

    DcsBios::BoardMode board = DcsBios::determineBoardMode(kBoardAddress);
    DcsBios::currentBoardMode = board;
#if PANEL_DEBUG
    printf("Exterior lighting control: board address 0x%X, mode %d\n", kBoardAddress, (int)board.mode);

    switch (board.mode) {
    case DcsBios::BoardModeType::HOST:
        printf("HOST MODE\n");
        break;
    case DcsBios::BoardModeType::SLAVE:
        printf("SLAVE MODE\n");
        break;
    case DcsBios::BoardModeType::USB_ONLY:
        printf("STANDALONE USB MODE\n");
        break;
    case DcsBios::BoardModeType::RS485_TERMINAL:
        printf("RS485 TERMINAL MODE\n");
        break;
    default:
        printf("INVALID ADDRESS\n");
        break;
    }
#endif

    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);
#if PANEL_DEBUG
    printf("Core 1 task launched!\n");
#endif

    DcsBios::setup();
#if PANEL_DEBUG
    printf("DCS-BIOS setup complete!\n");
#endif

    while (true) {
        remotePanel.poll();  // Before DcsBios::loop() so the remote inputs see fresh data
        DcsBios::loop();
        DcsBios::updateHeartbeat();
#if PANEL_DEBUG
        reportPanelState();
#endif
        sleep_us(10);
    }
}
