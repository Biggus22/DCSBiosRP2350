// F-4E Weapons Panel — RP2350-Zero + CH32V003 I2C sub-panel
//
// The RP2350 carries the five pylon arm switches, the panel dim knob and the
// 12 SK6812 lamps. The CH32V003 on the same PCB carries the gun arm switch,
// the 8-position weapon select and the bomb quantity selector, and reports
// them over i2c1.
//
// The master arm switch, the delivery mode rotary and the bomb interval knob
// are not wired on this board yet; their inputs are commented out below so an
// unwired pin cannot report noise to the sim. The lamps are driven purely by
// the sim's output - the green "_SELECTED" lamps are the sim's weapon station
// selection, the amber "_ARMED" lamps are the sim's arm state.
//
// Board mode is a compile-time constant: 0xF is USB-only (bench), 0x1-0xD is
// RS485 slave. GP36-39 are the RP2350-Zero's QSPI flash pins, so the framework's
// default ADDR0-3 strap scheme is unavailable on this board.
//
// I2C sub-panel payload (7 bytes, appended to the standard layout):
//   [0]      state0  bit0 = CH32 PC0 (gun arm) level
//   [1..2]   adc0    raw 10-bit ADC of CH32 PA2 (bomb quantity)
//   [3..4]   adc1    unused
//   [5]      state2  bit i = weapon select position i pin level (one-hot)
//   [6]      value0  bomb quantity position 0..11, thresholded on the CH32
// The wire convention is pin levels: bit SET = pin HIGH (released), bit CLEAR =
// pin LOW (thrown/selected). That mirrors gpio_get() so the master's active-LOW
// logic holds without inversion.

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

#define NUM_LEDS 12
#define LED_PIN 27

// 0xF = USB-only. 0x0 = RS485 host, 0x1-0xD = RS485 slave, 0xE = RS485 terminal.
static constexpr uint8_t kBoardAddress = 0xF;

static constexpr uint8_t kPanelSlaveAddr = 0x08;  // CH32V003 7-bit I2C address
static constexpr uint8_t kPanelRegId = 0x01;       // panel register
static constexpr uint8_t kPanelCmd = 0x05;         // CMD_STATUS_REQ
static constexpr uint8_t kPanelPayloadLen = 7;     // state0, adc0, adc1, state2, value0
static constexpr uint32_t kPanelPollMs = 20;

// Bit positions in the CH32's first state byte.
static constexpr uint8_t kBitGunArm = 1 << 0;  // CH32 PC0 level

// PLT_WPN_DELIVERY_MODE: 16-position switch, four coded lines weighted 1/2/4/8.
// GP10 and GP11 are left alone - they carry the switch commons / spares.
static constexpr uint8_t kDeliveryBcd1 = 4;
static constexpr uint8_t kDeliveryBcd2 = 5;
static constexpr uint8_t kDeliveryBcd4 = 8;
static constexpr uint8_t kDeliveryBcd8 = 9;
// DCS-BIOS accepts 0..12 for this control, so the last three detents are ignored.
static constexpr unsigned int kDeliveryMaxPosition = 12;

WS2812 externalLeds(pio0, 0, LED_PIN, false);

uart_inst_t *rs485_uart = uart0;

// ---------------------------------------------------------------------------
// Lamp handling
// ---------------------------------------------------------------------------
//
// The 12 SK6812 pixels are driven by the DCS-BIOS lamp outputs: the six
// _SELECTED lamps are green, the six _ARMED lamps are yellow. Every lit lamp
// is scaled by the PANEL_DIM knob, so the stock DcsBios::LED class cannot be
// used - its brightness is fixed at construction time.

struct LedSlot {
    bool on = false;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

static LedSlot ledSlots[NUM_LEDS];
static unsigned int dimValue_ = 0;      // 0..65535, from the PANEL_DIM pot
static unsigned int renderedDim_ = 0;
static bool stripDirty_ = true;

static void renderStrip(uint8_t level) {
    for (int i = 0; i < NUM_LEDS; i++) {
        if (!ledSlots[i].on) {
            externalLeds.setPixel((uint16_t)i, externalLeds.rgb(0, 0, 0));
            continue;
        }
        const uint8_t r = (uint8_t)(((uint16_t)ledSlots[i].r * level) / 255);
        const uint8_t g = (uint8_t)(((uint16_t)ledSlots[i].g * level) / 255);
        const uint8_t b = (uint8_t)(((uint16_t)ledSlots[i].b * level) / 255);
        externalLeds.setPixel((uint16_t)i, externalLeds.rgb(r, g, b));
    }

    externalLeds.show();
    stripDirty_ = false;
    renderedDim_ = dimValue_;
}

// One SK6812 pixel fed by a DCS-BIOS lamp output.
class PanelLed : public DcsBios::Int16Buffer {
public:
    PanelLed(unsigned int address, unsigned int mask, uint8_t pixel,
             uint8_t r, uint8_t g, uint8_t b)
        : DcsBios::Int16Buffer(address), mask_(mask), pixel_(pixel) {
        ledSlots[pixel].r = r;
        ledSlots[pixel].g = g;
        ledSlots[pixel].b = b;
    }

    void loop() override {
        if (!hasUpdatedData()) return;
        ledSlots[pixel_].on = (getData() & mask_) != 0;
        stripDirty_ = true;
    }

private:
    unsigned int mask_;
    uint8_t pixel_;
};

// Lamp colours: the _SELECTED lamps are green, the _ARMED lamps are amber.
// The amber is a touch more orange than pure yellow - the real panel lamps
// are not yellow. Tune the three components here.
static constexpr uint8_t kSelectedR = 0, kSelectedG = 255, kSelectedB = 0;
static constexpr uint8_t kArmedR = 255, kArmedG = 150, kArmedB = 0;

// Pixel order follows the PCB silkscreen: LED1..LED12.
PanelLed ledGunSelected(F_4E_PLT_WPN_GUN_SELECTED_AM, 0, kSelectedR, kSelectedG, kSelectedB);
PanelLed ledGunArmed(F_4E_PLT_WPN_GUN_ARMED_AM, 1, kArmedR, kArmedG, kArmedB);
PanelLed ledRoSelected(F_4E_PLT_WPN_RO_SELECTED_AM, 2, kSelectedR, kSelectedG, kSelectedB);
PanelLed ledRoArmed(F_4E_PLT_WPN_RO_ARMED_AM, 3, kArmedR, kArmedG, kArmedB);
PanelLed ledRiSelected(F_4E_PLT_WPN_RI_SELECTED_AM, 4, kSelectedR, kSelectedG, kSelectedB);
PanelLed ledRiArmed(F_4E_PLT_WPN_RI_ARMED_AM, 5, kArmedR, kArmedG, kArmedB);
PanelLed ledCenterSelected(F_4E_PLT_WPN_CENTER_SELECTED_AM, 6, kSelectedR, kSelectedG, kSelectedB);
PanelLed ledCenterArmed(F_4E_PLT_WPN_CENTER_ARMED_AM, 7, kArmedR, kArmedG, kArmedB);
PanelLed ledLiSelected(F_4E_PLT_WPN_LI_SELECTED_AM, 8, kSelectedR, kSelectedG, kSelectedB);
PanelLed ledLiArmed(F_4E_PLT_WPN_LI_ARMED_AM, 9, kArmedR, kArmedG, kArmedB);
PanelLed ledLoSelected(F_4E_PLT_WPN_LO_SELECTED_AM, 10, kSelectedR, kSelectedG, kSelectedB);
PanelLed ledLoArmed(F_4E_PLT_WPN_LO_ARMED_AM, 11, kArmedR, kArmedG, kArmedB);

// ---------------------------------------------------------------------------
// Panel inputs on the RP2350
// ---------------------------------------------------------------------------
//
// The switches are read active LOW (input pull-up, switch common to GND).
//
// The DCS-BIOS F-4E module defines the pylon arm switches and the gun arm as
// push buttons (definePushButton, api_variant momentary_last_position), where
// 0 = pressed = armed and 1 = released = safe. The master arm is a toggle
// switch (defineToggleSwitch), where 1 = on = armed. The panel's default
// convention (pin low -> "1") is therefore right for the master arm and
// inverted for every push button, which is what the reverse flag is for.
//
// The debounce is 100ms rather than the default 50ms: a toggle switch's
// contacts can bounce for long enough to show a "stable" window in the middle
// of the bounce, and a spurious release makes the sim drop the arm again,
// which shows up as a flickering arm lamp.

// The master arm switch is not wired on this board yet. Uncomment when it is:
// DcsBios::Switch2Pos pltWpnMasterArm("PLT_WPN_MASTER_ARM", 3, false, 100);
DcsBios::Switch2Pos pltWpnRoArm("PLT_WPN_RO_ARM", 12, false, 100);
DcsBios::Switch2Pos pltWpnRiArm("PLT_WPN_RI_ARM", 13, true, 100);
DcsBios::Switch2Pos pltWpnCenterArm("PLT_WPN_CENTER_ARM", 14, true, 100);
DcsBios::Switch2Pos pltWpnLiArm("PLT_WPN_LI_ARM", 15, true, 100);
DcsBios::Switch2Pos pltWpnLoArm("PLT_WPN_LO_ARM", 26, true, 100);

// The delivery mode rotary is not wired on this board yet. Uncomment when it
// is - the four coded lines are weighted 1/2/4/8 and the control's range is
// 0..12, so the last three detents of the 16 position switch are ignored.
// DcsBios::BcdWheelValue pltWpnDeliveryMode("PLT_WPN_DELIVERY_MODE",
//                                           kDeliveryBcd1, kDeliveryBcd2,
//                                           kDeliveryBcd4, kDeliveryBcd8,
//                                           kDeliveryMaxPosition);

// GP26-29 are the only four ADC pins and PotentiometerEWMA hard-maps
// 26/27/28/29 to ADC0/1/2/3. GP26 is a switch here and GP27 drives the LED
// strip, so the knobs sit on ADC2 (GP28) and ADC3 (GP29).
// The dim knob is reversed, so that turning it towards the bright end of the
// pot raises the value (and with it the brightness of the lamps).
// The bomb interval knob is not wired on this board yet. Uncomment when it is:
// DcsBios::Potentiometer pltWpnBombInterval("PLT_WPN_BOMB_INTERVAL", 28, false, 0, 4095);
DcsBios::Potentiometer pltWpnPanelDim("PLT_WPN_PANEL_DIM", 29, true, 0, 4095);

// ---------------------------------------------------------------------------
// Sub-panel inputs - CH32V003 over I2C
// ---------------------------------------------------------------------------

static DcsBios::I2cBusHwMaster i2cBus;  // i2c1, GP6 SDA / GP7 SCL, 100 kHz

static DcsBios::I2cInputPanel wpnPanel(&i2cBus, kPanelSlaveAddr, kPanelRegId,
                                       kPanelCmd, kPanelPayloadLen, kPanelPollMs);

// Gun arm switch, CH32 PC0.
DcsBios::I2cInputSwitch2Pos pltWpnGunArm(&wpnPanel, kBitGunArm, "PLT_WPN_GUN_ARM");

// Weapon select, one-hot over the CH32's eight position pins (state byte 2).
DcsBios::I2cInputSwitchMultiPosT<POLL_EVERY_TIME, 8> pltWpnWeaponSelect(
    &wpnPanel, 1, "PLT_WPN_WEAPON_SELECT");

// Bomb quantity, thresholded to 0..11 by the CH32 from its PA2 analog input.
DcsBios::I2cInputValue pltWpnBombQuantity(&wpnPanel, DcsBios::I2C_INPUT_VALUE,
                                          "PLT_WPN_BOMB_QUANTITY");

// ---------------------------------------------------------------------------
// (no boot-time state report)
// ---------------------------------------------------------------------------
//
// DCS-BIOS push buttons (the arm switches are definePushButton,
// api_variant momentary_last_position) are actions, not state sets: sending
// the same value twice flips the sim's arm state. So the panel must report
// a switch position only when it actually changes, never on boot and never
// periodically. The CH32 inputs report naturally on their first poll.
// ---------------------------------------------------------------------------

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);  // Wait for USB CDC to be ready

    if (!externalLeds.begin(NUM_LEDS)) {
        // The strip stays dark. There is no output to report this on: the USB
        // link carries DCS-BIOS messages only.
    }

    // Power-on green flash.
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel((uint16_t)i, externalLeds.rgb(0, 201, 0));
    }
    externalLeds.show();
    sleep_ms(500);

    // Clear LEDs after the flash.
    externalLeds.clear();
    externalLeds.show();

    DcsBios::BoardMode board = DcsBios::determineBoardMode(kBoardAddress);
    DcsBios::currentBoardMode = board;

    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);

    DcsBios::setup();

    while (true) {
        wpnPanel.poll();  // Before DcsBios::loop() so the remote inputs see fresh data
        DcsBios::loop();
        DcsBios::updateHeartbeat();

        // The panel dim knob feeds the DCS-BIOS control and the lamps at the
        // same time, so the strip follows the same knob.
        dimValue_ = pltWpnPanelDim.getValue();
        if (stripDirty_ || dimValue_ != renderedDim_) {
            renderStrip((uint8_t)((dimValue_ * 255) / 65535));
        }

        sleep_us(10);
    }
}
