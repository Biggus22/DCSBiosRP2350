#ifndef WEACT_RP2350B_CORE
#define WEACT_RP2350B_CORE
#endif

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/ST7789.h"
#include "internal/BYJ48_stepper.h"
#include "internal/ws2812.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/gear_flag.h"

// ===========================================================================
// F-4E-45MC Left Subpanel — WeAct RP2350B Core (ST7789 + 28BYJ-48 + Servo)
//
// Hardware:
//   3× ST7789 displays (flaps, slats, gear) on shared SPI1
//   2× 28BYJ-48 steppers via DRV8833 (boost pump gauges, bipolar mode)
//   1× servo (stabilator position gauge)
//   3× gauge backlight LEDs (PWM via transistor)
//   1× AVTR recording indicator LED
//   WS2812 strip for console lighting
//   3-pos toggle (landing/taxi lights)
//   2-pos toggle (emergency brake)
//   Momentary button (weapon stores emergency release)
//   Latched button (ARI circuit breaker)
// ===========================================================================

// ---------------------------------------------------------------------------
// Pin definitions (board-specific, not in FoxConfig.h)
// ---------------------------------------------------------------------------
namespace Pin {
    // LED outputs (transistor-driven)
    constexpr uint GAUGE_BACKLIGHT_1 = 3;
    constexpr uint GAUGE_BACKLIGHT_2 = 4;
    constexpr uint GAUGE_BACKLIGHT_3 = 5;

    // Switches
    constexpr uint SWITCH_3POS_A = 6;
    constexpr uint SWITCH_3POS_B = 7;
    constexpr uint SWITCH_MOMENTARY = 8;
    constexpr uint SWITCH_LATCHED = 9;
    constexpr uint SWITCH_2POS_A = 11;
    constexpr uint SWITCH_2POS_B = 12;

    // WS2812 console lighting
    constexpr uint NEO_DRIVE_PIN = 10;

    // SPI1 (shared for 3× ST7789)
    constexpr uint SPI1_SCK = 14;
    constexpr uint SPI1_MOSI = 15;

    // Display 1 (Flaps) — vertical
    constexpr uint TFT1_CS = 16;
    constexpr uint TFT1_DC = 17;
    constexpr uint TFT1_RST = 18;

    // Display 2 (Slats) — vertical
    constexpr uint TFT2_CS = 19;
    constexpr uint TFT2_DC = 20;
    constexpr uint TFT2_RST = 21;

    // Display 3 (Gear) — horizontal
    constexpr uint TFT3_CS = 22;
    constexpr uint TFT3_DC = 23;
    constexpr uint TFT3_RST = 24;

    // Shared backlight PWM
    constexpr uint TFT_BL = 26;

    // AVTR indicator LED
    constexpr uint LED_AVTR = 13;

    // Stepper A (Boost Pump L) — 28BYJ-48 bipolar via DRV8833
    constexpr uint STEPPER_A_IN1 = 27;
    constexpr uint STEPPER_A_IN2 = 28;
    constexpr uint STEPPER_A_IN3 = 29;
    constexpr uint STEPPER_A_IN4 = 30;
    constexpr uint STEPPER_A_LIMIT = 31;

    // Stepper B (Boost Pump R) — 28BYJ-48 bipolar via DRV8833
    constexpr uint STEPPER_B_IN1 = 32;
    constexpr uint STEPPER_B_IN2 = 33;
    constexpr uint STEPPER_B_IN3 = 34;
    constexpr uint STEPPER_B_IN4 = 35;
    constexpr uint STEPPER_B_LIMIT = 36;

    // Servo (Stabilator — placeholder)
    constexpr uint SERVO_PIN = 37;
}

// ---------------------------------------------------------------------------
// Display instances
// ---------------------------------------------------------------------------
static ST7789* flapsTft  = nullptr;
static ST7789* slatsTft  = nullptr;
static ST7789* gearTft   = nullptr;

// ---------------------------------------------------------------------------
// Deferred display update flags
// ---------------------------------------------------------------------------
static volatile bool flapsDirty = false;
static volatile bool slatsDirty = false;
static volatile bool gearDirty  = false;

// ---------------------------------------------------------------------------
// Last-known values (to avoid redundant renders)
// ---------------------------------------------------------------------------
static int lastFlapsState  = -1;
static int lastSlatsState  = -1;
static int lastGearNose    = -1;
static int lastGearLeft    = -1;
static int lastGearRight   = -1;

// ---------------------------------------------------------------------------
// Gear flag animation state
// ---------------------------------------------------------------------------
static uint16_t gearAnimOffset  = 0;
static int      gearTargetState = 2;

// Target scroll offsets for each gear state
static const uint16_t GEAR_OFFSET_UP    = 0;
static const uint16_t GEAR_OFFSET_TRANS = 224;
static const uint16_t GEAR_OFFSET_DOWN  = 614; // 934 - 320

// ---------------------------------------------------------------------------
// Display update timing
// ---------------------------------------------------------------------------
static absolute_time_t lastDisplayUpdate;

// ---------------------------------------------------------------------------
// Stepper motors (28BYJ-48, bipolar mode via DRV8833)
// ---------------------------------------------------------------------------
static byj_motor_t stepperA;  // Boost Pump L
static byj_motor_t stepperB;  // Boost Pump R

// 350° sweep in steps: 2048 * (350/360) ≈ 1991
#define BYJ48_SWEEP_STEPS 1991

// ---------------------------------------------------------------------------
// WS2812 console lighting
// ---------------------------------------------------------------------------
#define NUM_CONSOLE_LEDS 6
static WS2812 consoleLeds(pio0, 0, Pin::NEO_DRIVE_PIN, false);

// ---------------------------------------------------------------------------
// UART instance for RS485
// ---------------------------------------------------------------------------
static uart_inst_t* rs485_uart = uart0;

// ===========================================================================
// DCS-BIOS Callbacks — Displays
// ===========================================================================

void onFlapsIndicatorChange(unsigned int newValue) {
    if ((int)newValue != lastFlapsState) {
        lastFlapsState = (int)newValue;
        flapsDirty = true;
    }
}
DcsBios::IntegerBuffer flapsIndicatorBuffer(F_4E_PLT_CONTROLS_FLAPS_INDICATOR, onFlapsIndicatorChange);

void onSlatsIndicatorChange(unsigned int newValue) {
    if ((int)newValue != lastSlatsState) {
        lastSlatsState = (int)newValue;
        slatsDirty = true;
    }
}
DcsBios::IntegerBuffer slatsIndicatorBuffer(F_4E_PLT_CONTROLS_SLATS_INDICATOR, onSlatsIndicatorChange);

static void updateGearTarget() {
    // Priority: DOWN > TRANS > UP
    int newTarget = 2; // default UP
    for (int v : {lastGearLeft, lastGearNose, lastGearRight}) {
        if (v == 0) { newTarget = 0; break; }
        if (v == 1) { newTarget = 1; }
    }
    if (newTarget != gearTargetState) {
        gearTargetState = newTarget;
        gearDirty = true;
    }
}

void onGearNoseChange(unsigned int newValue) {
    if ((int)newValue != lastGearNose) {
        lastGearNose = (int)newValue;
        updateGearTarget();
    }
}
DcsBios::IntegerBuffer gearNoseBuffer(F_4E_PLT_GEAR_INDICATOR_NOSE, onGearNoseChange);

void onGearLeftChange(unsigned int newValue) {
    if ((int)newValue != lastGearLeft) {
        lastGearLeft = (int)newValue;
        updateGearTarget();
    }
}
DcsBios::IntegerBuffer gearLeftBuffer(F_4E_PLT_GEAR_INDICATOR_LEFT, onGearLeftChange);

void onGearRightChange(unsigned int newValue) {
    if ((int)newValue != lastGearRight) {
        lastGearRight = (int)newValue;
        updateGearTarget();
    }
}
DcsBios::IntegerBuffer gearRightBuffer(F_4E_PLT_GEAR_INDICATOR_RIGHT, onGearRightChange);

// ===========================================================================
// DCS-BIOS Callbacks — Steppers (Boost Pump Gauges)
// ===========================================================================

void onBoostPumpLChange(unsigned int newValue) {
    int32_t pos = (int32_t)((uint64_t)newValue * BYJ48_SWEEP_STEPS / 65535);
    byj_set_position(&stepperA, pos);
}
DcsBios::IntegerBuffer boostPumpLBuffer(F_4E_PLT_FUEL_BOOST_PUMP_L, onBoostPumpLChange);

void onBoostPumpRChange(unsigned int newValue) {
    int32_t pos = (int32_t)((uint64_t)newValue * BYJ48_SWEEP_STEPS / 65535);
    byj_set_position(&stepperB, pos);
}
DcsBios::IntegerBuffer boostPumpRBuffer(F_4E_PLT_FUEL_BOOST_PUMP_R, onBoostPumpRChange);

// ===========================================================================
// DCS-BIOS Callbacks — Console Lighting (gauge backlights + WS2812 + AVTR)
// ===========================================================================

void onConsoleLightChange(unsigned int newValue) {
    // WS2812 console strip
    uint8_t intensity = (uint8_t)((newValue * 255) / 65535);
    for (int i = 0; i < NUM_CONSOLE_LEDS; i++) {
        consoleLeds.setPixel(i, consoleLeds.rgbw(intensity, 0, 0, 0));
    }
    consoleLeds.show();

    // Gauge backlight PWM (all three share the same dimming curve)
    uint16_t pwmLevel = (uint16_t)((uint64_t)newValue * 65535 / 65535);
    pwm_set_gpio_level(Pin::GAUGE_BACKLIGHT_1, pwmLevel);
    pwm_set_gpio_level(Pin::GAUGE_BACKLIGHT_2, pwmLevel);
    pwm_set_gpio_level(Pin::GAUGE_BACKLIGHT_3, pwmLevel);
}
DcsBios::IntegerBuffer consoleLightBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onConsoleLightChange);

// ===========================================================================
// DCS-BIOS Callbacks — AVTR Recording Indicator
// ===========================================================================

void onAvtrRecorderLightChange(unsigned int newValue) {
    gpio_put(Pin::LED_AVTR, newValue ? 1 : 0);
}
DcsBios::IntegerBuffer avtrLightBuffer(F_4E_PLT_AVTR_RECORDER_LIGHT, onAvtrRecorderLightChange);

// ===========================================================================
// Switches
// ===========================================================================

// 3-pos toggle: landing/taxi lights (TAXI=0, OFF=1, LAND=2)
DcsBios::Switch3Pos2PinT<POLL_EVERY_TIME> pltExtLightTaxiLand(
    "PLT_EXT_LIGHT_TAXI_LAND",
    Pin::SWITCH_3POS_A, Pin::SWITCH_3POS_B
);

// 2-pos toggle: emergency brake
DcsBios::Switch2Pos pltGearBrakesEmergency(
    "PLT_GEAR_BRAKES_EMERGENCY",
    Pin::SWITCH_2POS_A
);

// Momentary button: weapon stores emergency release
DcsBios::ActionButton pltWpnStoresEmergencyRelease(
    "PLT_WPN_STORES_EMERGENCY_RELEASE", "1",
    Pin::SWITCH_MOMENTARY
);

// Latched button: ARI circuit breaker
DcsBios::Switch2Pos pltCbAri(
    "PLT_CB_ARI",
    Pin::SWITCH_LATCHED
);

// ===========================================================================
// Display rendering helpers
// ===========================================================================

static void renderFlapsDisplay() {
    if (!flapsTft) return;
    flapsTft->fillScreen(0x0000);

    const char* label = "FLAPS";
    const char* value;
    switch (lastFlapsState) {
        case 0: value = "UP";   break;
        case 1: value = "DOWN"; break;
        default: value = "---"; break;
    }

    uint16_t labelW = (uint16_t)(strlen(label) * 5 * 2);
    uint16_t labelX = (flapsTft->width() > labelW) ? (flapsTft->width() - labelW) / 2 : 0;
    flapsTft->drawStringScaled(labelX, 10, label, 0xFFFF, 2);

    uint16_t valW = (uint16_t)(strlen(value) * 5 * 4);
    uint16_t valX = (flapsTft->width() > valW) ? (flapsTft->width() - valW) / 2 : 0;
    uint16_t valY = (flapsTft->height() > 32) ? (flapsTft->height() - 32) / 2 : 0;
    flapsTft->drawStringScaled(valX, valY, value, 0xFFFF, 4);
}

static void renderSlatsDisplay() {
    if (!slatsTft) return;
    slatsTft->fillScreen(0x0000);

    const char* label = "SLATS";
    const char* value;
    switch (lastSlatsState) {
        case 0: value = "RETRACT"; break;
        case 1: value = "AUTO";    break;
        case 2: value = "EXTEND";  break;
        default: value = "---";     break;
    }

    uint16_t labelW = (uint16_t)(strlen(label) * 5 * 2);
    uint16_t labelX = (slatsTft->width() > labelW) ? (slatsTft->width() - labelW) / 2 : 0;
    slatsTft->drawStringScaled(labelX, 10, label, 0xFFFF, 2);

    uint16_t valW = (uint16_t)(strlen(value) * 5 * 3);
    uint16_t valX = (slatsTft->width() > valW) ? (slatsTft->width() - valW) / 2 : 0;
    uint16_t valY = (slatsTft->height() > 24) ? (slatsTft->height() - 24) / 2 : 0;
    slatsTft->drawStringScaled(valX, valY, value, 0xFFFF, 3);
}

static void renderGearDisplay() {
    if (!gearTft) return;

    uint16_t target;
    switch (gearTargetState) {
        case 0:  target = GEAR_OFFSET_DOWN;  break;
        case 1:  target = GEAR_OFFSET_TRANS; break;
        default: target = GEAR_OFFSET_UP;    break;
    }

    // Animate toward target
    const uint16_t step = 24;
    if (gearAnimOffset < target) {
        gearAnimOffset += step;
        if (gearAnimOffset > target) gearAnimOffset = target;
        else gearDirty = true; // keep animating next frame
    } else if (gearAnimOffset > target) {
        if (gearAnimOffset < step) gearAnimOffset = 0;
        else gearAnimOffset -= step;
        if (gearAnimOffset < target) gearAnimOffset = target;
        else gearDirty = true; // keep animating next frame
    }

    gearTft->drawScrollingBitmap(gearFlagBitmap, GEAR_FLAG_HEIGHT, gearAnimOffset);
}

// ===========================================================================
// Stepper homing
// ===========================================================================

static void homeStepper(byj_motor_t* motor, int limitPin, const char* name) {
    printf("Homing %s (CCW toward limit switch on GP%d)...\n", name, limitPin);

    gpio_init(limitPin);
    gpio_set_dir(limitPin, GPIO_IN);
    gpio_pull_down(limitPin);

    const uint32_t maxSteps = BYJ48_STEPS_PER_REV;
    uint32_t steps = 0;

    while (steps < maxSteps) {
        if (gpio_get(limitPin)) {
            printf("  %s homed: limit triggered at step %lu\n", name, (unsigned long)steps);
            motor->current_position = 0;
            motor->target_position = 0;
            byj_sleep(motor);
            return;
        }
        byj_step_once(motor, -1);
        steps++;
        busy_wait_us(motor->step_delay_us);
    }

    printf("  %s WARNING: limit not reached after %lu steps, assuming position 0\n",
           name, (unsigned long)maxSteps);
    motor->current_position = 0;
    motor->target_position = 0;
    byj_sleep(motor);
}

// ===========================================================================
// main()
// ===========================================================================

int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("F-4E Instrument Panel starting...\n");

    DcsBios::initHeartbeat(HEARTBEAT_LED);

    // Board mode: USB_ONLY (address 0xF)
    uint8_t boardAddress = 0xF;
    DcsBios::currentBoardMode = DcsBios::determineBoardMode(boardAddress);
    printf("Board mode: USB_ONLY (address 0x%X)\n", boardAddress);

    // RS485 UART
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Launch Core 1 task
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched\n");

    // -----------------------------------------------------------------------
    // SPI1 init (shared for 3× ST7789)
    // -----------------------------------------------------------------------
    spi_init(spi1, 10 * 1000 * 1000);
    gpio_set_function(Pin::SPI1_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(Pin::SPI1_SCK, GPIO_FUNC_SPI);

    // -----------------------------------------------------------------------
    // AVTR LED (raw GPIO, initialized early so callback can use it)
    // -----------------------------------------------------------------------
    gpio_init(Pin::LED_AVTR);
    gpio_set_dir(Pin::LED_AVTR, GPIO_OUT);
    gpio_put(Pin::LED_AVTR, 0);

    // -----------------------------------------------------------------------
    // Gauge backlight PWM pins (initialized early for console lighting callback)
    // -----------------------------------------------------------------------
    gpio_set_function(Pin::GAUGE_BACKLIGHT_1, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(Pin::GAUGE_BACKLIGHT_1);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(Pin::GAUGE_BACKLIGHT_1, 0);
    }
    gpio_set_function(Pin::GAUGE_BACKLIGHT_2, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(Pin::GAUGE_BACKLIGHT_2);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(Pin::GAUGE_BACKLIGHT_2, 0);
    }
    gpio_set_function(Pin::GAUGE_BACKLIGHT_3, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(Pin::GAUGE_BACKLIGHT_3);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(Pin::GAUGE_BACKLIGHT_3, 0);
    }

    // -----------------------------------------------------------------------
    // Display 1 — Flaps (vertical, rotation=3)
    // -----------------------------------------------------------------------
    gpio_init(Pin::TFT1_CS);   gpio_set_dir(Pin::TFT1_CS, GPIO_OUT); gpio_put(Pin::TFT1_CS, 1);
    gpio_init(Pin::TFT1_DC);   gpio_set_dir(Pin::TFT1_DC, GPIO_OUT); gpio_put(Pin::TFT1_DC, 0);
    gpio_init(Pin::TFT1_RST);  gpio_set_dir(Pin::TFT1_RST, GPIO_OUT);
    gpio_init(Pin::TFT_BL);    gpio_set_dir(Pin::TFT_BL, GPIO_OUT);  gpio_put(Pin::TFT_BL, 1);

    gpio_put(Pin::TFT1_RST, 0); sleep_ms(10); gpio_put(Pin::TFT1_RST, 1); sleep_ms(120);

    flapsTft = new ST7789(spi1, Pin::TFT1_CS, Pin::TFT1_DC, Pin::TFT1_RST, Pin::TFT_BL);
    if (flapsTft) {
        flapsTft->init(10 * 1000 * 1000);
        flapsTft->setRotation(3);
        flapsTft->fillScreen(0x0000);
        flapsTft->setBacklight(128);
    }
    printf("Flaps display initialized\n");

    // -----------------------------------------------------------------------
    // Display 2 — Slats (vertical, rotation=3)
    // -----------------------------------------------------------------------
    gpio_init(Pin::TFT2_CS);   gpio_set_dir(Pin::TFT2_CS, GPIO_OUT); gpio_put(Pin::TFT2_CS, 1);
    gpio_init(Pin::TFT2_DC);   gpio_set_dir(Pin::TFT2_DC, GPIO_OUT); gpio_put(Pin::TFT2_DC, 0);
    gpio_init(Pin::TFT2_RST);  gpio_set_dir(Pin::TFT2_RST, GPIO_OUT);

    gpio_put(Pin::TFT2_RST, 0); sleep_ms(10); gpio_put(Pin::TFT2_RST, 1); sleep_ms(120);

    slatsTft = new ST7789(spi1, Pin::TFT2_CS, Pin::TFT2_DC, Pin::TFT2_RST, Pin::TFT_BL);
    if (slatsTft) {
        slatsTft->init(10 * 1000 * 1000);
        slatsTft->setRotation(3);
        slatsTft->fillScreen(0x0000);
        slatsTft->setBacklight(128);
    }
    printf("Slats display initialized\n");

    // -----------------------------------------------------------------------
    // Display 3 — Gear (horizontal, rotation=0)
    // -----------------------------------------------------------------------
    gpio_init(Pin::TFT3_CS);   gpio_set_dir(Pin::TFT3_CS, GPIO_OUT); gpio_put(Pin::TFT3_CS, 1);
    gpio_init(Pin::TFT3_DC);   gpio_set_dir(Pin::TFT3_DC, GPIO_OUT); gpio_put(Pin::TFT3_DC, 0);
    gpio_init(Pin::TFT3_RST);  gpio_set_dir(Pin::TFT3_RST, GPIO_OUT);

    gpio_put(Pin::TFT3_RST, 0); sleep_ms(10); gpio_put(Pin::TFT3_RST, 1); sleep_ms(120);

    gearTft = new ST7789(spi1, Pin::TFT3_CS, Pin::TFT3_DC, Pin::TFT3_RST, Pin::TFT_BL);
    if (gearTft) {
        gearTft->init(10 * 1000 * 1000);
        gearTft->setRotation(0);
        gearTft->fillScreen(0x0000);
        gearTft->setBacklight(128);
    }
    printf("Gear display initialized\n");

    // -----------------------------------------------------------------------
    // WS2812 console lighting
    // -----------------------------------------------------------------------
    consoleLeds.begin(NUM_CONSOLE_LEDS);
    consoleLeds.clear();
    consoleLeds.show();
    printf("WS2812 console lighting initialized\n");

    // -----------------------------------------------------------------------
    // Stepper A (Boost Pump L)
    // -----------------------------------------------------------------------
    const byj_gpio_config_t stepperACfg = {
        Pin::STEPPER_A_IN1, Pin::STEPPER_A_IN2, Pin::STEPPER_A_IN3, Pin::STEPPER_A_IN4
    };
    byj_init_gpio(&stepperA, &stepperACfg, BYJ_MODE_FULL_DOUBLE);
    byj_set_output_mode(&stepperA, BYJ_OUTPUT_BIPOLAR);
    byj_set_speed(&stepperA, 3000);

    // -----------------------------------------------------------------------
    // Stepper B (Boost Pump R)
    // -----------------------------------------------------------------------
    const byj_gpio_config_t stepperBCfg = {
        Pin::STEPPER_B_IN1, Pin::STEPPER_B_IN2, Pin::STEPPER_B_IN3, Pin::STEPPER_B_IN4
    };
    byj_init_gpio(&stepperB, &stepperBCfg, BYJ_MODE_FULL_DOUBLE);
    byj_set_output_mode(&stepperB, BYJ_OUTPUT_BIPOLAR);
    byj_set_speed(&stepperB, 3000);

    // -----------------------------------------------------------------------
    // Home both steppers (CCW toward limit switch = zero position)
    // -----------------------------------------------------------------------
    homeStepper(&stepperA, Pin::STEPPER_A_LIMIT, "Stepper A (Boost Pump L)");
    homeStepper(&stepperB, Pin::STEPPER_B_LIMIT, "Stepper B (Boost Pump R)");
    printf("Steppers homed\n");

    // -----------------------------------------------------------------------
    // Servo (Stabilator position gauge)
    // -----------------------------------------------------------------------
    DcsBios::ServoOutput stabilatorServo(F_4E_PLT_CONTROLS_STAB_TRIM_A, Pin::SERVO_PIN);

    // -----------------------------------------------------------------------
    // DCS-BIOS setup
    // -----------------------------------------------------------------------
    DcsBios::setup();
    lastDisplayUpdate = get_absolute_time();

    // Initial display render
    renderFlapsDisplay();
    renderSlatsDisplay();
    renderGearDisplay();

    printf("F-4E Instrument Panel ready\n");

    // -----------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------
    while (true) {
        DcsBios::loop();
        DcsBios::updateHeartbeat();

        // Deferred display updates (throttled to 10 Hz)
        if (absolute_time_diff_us(lastDisplayUpdate, get_absolute_time()) > 100000) {
            if (flapsDirty)  { renderFlapsDisplay();  flapsDirty  = false; }
            if (slatsDirty)  { renderSlatsDisplay();  slatsDirty  = false; }
            if (gearDirty)   { renderGearDisplay();   gearDirty   = false; }
            lastDisplayUpdate = get_absolute_time();
        }

        // Stepper updates (non-blocking)
        byj_update(&stepperA);
        byj_update(&stepperB);

        sleep_us(10);
    }
}
