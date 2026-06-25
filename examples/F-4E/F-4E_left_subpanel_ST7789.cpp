#ifndef WEACT_RP2350B_CORE
#define WEACT_RP2350B_CORE
#endif

#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/ws2812.h"
#include "internal/ST7789.h"
#include "internal/BYJ48_stepper.h"
#include "internal/MS33558_48.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"

// F-4E-45MC Left Subpanel — WeAct RP2350B Core (ST7789 + 28BYJ-48 + Servo)

// Set to 1 to show diagnostic positioning overlay on all displays
#define DIAGNOSTIC_POSITIONING 0

// Calibration mode - set to 0 after positioning is verified
#define CALIBRATION_MODE 1

// Calibration cycle - set to 1 to cycle through all positions at 10s intervals
#define CALIBRATION_CYCLE 1
#define CAL_PHASE_MS 10000

// Combined slats+flaps display layout (landscape, rotation 3)
#define SF_SLATS_X       16
#define SF_FLAPS_X       176
#define SF_REGION_W      128
#define SF_BMP_Y         17
#define SF_BMP_H         138

// Pin definitions (board-specific)
static const uint GAUGE_BACKLIGHT_1 = 3;
static const uint GAUGE_BACKLIGHT_2 = 4;
static const uint GAUGE_BACKLIGHT_3 = 5;
static const uint SWITCH_3POS_A = 6;
static const uint SWITCH_3POS_B = 7;
static const uint SWITCH_MOMENTARY = 8;
static const uint SWITCH_LATCHED = 9;
static const uint NEO_DRIVE_PIN = 10;
static const uint SWITCH_2POS_A = 11;
static const uint SWITCH_2POS_B = 12;
static const uint LED_AVTR = 13;
static const uint SPI1_SCK = 14;
static const uint SPI1_MOSI = 15;
static const uint TFT1_CS = 16;
static const uint TFT1_DC = 17;
static const uint TFT1_RST = 18;
// static const uint TFT2_CS = 19;
// static const uint TFT2_DC = 20;
// static const uint TFT2_RST = 21;
static const uint TFT3_CS = 22;
static const uint TFT3_DC = 23;
static const uint TFT3_RST = 24;
static const uint TFT_BL = 26;
static const uint STEPPER_A_IN1 = 27;
static const uint STEPPER_A_IN2 = 28;
static const uint STEPPER_A_IN3 = 29;
static const uint STEPPER_A_IN4 = 30;
static const uint STEPPER_A_LIMIT = 31;
static const uint STEPPER_B_IN1 = 32;
static const uint STEPPER_B_IN2 = 33;
static const uint STEPPER_B_IN3 = 34;
static const uint STEPPER_B_IN4 = 35;
static const uint STEPPER_B_LIMIT = 36;
static const uint SERVO_PIN = 37;

// Display instances
static ST7789* flapsTft  = nullptr;
static ST7789* gearTft   = nullptr;

// Raw DCS-BIOS values (0-65535, 0xFFFF = no data yet)
static uint16_t slatsRawValue = 0xFFFF;
static uint16_t flapsRawValue = 0xFFFF;
static uint16_t gearRawNose = 0xFFFF;
static uint16_t gearRawLeft = 0xFFFF;
static uint16_t gearRawRight = 0xFFFF;

// Last rendered values (to avoid redundant redraws)
static uint16_t lastSlatsRendered = 0xFFFF;
static uint16_t lastFlapsRendered = 0xFFFF;
static uint16_t lastGearNoseRendered = 0xFFFF;
static uint16_t lastGearLeftRendered = 0xFFFF;
static uint16_t lastGearRightRendered = 0xFFFF;

// Display update timing
static absolute_time_t lastDisplayUpdate;

#if CALIBRATION_CYCLE
static absolute_time_t calPhaseStart;
static uint8_t calPhase = 0;
#endif

// Stepper motors (28BYJ-48, bipolar mode via DRV8833)
static byj_motor_t stepperA;
static byj_motor_t stepperB;

// 350 degree sweep in steps: 2048 * (350/360) = 1991
#define BYJ48_SWEEP_STEPS 1991

// WS2812 console lighting
#define NUM_CONSOLE_LEDS 6
static WS2812 consoleLeds(pio0, 0, NEO_DRIVE_PIN, false);

uart_inst_t *rs485_uart = uart0;

// DCS-BIOS Callbacks - Displays (Raw Values)
void onSlatsRawChange(unsigned int newValue) {
    slatsRawValue = (uint16_t)newValue;
}
DcsBios::IntegerBuffer slatsRawBuffer(0x2dfc, 0xffff, 0, onSlatsRawChange);

void onFlapsRawChange(unsigned int newValue) {
    flapsRawValue = (uint16_t)newValue;
}
DcsBios::IntegerBuffer flapsRawBuffer(0x2dfa, 0xffff, 0, onFlapsRawChange);

void onGearNoseRawChange(unsigned int newValue) {
    gearRawNose = (uint16_t)newValue;
}
DcsBios::IntegerBuffer gearNoseRawBuffer(0x2df2, 0xffff, 0, onGearNoseRawChange);

void onGearLeftRawChange(unsigned int newValue) {
    gearRawLeft = (uint16_t)newValue;
}
DcsBios::IntegerBuffer gearLeftRawBuffer(0x2df0, 0xffff, 0, onGearLeftRawChange);

void onGearRightRawChange(unsigned int newValue) {
    gearRawRight = (uint16_t)newValue;
}
DcsBios::IntegerBuffer gearRightRawBuffer(0x2df4, 0xffff, 0, onGearRightRawChange);

// DCS-BIOS Callbacks - Steppers (Boost Pump Gauges)
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

// DCS-BIOS Callbacks - Console Lighting (gauge backlights + WS2812 + AVTR)
void onConsoleLightChange(unsigned int newValue) {
    uint8_t intensity = (uint8_t)((newValue * 255) / 65535);
    for (int i = 0; i < NUM_CONSOLE_LEDS; i++) {
        consoleLeds.setPixel(i, consoleLeds.rgbw(intensity, 0, 0, 0));
    }
    consoleLeds.show();

    uint16_t pwmLevel = (uint16_t)((uint64_t)newValue * 65535 / 65535);
    pwm_set_gpio_level(GAUGE_BACKLIGHT_1, pwmLevel);
    pwm_set_gpio_level(GAUGE_BACKLIGHT_2, pwmLevel);
    pwm_set_gpio_level(GAUGE_BACKLIGHT_3, pwmLevel);
}
DcsBios::IntegerBuffer consoleLightBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onConsoleLightChange);

// DCS-BIOS Callbacks - AVTR Recording Indicator
void onAvtrRecorderLightChange(unsigned int newValue) {
    gpio_put(LED_AVTR, newValue ? 1 : 0);
}
DcsBios::IntegerBuffer avtrLightBuffer(F_4E_PLT_AVTR_RECORDER_LIGHT, onAvtrRecorderLightChange);

// Switches
DcsBios::Switch3Pos2PinT<POLL_EVERY_TIME> pltExtLightTaxiLand(
    "PLT_EXT_LIGHT_TAXI_LAND",
    SWITCH_3POS_A, SWITCH_3POS_B
);

DcsBios::Switch2Pos pltGearBrakesEmergency(
    "PLT_GEAR_BRAKES_EMERGENCY",
    SWITCH_2POS_A
);

DcsBios::ActionButton pltWpnStoresEmergencyRelease(
    "PLT_WPN_STORES_EMERGENCY_RELEASE", "1",
    SWITCH_MOMENTARY
);

DcsBios::Switch2Pos pltCbAri(
    "PLT_CB_ARI",
    SWITCH_LATCHED
);

// Display rendering helpers

// Deselect all displays before any SPI transaction
static void csDeselectAll() {
    gpio_put(TFT1_CS, 1);
    gpio_put(TFT3_CS, 1);
}


// Draw MS33558 text centered at (cx, cy) using drawPixel.
// String and columns are reversed to compensate for MX=1 at rotation 3.
static void drawMS33558Centered(ST7789* tft, uint16_t cx, uint16_t cy,
                                 const char* s, uint16_t color) {
    // Compute total width
    uint16_t totalW = 0;
    for (const char* p = s; *p; ++p) {
        int idx = MS33558_48_char_map[(int)(uint8_t)*p];
        if (idx < MS33558_48_glyph_count)
            totalW += MS33558_48_glyph_widths[idx] + 2;
    }
    if (totalW < 2) return;
    totalW -= 2;

    uint16_t x0 = cx - totalW / 2;
    uint16_t y0 = cy - MS33558_48_height / 2;

    // Track character widths for reverse positioning
    uint16_t widths[16];
    uint8_t len = 0;
    for (const char* p = s; *p && len < 16; ++p) {
        int idx = MS33558_48_char_map[(int)(uint8_t)*p];
        widths[len++] = (idx < MS33558_48_glyph_count) ? MS33558_48_glyph_widths[idx] : 0;
    }

    // Render characters in reverse order (each character column-reversed)
    uint16_t cursor = x0;
    for (int ci = len - 1; ci >= 0; --ci) {
        const char* p = s + ci;
        int idx = MS33558_48_char_map[(int)(uint8_t)*p];
        if (idx >= MS33558_48_glyph_count) { cursor += 4; continue; }
        uint16_t gw = widths[ci];
        uint32_t off = MS33558_48_glyph_offsets[idx];
        int bpr = (gw + 7) / 8;
        // Render font columns in reverse order (for MX=1 mirror compensation)
        for (int row = 0; row < MS33558_48_height; ++row) {
            for (int col = gw - 1; col >= 0; --col) {
                int bi = off + row * bpr + (col / 8);
                int bit = 7 - (col % 8);
                if (MS33558_48_bitmap[bi] & (1 << bit))
                    tft->drawPixel(cursor + (gw - 1 - col), y0 + row, color);
            }
        }
        cursor += gw + 2;
    }
}

#if CALIBRATION_MODE
static void drawCalibrationOverlay(ST7789* tft) {
    uint16_t green = 0x07E0;
    uint16_t blue  = 0x001F;
    uint16_t yellow = 0xFFE0;

    tft->drawFillRect(SF_SLATS_X, SF_BMP_Y, SF_REGION_W, 1, green);
    tft->drawFillRect(SF_SLATS_X, SF_BMP_Y + SF_BMP_H - 1, SF_REGION_W, 1, green);
    tft->drawFillRect(SF_SLATS_X, SF_BMP_Y, 1, SF_BMP_H, green);
    tft->drawFillRect(SF_SLATS_X + SF_REGION_W - 1, SF_BMP_Y, 1, SF_BMP_H, green);

    tft->drawFillRect(SF_FLAPS_X, SF_BMP_Y, SF_REGION_W, 1, blue);
    tft->drawFillRect(SF_FLAPS_X, SF_BMP_Y + SF_BMP_H - 1, SF_REGION_W, 1, blue);
    tft->drawFillRect(SF_FLAPS_X, SF_BMP_Y, 1, SF_BMP_H, blue);
    tft->drawFillRect(SF_FLAPS_X + SF_REGION_W - 1, SF_BMP_Y, 1, SF_BMP_H, blue);

    uint16_t scx = SF_SLATS_X + SF_REGION_W / 2;
    uint16_t scy = SF_BMP_Y + SF_BMP_H / 2;
    tft->drawFillRect(scx - 3, scy - 1, 7, 2, green);
    tft->drawFillRect(scx - 1, scy - 3, 2, 7, green);

    uint16_t fcx = SF_FLAPS_X + SF_REGION_W / 2;
    uint16_t fcy = SF_BMP_Y + SF_BMP_H / 2;
    tft->drawFillRect(fcx - 3, fcy - 1, 7, 2, blue);
    tft->drawFillRect(fcx - 1, fcy - 3, 2, 7, blue);

    tft->drawFillRect(SF_FLAPS_X - 1, 0, 2, SF_BMP_H + SF_BMP_Y, yellow);
}

static void drawGearCalibrationOverlay(ST7789* tft) {
    uint16_t gw = tft->width();
    uint16_t gh = tft->height();
    uint16_t colW = gw / 3;
    uint16_t colCX = gw / 6;
    uint16_t white = 0xFFFF;

    for (int i = 0; i < 3; ++i) {
        uint16_t cx = colCX + i * colW;
        tft->drawFillRect(cx - colW / 2, 0, colW, 1, white);
        tft->drawFillRect(cx - colW / 2, gh - 1, colW, 1, white);
        tft->drawFillRect(cx - colW / 2, 0, 1, gh, white);
        tft->drawFillRect(cx + colW / 2 - 1, 0, 1, gh, white);
        tft->drawFillRect(cx - 3, gh / 2 - 1, 7, 2, white);
        tft->drawFillRect(cx - 1, gh / 2 - 3, 2, 7, white);
    }
}

static void calibrationCycleStep() {
#if CALIBRATION_CYCLE
    if (absolute_time_diff_us(calPhaseStart, get_absolute_time()) < CAL_PHASE_MS * 1000)
        return;
    calPhaseStart = get_absolute_time();
    calPhase = (calPhase + 1) % 3;

    switch (calPhase) {
        case 0:
            slatsRawValue = 0;
            flapsRawValue = 0;
            gearRawNose = 0;
            gearRawLeft = 0;
            gearRawRight = 0;
            break;
        case 1:
            slatsRawValue = 65535;
            flapsRawValue = 1;
            gearRawNose = 1;
            gearRawLeft = 1;
            gearRawRight = 1;
            break;
        case 2:
            slatsRawValue = 0;
            flapsRawValue = 65535;
            gearRawNose = 65535;
            gearRawLeft = 65535;
            gearRawRight = 65535;
            break;
    }
#endif
}
#endif

static void renderSlatsFlapsDisplay() {
    if (!flapsTft) return;
    csDeselectAll();
    flapsTft->drawFillRect(0, 0, flapsTft->width(), flapsTft->height(), 0x0000);

    uint16_t white = 0xFFFF;
    uint16_t slatsCX = SF_SLATS_X + SF_REGION_W / 2;
    uint16_t flapsCX = SF_FLAPS_X + SF_REGION_W / 2;
    uint16_t cy = SF_BMP_Y + SF_BMP_H / 2;

    if (slatsRawValue == 0)
        drawMS33558Centered(flapsTft, slatsCX, cy, "IN", white);
    else if (slatsRawValue == 65535)
        drawMS33558Centered(flapsTft, slatsCX, cy, "OUT", white);
    else
        flapsTft->drawStringScaled(slatsCX - 60, cy - 14, "/////", white, 4);

    if (flapsRawValue == 0)
        drawMS33558Centered(flapsTft, flapsCX, cy, "UP", white);
    else if (flapsRawValue == 65535)
        drawMS33558Centered(flapsTft, flapsCX, cy, "DN", white);
    else
        flapsTft->drawStringScaled(flapsCX - 60, cy - 14, "/////", white, 4);

#if CALIBRATION_MODE
    drawCalibrationOverlay(flapsTft);
#endif
}

static void renderGearDisplay() {
    if (!gearTft) return;
    csDeselectAll();
    gearTft->drawFillRect(0, 0, gearTft->width(), gearTft->height(), 0x0000);

    uint16_t gw = gearTft->width();
    uint16_t gh = gearTft->height();
    uint16_t colW = gw / 3;
    uint16_t white = 0xFFFF;
    uint16_t cy = gh / 2;

    auto renderCol = [&](uint16_t rawVal, uint16_t cx) {
        if (rawVal == 0)
            drawMS33558Centered(gearTft, cx, cy, "UP", white);
        else if (rawVal == 65535)
            drawMS33558Centered(gearTft, cx, cy, "DN", white);
        else
            gearTft->drawStringScaled(cx - 60, cy - 14, "/////", white, 4);
    };

    uint16_t colCX = gw / 6;
    renderCol(gearRawLeft,   colCX);
    renderCol(gearRawNose,   colCX + colW);
    renderCol(gearRawRight,  colCX + colW * 2);

#if CALIBRATION_MODE
    drawGearCalibrationOverlay(gearTft);
#endif
}

// Stepper homing
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

int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("F-4E Instrument Panel starting...\n");

    DcsBios::initHeartbeat(HEARTBEAT_LED);

    uint8_t boardAddress = 0xF;

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    switch (board.mode)
    {
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
    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched\n");

    // SPI1 init (shared for 3x ST7789)
    spi_init(spi1, 10 * 1000 * 1000);
    gpio_set_function(SPI1_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI1_SCK, GPIO_FUNC_SPI);

    // AVTR LED (raw GPIO, initialized early so callback can use it)
    gpio_init(LED_AVTR);
    gpio_set_dir(LED_AVTR, GPIO_OUT);
    gpio_put(LED_AVTR, 0);

    // Gauge backlight PWM pins (initialized early for console lighting callback)
    gpio_set_function(GAUGE_BACKLIGHT_1, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(GAUGE_BACKLIGHT_1);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(GAUGE_BACKLIGHT_1, 0);
    }
    gpio_set_function(GAUGE_BACKLIGHT_2, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(GAUGE_BACKLIGHT_2);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(GAUGE_BACKLIGHT_2, 0);
    }
    gpio_set_function(GAUGE_BACKLIGHT_3, GPIO_FUNC_PWM);
    {
        uint slice = pwm_gpio_to_slice_num(GAUGE_BACKLIGHT_3);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, 1.0f);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(GAUGE_BACKLIGHT_3, 0);
    }

    // Initialize ALL display control pins as outputs high BEFORE any init
    // Explicitly set SIO function to prevent SPI peripheral conflicts
    // Use 8mA drive strength on CS pins for noise immunity
    gpio_set_function(TFT1_CS, GPIO_FUNC_SIO); gpio_set_dir(TFT1_CS, GPIO_OUT); gpio_put(TFT1_CS, 1);
    gpio_set_drive_strength(TFT1_CS, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_function(TFT1_DC, GPIO_FUNC_SIO); gpio_set_dir(TFT1_DC, GPIO_OUT); gpio_put(TFT1_DC, 0);
    gpio_set_function(TFT1_RST, GPIO_FUNC_SIO); gpio_set_dir(TFT1_RST, GPIO_OUT); gpio_put(TFT1_RST, 1);
    // gpio_set_function(TFT2_CS, GPIO_FUNC_SIO); gpio_set_dir(TFT2_CS, GPIO_OUT); gpio_put(TFT2_CS, 1);
    // gpio_set_drive_strength(TFT2_CS, GPIO_DRIVE_STRENGTH_8MA);
    // gpio_set_function(TFT2_DC, GPIO_FUNC_SIO); gpio_set_dir(TFT2_DC, GPIO_OUT); gpio_put(TFT2_DC, 0);
    // gpio_set_function(TFT2_RST, GPIO_FUNC_SIO); gpio_set_dir(TFT2_RST, GPIO_OUT); gpio_put(TFT2_RST, 1);
    gpio_set_function(TFT3_CS, GPIO_FUNC_SIO); gpio_set_dir(TFT3_CS, GPIO_OUT); gpio_put(TFT3_CS, 1);
    gpio_set_drive_strength(TFT3_CS, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_function(TFT3_DC, GPIO_FUNC_SIO); gpio_set_dir(TFT3_DC, GPIO_OUT); gpio_put(TFT3_DC, 0);
    gpio_set_function(TFT3_RST, GPIO_FUNC_SIO); gpio_set_dir(TFT3_RST, GPIO_OUT); gpio_put(TFT3_RST, 1);
    gpio_set_function(TFT_BL, GPIO_FUNC_SIO); gpio_set_dir(TFT_BL, GPIO_OUT); gpio_put(TFT_BL, 1);

    // Display 1 - Combined Slats+Flaps (landscape, rotation=3)
    csDeselectAll(); sleep_ms(10);
    gpio_put(TFT1_RST, 0); sleep_ms(10); gpio_put(TFT1_RST, 1); sleep_ms(120);

    flapsTft = new ST7789(spi1, TFT1_CS, TFT1_DC, TFT1_RST, TFT_BL);
    if (flapsTft) {
        flapsTft->init(4 * 1000 * 1000);
        flapsTft->setRotation(3);
        flapsTft->fillScreen(0x0000);
        flapsTft->setBacklight(128);
    }
    csDeselectAll(); sleep_ms(10);
    printf("Combined Slats+Flaps display initialized\n");

    // Display 2 - Slats (removed - combined with flaps on TFT1)
    // csDeselectAll(); sleep_ms(10);
    // gpio_put(TFT2_RST, 0); sleep_ms(10); gpio_put(TFT2_RST, 1); sleep_ms(120);
    // slatsTft = new ST7789(spi1, TFT2_CS, TFT2_DC, TFT2_RST, TFT_BL);
    // if (slatsTft) {
    //     slatsTft->init(4 * 1000 * 1000);
    //     slatsTft->setRotation(0);
    //     slatsTft->fillScreen(0x0000);
    //     slatsTft->setBacklight(128);
    // }
    // csDeselectAll(); sleep_ms(10);
    // printf("Slats display initialized\n");

    // Display 3 - Gear (landscape, rotation=3)
    csDeselectAll(); sleep_ms(10);
    gpio_put(TFT3_RST, 0); sleep_ms(10); gpio_put(TFT3_RST, 1); sleep_ms(120);

    gearTft = new ST7789(spi1, TFT3_CS, TFT3_DC, TFT3_RST, TFT_BL);
    if (gearTft) {
        gearTft->init(4 * 1000 * 1000);
        gearTft->setRotation(3);
        gearTft->setInversion(false);
        gearTft->fillScreen(0x0000);
        gearTft->setBacklight(128);
    }
    csDeselectAll(); sleep_ms(10);
    printf("Gear display initialized\n");

    // Bump SPI speed to 10MHz for pixel operations
    spi_set_baudrate(spi1, 10 * 1000 * 1000);

    // WS2812 console lighting
    consoleLeds.begin(NUM_CONSOLE_LEDS);
    consoleLeds.clear();
    consoleLeds.show();
    printf("WS2812 console lighting initialized\n");

    // Stepper A (Boost Pump L)
    const byj_gpio_config_t stepperACfg = {
        STEPPER_A_IN1, STEPPER_A_IN2, STEPPER_A_IN3, STEPPER_A_IN4
    };
    byj_init_gpio(&stepperA, &stepperACfg, BYJ_MODE_FULL_DOUBLE);
    byj_set_output_mode(&stepperA, BYJ_OUTPUT_BIPOLAR);
    byj_set_speed(&stepperA, 3000);

    // Stepper B (Boost Pump R)
    const byj_gpio_config_t stepperBCfg = {
        STEPPER_B_IN1, STEPPER_B_IN2, STEPPER_B_IN3, STEPPER_B_IN4
    };
    byj_init_gpio(&stepperB, &stepperBCfg, BYJ_MODE_FULL_DOUBLE);
    byj_set_output_mode(&stepperB, BYJ_OUTPUT_BIPOLAR);
    byj_set_speed(&stepperB, 3000);

    // Home both steppers (CCW toward limit switch = zero position)
    homeStepper(&stepperA, STEPPER_A_LIMIT, "Stepper A (Boost Pump L)");
    homeStepper(&stepperB, STEPPER_B_LIMIT, "Stepper B (Boost Pump R)");
    printf("Steppers homed\n");

    // Servo (Stabilator position gauge)
    DcsBios::ServoOutput stabilatorServo(F_4E_PLT_CONTROLS_STAB_TRIM_A, SERVO_PIN);

    // DCS-BIOS setup
    DcsBios::setup();
    printf("DCS-BIOS setup complete!\n");
    lastDisplayUpdate = get_absolute_time();
#if CALIBRATION_CYCLE
    calPhaseStart = get_absolute_time();
#endif

    // Initial display render
    renderSlatsFlapsDisplay();
    renderGearDisplay();

    printf("F-4E Instrument Panel ready\n");

    while (true) {
        DcsBios::loop();
        DcsBios::updateHeartbeat();

#if CALIBRATION_CYCLE
        calibrationCycleStep();
#endif

        bool sfChanged = (slatsRawValue != lastSlatsRendered) ||
                         (flapsRawValue != lastFlapsRendered);
        bool gearChanged = (gearRawNose != lastGearNoseRendered) ||
                           (gearRawLeft != lastGearLeftRendered) ||
                           (gearRawRight != lastGearRightRendered);

        if (absolute_time_diff_us(lastDisplayUpdate, get_absolute_time()) > 33000) {
            if (sfChanged) {
                renderSlatsFlapsDisplay();
                lastSlatsRendered = slatsRawValue;
                lastFlapsRendered = flapsRawValue;
            }
            if (gearChanged) {
                renderGearDisplay();
                lastGearNoseRendered = gearRawNose;
                lastGearLeftRendered = gearRawLeft;
                lastGearRightRendered = gearRawRight;
            }
            lastDisplayUpdate = get_absolute_time();
        }

        byj_update(&stepperA);
        byj_update(&stepperB);

        sleep_us(10);
    }
}
