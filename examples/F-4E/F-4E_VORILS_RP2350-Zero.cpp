#ifndef PICO_BOARD
#define PICO_BOARD
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
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "internal/ST7789.h"
#define USE_EPAPER 0
#if USE_EPAPER
#include "internal/EPaper213.h"
#endif

// ST7789 wiring (user): CS=GPIO13, DC=GPIO12, RST=GPIO11, BLK(backlight)=GPIO10, SCK=GPIO14, MOSI=SDA=GPIO15
static const uint TFT_CS_PIN = 13;
static const uint TFT_DC_PIN = 12;
static const uint TFT_RST_PIN = 11;
static const uint TFT_BLK_PIN = 10;
static const uint TFT_SCK_PIN = 14; // SPI SCK (spi1)
static const uint TFT_MOSI_PIN = 15; // SPI MOSI (spi1)


#define NUM_LEDS 6           // Total number of SK6812 LEDs

WS2812 externalLeds(pio0, 0, 9, false); // Global WS2812 object for external NeoPixels on pin 9
uart_inst_t *rs485_uart = uart0;

#if USE_EPAPER
// WeAct 2.13" e-paper pins on SPI1 (SCK=14, MOSI=15)
const DcsBios::Epaper213Pins vorIlsEpaperPins{.cs = 13, .dc = 12, .rst = 11, .busy = 10, .sck = 14, .mosi = 15};
DcsBios::Epaper213 vorIlsDisplay(vorIlsEpaperPins, spi1);
char pendingFrequency[7] = {0};
volatile bool freqDirty = false;
absolute_time_t lastDisplayUpdate;

// What is currently shown on the ST7789 (for per-digit updates)
char displayedFrequency[7] = {0};
bool epaperInProgress = false;
#endif
char lastFrequency[7] = {0};

// ST7789 display instance and deferred update buffer
static ST7789* vorIlsTft = nullptr;
char pendingFrequency[7] = {0};
volatile bool freqDirty = false;
absolute_time_t lastDisplayUpdate;

void onPltVorIlsFrequencyChange(char *newValue)
{
    if (strcmp(newValue, lastFrequency) == 0) return;

    strncpy(lastFrequency, newValue, sizeof(lastFrequency) - 1);
    lastFrequency[sizeof(lastFrequency) - 1] = '\0';
    char sanitized[7] = {0};
    size_t idx = 0;
    for (size_t i = 0; newValue[i] != '\0' && idx < 6; ++i) {
        if ((newValue[i] >= '0' && newValue[i] <= '9') || newValue[i] == '.') {
            sanitized[idx++] = newValue[i];
        }
    }
    memcpy(pendingFrequency, sanitized, sizeof(pendingFrequency));
    freqDirty = true; // defer display to main loop to avoid blocking serial
}
DcsBios::StringBuffer<6> pltVorIlsFrequencyBuffer(F_4E_PLT_VOR_ILS_FREQUENCY_A, onPltVorIlsFrequencyChange);

// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0)); // Red for F-4
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// Updated panel inputs for the new PCB revision
DcsBios::RotaryEncoderT<POLL_EVERY_TIME, DcsBios::TWO_STEPS_PER_DETENT, 5> pltVorIlsMarkerVolume("PLT_VOR_ILS_MARKER_VOLUME", "-1600", "+1600", 26, 27);
DcsBios::RotaryEncoderT<POLL_EVERY_TIME, DcsBios::TWO_STEPS_PER_DETENT, 5> pltVorIlsVolume("PLT_VOR_ILS_VOLUME", "-1600", "+1600", 28, 29);
// Emulated concentric encoder: outer = hundreds, inner = decimals.
// Uses encoder pins A=3, B=4 and toggle/button pin=5 (user switch on GPIO5, active low).
DcsBios::EmulatedConcentricRotaryEncoder1Step pltVorIlsFrequencyConcentric(
    "PLT_VOR_ILS_FREQUENCY_HUNDREDS", "DEC", "INC",
    "PLT_VOR_ILS_FREQUENCY_DECIMALS", "DEC", "INC",
    3, 4, 5);

// DCS-BIOS callback function for F-14 console lighting (red)
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t brightness = 0;
    
    if (consoleBrightness <= 8) {
        uint8_t percentages[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};
        brightness = (percentages[consoleBrightness] * 255) / 100;
    } else {
        brightness = (uint8_t)((consoleBrightness * 255) / 65535);
    }
    
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0)); // Red for F-14
    }
    externalLeds.show();
}

// Declare the IntegerBuffer for F-14 console lighting
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);



int main()
{
    stdio_init_all();                      // Initialize USB CDC
    // Startup diagnostic: print on USB-CDC for a few seconds so we can
    // confirm enumeration and detect early crashes before main init.
    for (int i = 0; i < 5; ++i) {
        printf("boot %d\r\n", i);
        sleep_ms(1000);
    }
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(200);                        // short pause after heartbeat init

    externalLeds.begin(NUM_LEDS);

    // Power-on Green Flash for 1 second
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Set all pixels to green (R=0, G=255, B=0, W=0)
    }
    externalLeds.show();
    sleep_ms(5000);

    // Clear LEDs after the flash
    externalLeds.clear();
    externalLeds.show();

    uint8_t boardAddress = 0xF;

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    switch (board.mode)
    {
    case DcsBios::BoardModeType::HOST:
        break;
    case DcsBios::BoardModeType::SLAVE:
        break;
    case DcsBios::BoardModeType::USB_ONLY:
        break;
    case DcsBios::BoardModeType::RS485_TERMINAL:
        break;
    default:
        break;
    }
    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");


// Initialize SPI and basic ST7789 control pins (no driver integrated here)
// Use spi1 with user-chosen pins: SCK=TFT_SCK_PIN, MOSI=TFT_MOSI_PIN
spi_init(spi1, 10 * 1000 * 1000); // 10 MHz
gpio_set_function(TFT_MOSI_PIN, GPIO_FUNC_SPI);
gpio_set_function(TFT_SCK_PIN, GPIO_FUNC_SPI);

// Configure control pins
gpio_init(TFT_CS_PIN);
gpio_set_dir(TFT_CS_PIN, GPIO_OUT);
gpio_put(TFT_CS_PIN, 1); // CS idle high

gpio_init(TFT_DC_PIN);
gpio_set_dir(TFT_DC_PIN, GPIO_OUT);
gpio_put(TFT_DC_PIN, 0);

gpio_init(TFT_RST_PIN);
gpio_set_dir(TFT_RST_PIN, GPIO_OUT);

gpio_init(TFT_BLK_PIN);
gpio_set_dir(TFT_BLK_PIN, GPIO_OUT);
gpio_put(TFT_BLK_PIN, 1); // backlight on

// Hardware reset pulse
gpio_put(TFT_RST_PIN, 0);
sleep_ms(10);
gpio_put(TFT_RST_PIN, 1);
sleep_ms(120);
    // Instantiate and initialize the ST7789 driver
    vorIlsTft = new ST7789(spi1, TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN, TFT_BLK_PIN);
    if (vorIlsTft) {
        vorIlsTft->init(10 * 1000 * 1000);
        // Try rotation=3 (270°) if 90° produced upside-down text
        vorIlsTft->setRotation(3);
        vorIlsTft->fillScreen(0x0000); // initial clear to black
        // Reduce backlight to ~25% to lower brightness
        vorIlsTft->setBacklight(64);
        // initial clear to black
        vorIlsTft->fillScreen(0x0000);
    }
    DcsBios::setup(); // Initialize DCS-BIOS framework
    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
#if USE_EPAPER
        if (freqDirty && !epaperInProgress && !gpio_get(vorIlsEpaperPins.busy) && absolute_time_diff_us(lastDisplayUpdate, get_absolute_time()) > 500000) {
            if (vorIlsDisplay.beginDisplayText(pendingFrequency)) {
                epaperInProgress = true;
                freqDirty = false;
                lastDisplayUpdate = get_absolute_time();
            }
        }

        if (epaperInProgress) {
            if (vorIlsDisplay.process()) {
                epaperInProgress = false;
                lastDisplayUpdate = get_absolute_time();
            }
        }
#endif
        // Handle deferred ST7789 updates (non-blocking from DCS-BIOS callbacks)
        if (freqDirty && vorIlsTft && absolute_time_diff_us(lastDisplayUpdate, get_absolute_time()) > 200000) {
            // Draw scaled (x3) text for readability and center it, only clear the text area to avoid full-screen flicker
            if (vorIlsTft) {
                const uint8_t baseScale = 3;              // previous scale used
                const uint8_t scale = baseScale * 2;      // increase size by a factor of 2
                size_t len = strlen(pendingFrequency);
                if (len > 0) {
                    // Use MS33558 numeric font at 48px height. Reserve conservative area for clearing.
                    const uint16_t fontH = 48;
                    const uint16_t glyphSpacing = 2; // pixels between glyphs
                    uint16_t textW = (uint16_t)(len * (fontH + glyphSpacing));
                    uint16_t textH = fontH;
                    uint16_t x = 0;
                    uint16_t y = 0;
                    if (vorIlsTft->width() > textW) x = (vorIlsTft->width() - textW) / 2;
                    if (vorIlsTft->height() > textH) y = (vorIlsTft->height() - textH) / 2;
                    // shift left two numeric character widths and down half a numeric character height
                    int32_t tx = (int32_t)x;
                    int32_t ty = (int32_t)y;
                    tx -= 2 * (fontH + glyphSpacing);
                    ty += (fontH / 2);
                    // also keep previous baseline shift for visual alignment
                    const uint8_t baseScale = 3;
                    uint16_t priorCharHeight = (uint16_t)(7 * baseScale);
                    if (vorIlsTft->height() > (uint32_t)ty + priorCharHeight) ty += priorCharHeight;
                    if (tx < 0) tx = 0;
                    x = (uint16_t)tx;
                    y = (uint16_t)ty;
                    // Partial refresh: clear only the bounding box under the digits
                    uint16_t tW = vorIlsTft->number48Width(pendingFrequency);
                    uint16_t tH = vorIlsTft->number48Height();
                    int32_t bx = (vorIlsTft->width() > tW) ? (vorIlsTft->width() - tW) / 2 : 0;
                    // Start centered vertically, then shift down by one full glyph height
                    int32_t by_center = (vorIlsTft->height() > tH) ? (vorIlsTft->height() - tH) / 2 : 0;
                    int32_t by = by_center + (int32_t)tH;
                    // Clamp so we don't go off-screen
                    if (by < 0) by = 0;
                    if ((uint32_t)by + tH > vorIlsTft->height()) {
                        by = (int32_t)vorIlsTft->height() - (int32_t)tH;
                        if (by < 0) by = 0;
                    }
                    const int pad = 2;
                    int32_t clear_x = bx > pad ? bx - pad : 0;
                    int32_t clear_y = by > pad ? by - pad : 0;
                    uint16_t clear_w = (uint16_t) (tW + pad * 2);
                    uint16_t clear_h = (uint16_t) (tH + pad * 2);
                    vorIlsTft->clearRect((uint16_t)clear_x, (uint16_t)clear_y, clear_w, clear_h, 0x0000);
                    vorIlsTft->drawNumber48_new((uint16_t)bx, (uint16_t)by, pendingFrequency, 0xFFFF);
                }
            }
            freqDirty = false;
            lastDisplayUpdate = get_absolute_time();
        }
        sleep_us(10);
    }
}