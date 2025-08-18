// src/internal/ws2812.cpp
#include "ws2812.h"
#include "ws2812.pio.h"
#include "hardware/clocks.h"
#include <cstring>


WS2812::WS2812(PIO pio, uint sm, uint pin, bool isRgbw)
    : _pio(pio), _sm(sm), _pin(pin), _isRgbw(isRgbw), _offset(0), _numPixels(0), _pixelBuffer(nullptr), _brightness(255) {}

    bool WS2812::begin(uint16_t numPixels) {
        _numPixels = numPixels;

        if (_pixelBuffer) delete[] _pixelBuffer;
        _pixelBuffer = new uint32_t[_numPixels];
        std::memset(_pixelBuffer, 0, sizeof(uint32_t) * _numPixels);

        // --- NEW: claim a free SM + program compatible with the pin ---
        if (!pio_can_add_program(_pio, &ws2812_program)) return false;
        bool ok = pio_claim_free_sm_and_add_program_for_gpio_range(
            &ws2812_program, &_pio, &_sm, &_offset, _pin, 1, true  // <-- key!
        );
        if (!ok) return false;

        // --- This is now safe ---
        ws2812_program_init(_pio, _sm, _offset, _pin, 800000, _isRgbw); // Keep at 800kHz, _isRgbw comes from constructor
        return true;
    }

void WS2812::setPixel(uint16_t index, uint32_t grb) {
    if (index >= _numPixels) return;
    _pixelBuffer[index] = grb;
}

void WS2812::setBrightness(uint8_t brightness) {
    _brightness = brightness;
}

void WS2812::clear() {
    if (_pixelBuffer) std::memset(_pixelBuffer, 0, sizeof(uint32_t) * _numPixels);
}

void WS2812::show() {
    for (uint16_t i = 0; i < _numPixels; ++i) {
        uint32_t color = _pixelBuffer[i];
        // The show() function's extraction order (GRWB) is fixed by the PIO program definition.
        // It reads data from the 32-bit word in this order: Green (MSB), Red, White, Blue (LSB).
        uint8_t g_orig = (color >> 24) & 0xFF; // Green component
        uint8_t r_orig = (color >> 16) & 0xFF; // Red component
        uint8_t w_orig = (color >> 8) & 0xFF;  // White component
        uint8_t b_orig = (color >> 0) & 0xFF;  // Blue component

        // Apply global brightness scaling
        r_orig = (r_orig * _brightness) / 255;
        g_orig = (g_orig * _brightness) / 255;
        b_orig = (b_orig * _brightness) / 255;
        w_orig = (w_orig * _brightness) / 255; // Apply brightness to white component

        // Reconstruct the color with brightness applied for PIO output (in GRWB order)
        uint32_t scaledColor = ((uint32_t)g_orig << 24) | ((uint32_t)r_orig << 16) | ((uint32_t)w_orig << 8) | ((uint32_t)b_orig << 0);
        pio_sm_put_blocking(_pio, _sm, scaledColor);
    }
}

uint32_t WS2812::rgb(uint8_t r, uint8_t g, uint8_t b) {
    // Corrected R/G swap: User's Red (r) goes to PIO's Green slot, User's Green (g) goes to PIO's Red slot.
    return ((uint32_t)r << 24) | // User Red -> PIO Green
           ((uint32_t)g << 16) | // User Green -> PIO Red
           ((uint32_t)0 << 8)  | // White (0 for RGB)
           ((uint32_t)b << 0);   // User Blue -> PIO Blue
}

uint32_t WS2812::rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    // Corrected R/G swap: User's Red (r) goes to PIO's Green slot, User's Green (g) goes to PIO's Red slot.
    // White (w) will effectively be ignored by PIO since _isRgbw is false.
    return ((uint32_t)r << 24) | // User Red -> PIO Green
           ((uint32_t)g << 16) | // User Green -> PIO Red
           ((uint32_t)w << 8)  | // User White -> PIO White
           ((uint32_t)b << 0);   // User Blue -> PIO Blue
}

WS2812::~WS2812() {
    if (_pixelBuffer) delete[] _pixelBuffer;
}