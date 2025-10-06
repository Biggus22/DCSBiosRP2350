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
        ws2812_program_init(_pio, _sm, _offset, _pin, 800000, _isRgbw);
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
        uint8_t r = (color >> 8) & 0xFF;
        uint8_t g = (color >> 16) & 0xFF;
        uint8_t b = color & 0xFF;

        // Apply global brightness scaling (stacked with any per-LED brightness)
        r = (r * _brightness) / 255;
        g = (g * _brightness) / 255;
        b = (b * _brightness) / 255;

        uint32_t scaledColor = ((uint32_t)r << 8) | ((uint32_t)g << 16) | b;
        pio_sm_put_blocking(_pio, _sm, scaledColor << 8u);
    }
}

uint32_t WS2812::rgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 8) | ((uint32_t)g << 16) | (uint32_t)b;
}

uint32_t WS2812::rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    return ((uint32_t)r << 8) | ((uint32_t)g << 16) | ((uint32_t)w << 24) | (uint32_t)b;
}

WS2812::~WS2812() {
    if (_pixelBuffer) delete[] _pixelBuffer;
}
