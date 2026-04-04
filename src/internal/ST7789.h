#ifndef __ST7789_H__
#define __ST7789_H__

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdint.h>

#pragma message("Including ST7789 header: " __FILE__)

class ST7789 {
public:
    ST7789(spi_inst_t* spi, uint cs_pin, uint dc_pin, uint rst_pin, uint bl_pin);
    void init(uint32_t baudrate = 10000000);
    void fillScreen(uint16_t color);
    void drawPixel(uint16_t x, uint16_t y, uint16_t color);
    void drawChar(uint16_t x, uint16_t y, char c, uint16_t color);
    void drawStringScaled(uint16_t x, uint16_t y, const char* s, uint16_t color, uint8_t scale);
    void setRotation(uint8_t m);
    void drawString(uint16_t x, uint16_t y, const char* s, uint16_t color);
    // Set backlight brightness (0..255). Uses PWM on the BL pin.
    void setBacklight(uint8_t level);
    // Backwards-compatible numeric draw (for older call sites)
    void drawNumber48(uint16_t x, uint16_t y, const char* s, uint16_t color);
    // Draw numeric string using MS33558 48px numeric font if available
    void drawNumber48_new(uint16_t x, uint16_t y, const char* s, uint16_t color);
    // Draw numeric string right-justified so the right edge is at `right_x`
    void drawNumber48Right(uint16_t right_x, uint16_t y, const char* s, uint16_t color);
    // Helpers to compute rendered numeric string dimensions (for partial clears)
    uint16_t number48Width(const char* s);
    uint16_t number48Height();
    // Per-digit metrics for partial updates
    uint16_t number48CellWidth();
    uint16_t number48Spacing();
    // Clear a rectangular region (public helper)
    void drawFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void clearRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    uint16_t width();
    uint16_t height();

private:
    spi_inst_t* spi_;
    uint cs_;
    uint dc_;
    uint rst_;
    uint bl_;
    uint16_t width_;
    uint16_t height_;
    uint8_t rotation_;

    void sendCommand(uint8_t cmd);
    void sendData(const uint8_t* data, size_t len);
    void sendData8(uint8_t d);
    void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    static const uint8_t font5x7[];
};

#endif // __ST7789_H__
