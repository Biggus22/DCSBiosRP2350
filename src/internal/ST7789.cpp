#include "ST7789.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "pico/time.h"
#include <string.h>

#pragma message("ST7789.cpp re-saved to force rebuild")

// If the MS33558 48px numeric font header exists, include it at file scope.
#if defined(__has_include)
#  if __has_include("MS33558_48.h")
#    include "MS33558_48.h"
#    define HAS_MS33558_48 1
#  else
#    define HAS_MS33558_48 0
#  endif
#else
#  define HAS_MS33558_48 0
#endif

// Basic 5x7 font (ASCII 32..127), each char 5 bytes
const uint8_t ST7789::font5x7[] = {
#include "font5x7.inc"
};

// Panel dimensions and offsets for 172x320 variant
#define TFT_WIDTH 172
#define TFT_HEIGHT 320
// These offsets may need tweaking for your particular panel. Try 0 first; some panels use 34.
#define TFT_X_OFFSET 0
#define TFT_Y_OFFSET 0
// Workaround: some 172x320 modules have a few visible physical rows
// that are outside the nominal 320 address range. Add a small
// bottom padding to full-screen clears so the black background
// fully covers the visible area. Adjust if necessary.
#define TFT_BOTTOM_PAD 0

ST7789::ST7789(spi_inst_t* spi, uint cs_pin, uint dc_pin, uint rst_pin, uint bl_pin)
    : spi_(spi), cs_(cs_pin), dc_(dc_pin), rst_(rst_pin), bl_(bl_pin) {
    width_ = TFT_WIDTH;
    height_ = TFT_HEIGHT;
    rotation_ = 0;
    // Ensure control pins are configured in driver as a fallback
    gpio_init(cs_);
    gpio_set_dir(cs_, GPIO_OUT);
    gpio_put(cs_, 1);

    gpio_init(dc_);
    gpio_set_dir(dc_, GPIO_OUT);

    gpio_init(rst_);
    gpio_set_dir(rst_, GPIO_OUT);

    gpio_init(bl_);
    // Default backlight on as GPIO high; `setBacklight` will switch to PWM when called.
    gpio_set_dir(bl_, GPIO_OUT);
    gpio_put(bl_, 1);
}

void ST7789::sendCommand(uint8_t cmd) {
    gpio_put(dc_, 0);
    sleep_us(2);
    gpio_put(cs_, 0);
    sleep_us(2);
    spi_write_blocking(spi_, &cmd, 1);
    sleep_us(2);
    gpio_put(cs_, 1);
    sleep_us(2);
}

void ST7789::sendData(const uint8_t* data, size_t len) {
    gpio_put(dc_, 1);
    sleep_us(2);
    gpio_put(cs_, 0);
    sleep_us(2);
    spi_write_blocking(spi_, data, len);
    sleep_us(2);
    gpio_put(cs_, 1);
    sleep_us(2);
}

void ST7789::sendData8(uint8_t d) {
    sendData(&d, 1);
}

void ST7789::init(uint32_t baudrate) {
    // Configure SPI data format (8 bits, mode 0, MSB first)
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_baudrate(spi_, baudrate);

    // Hardware reset
    gpio_put(rst_, 0);
    sleep_ms(10);
    gpio_put(rst_, 1);
    sleep_ms(120);

    sendCommand(0x01); // SWRESET
    sleep_ms(150);

    sendCommand(0x11); // SLPOUT
    sleep_ms(120);

    // Set color mode to 16-bit (RGB565)
    sendCommand(0x3A); // COLMOD
    uint8_t colmod = 0x55; // 16-bit/pixel
    sendData(&colmod, 1);
    sleep_ms(10);

    // Memory Access Control (orientation + BGR)
    sendCommand(0x36);
    // Use BGR order by default for many displays. Rotation bits adjusted in setRotation().
    uint8_t madctl = 0x08; // BGR
    sendData(&madctl, 1);

    // Column/Row address set will use offsets in setAddrWindow

    // Inversion ON (try if panel needs it)
    sendCommand(0x21); // INVON

    // Normal display mode
    sendCommand(0x13); // NORON
    sleep_ms(10);

    sendCommand(0x29); // DISPON
    sleep_ms(120);
}

void ST7789::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];
    sendCommand(0x2A); // CASET
    // Apply panel offsets. For 90/270 rotations the X/Y offsets need to be swapped
    uint16_t xs, xe, ys, ye;
    if ((rotation_ & 1) == 1) {
        // rotation 1 or 3: swap offsets
        xs = x0 + TFT_Y_OFFSET;
        xe = x1 + TFT_Y_OFFSET;
    } else {
        xs = x0 + TFT_X_OFFSET;
        xe = x1 + TFT_X_OFFSET;
    }
    data[0] = (xs >> 8) & 0xFF;
    data[1] = xs & 0xFF;
    data[2] = (xe >> 8) & 0xFF;
    data[3] = xe & 0xFF;
    sendData(data, 4);

    sendCommand(0x2B); // RASET
    if ((rotation_ & 1) == 1) {
        ys = y0 + TFT_X_OFFSET;
        ye = y1 + TFT_X_OFFSET;
    } else {
        ys = y0 + TFT_Y_OFFSET;
        ye = y1 + TFT_Y_OFFSET;
    }
    data[0] = (ys >> 8) & 0xFF;
    data[1] = ys & 0xFF;
    data[2] = (ye >> 8) & 0xFF;
    data[3] = ye & 0xFF;
    sendData(data, 4);

    sendCommand(0x2C); // RAMWR
}

void ST7789::fillScreen(uint16_t color) {
    // Extend the vertical address to include a small bottom padding
    uint16_t end_y = height_ - 1;
#if defined(TFT_BOTTOM_PAD)
    // Guard against overflow
    if ((uint32_t)height_ + (uint32_t)TFT_BOTTOM_PAD - 1 > 0xFFFFu) {
        end_y = 0xFFFF;
    } else {
        end_y = (uint16_t)(height_ - 1 + TFT_BOTTOM_PAD);
    }
#endif
    setAddrWindow(0, 0, width_ - 1, end_y);

    uint8_t buf[512];
    for (size_t i = 0; i < sizeof(buf); i += 2) {
        buf[i] = (color >> 8) & 0xFF;
        buf[i+1] = color & 0xFF;
    }

    gpio_put(dc_, 1);
    sleep_us(2);
    gpio_put(cs_, 0);
    sleep_us(2);
    size_t total = (size_t)width_ * (size_t)(height_ + TFT_BOTTOM_PAD);
    while (total) {
        size_t toWrite = sizeof(buf) / 2; // number of pixels
        if (toWrite > total) toWrite = total;
        spi_write_blocking(spi_, buf, toWrite * 2);
        total -= toWrite;
    }
    sleep_us(2);
    gpio_put(cs_, 1);
    sleep_us(2);
}

void ST7789::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    setAddrWindow(x, y, x, y);
    uint8_t d[2] = { (uint8_t)(color >> 8), (uint8_t)color };
    sendData(d, 2);
}

void ST7789::drawChar(uint16_t x, uint16_t y, char c, uint16_t color) {
    if (c < 32 || c > 126) c = '?';
    const uint8_t* glyph = &font5x7[(c - 32) * 5];
    for (int col = 0; col < 5; ++col) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; ++row) {
            if (bits & (1 << row)) {
                drawPixel(x + col, y + row, color);
            }
        }
    }
}

void ST7789::drawFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (w == 0 || h == 0) return;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    uint8_t buf[64];
    for (size_t i = 0; i < sizeof(buf); i += 2) {
        buf[i] = (color >> 8) & 0xFF;
        buf[i+1] = color & 0xFF;
    }
    gpio_put(dc_, 1);
    sleep_us(2);
    gpio_put(cs_, 0);
    sleep_us(2);
    size_t total = (size_t)w * h;
    while (total) {
        size_t pixels = sizeof(buf) / 2;
        if (pixels > total) pixels = total;
        spi_write_blocking(spi_, buf, pixels * 2);
        total -= pixels;
    }
    sleep_us(2);
    gpio_put(cs_, 1);
    sleep_us(2);
}

void ST7789::drawScrollingBitmap(const uint8_t* bitmap, uint16_t bmpH, uint16_t scrollOffset) {
    uint16_t dw = width_;
    uint16_t dh = height_;
    if (dw == 0 || dh == 0) return;

    setAddrWindow(0, 0, dw - 1, dh - 1);
    gpio_put(dc_, 1);
    sleep_us(2);
    gpio_put(cs_, 0);
    sleep_us(2);

    uint8_t blackBuf[128];
    memset(blackBuf, 0, sizeof(blackBuf));

    for (uint16_t row = 0; row < dh; row++) {
        uint16_t bmpRow = scrollOffset + row;
        if (bmpRow < bmpH) {
            const uint8_t* rowData = bitmap + (uint32_t)bmpRow * dw * 2;
            spi_write_blocking(spi_, rowData, dw * 2);
        } else {
            size_t remaining = dw;
            while (remaining) {
                size_t chunk = (remaining < sizeof(blackBuf) / 2) ? remaining : (sizeof(blackBuf) / 2);
                spi_write_blocking(spi_, blackBuf, chunk * 2);
                remaining -= chunk;
            }
        }
    }
    sleep_us(2);
    gpio_put(cs_, 1);
    sleep_us(2);
}

void ST7789::drawScrollingBitmapRegion(uint16_t dstX, uint16_t dstY,
                                       uint16_t dstW, uint16_t dstH,
                                       const uint8_t* bitmap, uint16_t bmpW, uint16_t bmpH,
                                       uint16_t scrollOffset, uint16_t dispH) {
    if (dstW == 0 || dstH == 0 || bmpW == 0 || bmpH == 0) return;

    if (dispH == 0) dispH = bmpH; // legacy: no vertical scaling

    // Set a single address window covering all output rows
    setAddrWindow(dstX, dstY, dstX + dstW - 1, dstY + dstH - 1);
    gpio_put(dc_, 1);
    sleep_us(2);
    gpio_put(cs_, 0);
    sleep_us(2);

    // Precompute nearest-neighbour source column for each destination column
    uint16_t srcCol[320];
    for (uint16_t col = 0; col < dstW; ++col) {
        srcCol[col] = (col * bmpW) / dstW;
    }

    uint8_t rowBuf[640]; // max row = 320 px * 2 bytes
    // When MY=1 (rotation >= 2), the display writes bottom-up within the
    // address window, so we reverse row order to keep row 0 at the top.
    bool reverseRows = (rotation_ >= 2);
    for (uint16_t ri = 0; ri < dstH; ++ri) {
        uint16_t row = reverseRows ? (dstH - 1 - ri) : ri;
        // Map display row to source row via the full displayed height
        uint32_t fullRow = (uint32_t)scrollOffset + row;
        uint16_t bmpRow = (uint16_t)((fullRow * bmpH) / dispH);
        if (fullRow < dispH && bmpRow < bmpH) {
            const uint8_t* srcRow = bitmap + (uint32_t)bmpRow * bmpW * 2;
            for (uint16_t col = 0; col < dstW; ++col) {
                uint16_t sc = srcCol[col];
                rowBuf[col * 2]     = srcRow[sc * 2];
                rowBuf[col * 2 + 1] = srcRow[sc * 2 + 1];
            }
        } else {
            memset(rowBuf, 0, dstW * 2);
        }
        spi_write_blocking(spi_, rowBuf, dstW * 2);
    }
    sleep_us(2);
    gpio_put(cs_, 1);
    sleep_us(2);
}

void ST7789::drawStringScaled(uint16_t x, uint16_t y, const char* s, uint16_t color, uint8_t scale) {
    uint16_t cursor = x;
    while (*s) {
        char c = *s++;
        if (c < 32 || c > 126) c = '?';
        const uint8_t* glyph = &font5x7[(c - 32) * 5];
        for (int col = 0; col < 5; ++col) {
            uint8_t bits = glyph[col];
            for (int row = 0; row < 7; ++row) {
                if (bits & (1 << row)) {
                    uint16_t px = cursor + col * scale;
                    uint16_t py = y + row * scale;
                    drawFillRect(px, py, scale, scale, color);
                }
            }
        }
        cursor += (5 + 1) * scale; // advance by character width + spacing
    }
}

void ST7789::setRotation(uint8_t m) {
    // m = 0..3
    uint8_t madctl = 0;
    switch (m & 3) {
        case 0: madctl = 0x08; break; // default + BGR
        case 1: madctl = 0x68; break; // rotate 90 + BGR
        case 2: madctl = 0xC8; break; // rotate 180 + BGR
        case 3: madctl = 0xE8; break; // rotate 270 + BGR + horizontal flip
    }
    sendCommand(0x36);
    sendData(&madctl, 1);
    rotation_ = m & 3;
    // swap width/height for 90/270 degree rotations
    if ((rotation_ & 1) == 1) {
        width_ = TFT_HEIGHT;
        height_ = TFT_WIDTH;
    } else {
        width_ = TFT_WIDTH;
        height_ = TFT_HEIGHT;
    }
}

void ST7789::setInversion(bool on) {
    sendCommand(on ? 0x21 : 0x20);
}

uint16_t ST7789::width() { return width_; }
uint16_t ST7789::height() { return height_; }

void ST7789::setBacklight(uint8_t level) {
    // Configure BL pin for PWM and set duty (0..255)
    gpio_set_function(bl_, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(bl_);
    pwm_set_enabled(slice, false);
    pwm_set_wrap(slice, 255);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(bl_), level);
    pwm_set_enabled(slice, true);
}

void ST7789::clearRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    drawFillRect(x, y, w, h, color);
}

void ST7789::drawString(uint16_t x, uint16_t y, const char* s, uint16_t color) {
    uint16_t cursor = x;
    while (*s) {
        drawChar(cursor, y, *s, color);
        cursor += 6; // 5px +1 spacing
        ++s;
    }

}

// `drawNumber48` implementation moved to src/internal/ST7789_numbers.cpp

uint16_t ST7789::number48Width(const char* s) {
#if HAS_MS33558_48
    uint16_t cell_w = 0;
    for (int i = 0; i < MS33558_48_glyph_count; ++i) {
        if (MS33558_48_glyph_widths[i] > cell_w) cell_w = MS33558_48_glyph_widths[i];
    }
    const uint16_t spacing = 2;
    uint32_t len = 0;
    for (const char* p = s; *p; ++p) ++len;
    return (uint16_t)(len * (cell_w + spacing));
#else
    const uint8_t scale = 6;
    uint32_t len = 0;
    for (const char* p = s; *p; ++p) ++len;
    return (uint16_t)(len * (5 + 1) * scale);
#endif
}

uint16_t ST7789::number48Height() {
#if HAS_MS33558_48
    return MS33558_48_height;
#else
    return (uint16_t)(7 * 6);
#endif
}

uint16_t ST7789::number48CellWidth() {
#if HAS_MS33558_48
    uint16_t cell_w = 0;
    for (int i = 0; i < MS33558_48_glyph_count; ++i) {
        if (MS33558_48_glyph_widths[i] > cell_w) cell_w = MS33558_48_glyph_widths[i];
    }
    return cell_w;
#else
    return 5 * 6;
#endif
}

uint16_t ST7789::number48Spacing() {
#if HAS_MS33558_48
    return 2;
#else
    return 1 * 6;
#endif
}
