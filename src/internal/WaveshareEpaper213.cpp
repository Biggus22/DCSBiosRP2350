#include "WaveshareEpaper213.h"

#include <algorithm>
#include <cstdlib>
#include <utility>

namespace DcsBios {

namespace {
constexpr uint8_t kDefaultLut[] = {
    0x80, 0x48, 0x40, 0x00, 0x00, 0x00,
    0x40, 0x48, 0x40, 0x00, 0x00, 0x00,
    0x80, 0x48, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const EpaperGlyph kBasicGlyphs[] = {
    {' ', 5, {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', 5, {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', 5, {0x00, 0x00, 0x60, 0x60, 0x00}},
    {':', 5, {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', 5, {0x3E, 0x45, 0x49, 0x51, 0x3E}},
    {'1', 5, {0x00, 0x21, 0x7F, 0x01, 0x00}},
    {'2', 5, {0x21, 0x43, 0x45, 0x49, 0x31}},
    {'3', 5, {0x42, 0x41, 0x51, 0x69, 0x46}},
    {'4', 5, {0x0C, 0x14, 0x24, 0x7F, 0x04}},
    {'5', 5, {0x72, 0x51, 0x51, 0x51, 0x4E}},
    {'6', 5, {0x3E, 0x49, 0x49, 0x49, 0x26}},
    {'7', 5, {0x40, 0x47, 0x48, 0x50, 0x60}},
    {'8', 5, {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', 5, {0x32, 0x49, 0x49, 0x49, 0x3E}},
    {'A', 5, {0x7E, 0x09, 0x09, 0x09, 0x7E}},
    {'B', 5, {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', 5, {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', 5, {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', 5, {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', 5, {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', 5, {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', 5, {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', 5, {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', 5, {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', 5, {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', 5, {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', 5, {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', 5, {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', 5, {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', 5, {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', 5, {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', 5, {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', 5, {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', 5, {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', 5, {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', 5, {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', 5, {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', 5, {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', 5, {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', 5, {0x61, 0x51, 0x49, 0x45, 0x43}}
};
} // namespace

const EpaperFontDescriptor WaveshareEpaper213::BasicFont = {
    kBasicGlyphs,
    sizeof(kBasicGlyphs) / sizeof(kBasicGlyphs[0]),
    7,
    1,
    5
};

WaveshareEpaper213::WaveshareEpaper213(const WaveshareEpaperConnection& connection,
                                       WaveshareEpaperGeometry geometry)
    : connection_(connection),
      geometry_(geometry),
      lineBytes_(0),
      bufferSize_(0),
      frameBuffer_(nullptr),
      font_(&BasicFont),
      spiConfigured_(false),
      poweredOn_(false) {
    if (geometry_.width == 0) {
        geometry_.width = DefaultWidth;
    }
    if (geometry_.height == 0) {
        geometry_.height = DefaultHeight;
    }

    lineBytes_ = static_cast<uint16_t>((geometry_.width + 7) / 8);
    bufferSize_ = static_cast<size_t>(lineBytes_) * geometry_.height;
    frameBuffer_.reset(new uint8_t[bufferSize_]);
    fillBuffer(0xFF);
}

void WaveshareEpaper213::init(bool autoClear) {
    configureSpi();
    resetHardware();
    powerOn();
    if (autoClear) {
        clear(false);
        display(true);
    }
}

void WaveshareEpaper213::display(bool waitForIdle) {
    if (!poweredOn_) {
        powerOn();
    }
    sendCommand(0x10);
    sendData(frameBuffer_.get(), bufferSize_);
    sendCommand(0x12); // display refresh
    if (waitForIdle) {
        waitUntilIdle();
    }
}

void WaveshareEpaper213::clear(bool black) {
    fillBuffer(black ? 0x00 : 0xFF);
}

void WaveshareEpaper213::sleep() {
    sendCommand(0x02); // power off
    waitUntilIdle();
    sendCommand(0x07); // deep sleep
    sendData(0xA5);
    poweredOn_ = false;
}

void WaveshareEpaper213::setTextFont(const EpaperFontDescriptor* font) {
    font_ = (font != nullptr) ? font : &BasicFont;
}

void WaveshareEpaper213::drawPixel(uint16_t x, uint16_t y, bool black) {
    if (x >= geometry_.width || y >= geometry_.height) {
        return;
    }
    size_t index = static_cast<size_t>(y) * lineBytes_ + (x / 8);
    uint8_t bit = 0x80 >> (x & 0x07);
    if (black) {
        frameBuffer_[index] &= ~bit;
    } else {
        frameBuffer_[index] |= bit;
    }
}

void WaveshareEpaper213::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool black) {
    bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int16_t dx = x1 - x0;
    int16_t dy = std::abs(y1 - y0);
    int16_t error = dx / 2;
    int16_t ystep = (y0 < y1) ? 1 : -1;
    int16_t y = y0;

    for (int16_t x = x0; x <= x1; ++x) {
        if (steep) {
            drawPixel(y, x, black);
        } else {
            drawPixel(x, y, black);
        }
        error -= dy;
        if (error < 0) {
            y += ystep;
            error += dx;
        }
    }
}

void WaveshareEpaper213::drawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                   bool black) {
    if (width == 0 || height == 0) {
        return;
    }
    drawLine(x, y, x + width - 1, y, black);
    drawLine(x, y + height - 1, x + width - 1, y + height - 1, black);
    drawLine(x, y, x, y + height - 1, black);
    drawLine(x + width - 1, y, x + width - 1, y + height - 1, black);
}

void WaveshareEpaper213::fillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                   bool black) {
    if (width == 0 || height == 0) {
        return;
    }
    for (uint16_t row = 0; row < height; ++row) {
        for (uint16_t col = 0; col < width; ++col) {
            drawPixel(x + col, y + row, black);
        }
    }
}

void WaveshareEpaper213::drawBitmap(uint16_t x, uint16_t y, uint16_t bitmapWidth,
                                    uint16_t bitmapHeight, const uint8_t* bitmap,
                                    bool invert) {
    if (!bitmap) {
        return;
    }
    if (bitmapWidth == 0 || bitmapHeight == 0) {
        return;
    }
    uint16_t rowStride = (bitmapWidth + 7) / 8;
    for (uint16_t row = 0; row < bitmapHeight; ++row) {
        for (uint16_t col = 0; col < bitmapWidth; ++col) {
            size_t byteIndex = row * rowStride + (col / 8);
            bool bit = bitmap[byteIndex] & (0x80 >> (col & 0x07));
            if (invert) {
                bit = !bit;
            }
            drawPixel(x + col, y + row, bit);
        }
    }
}

void WaveshareEpaper213::drawText(uint16_t x, uint16_t y, const char* text, bool black,
                                  bool wrap) {
    if (!text) {
        return;
    }
    const EpaperFontDescriptor* font = font_ ? font_ : &BasicFont;
    uint16_t startX = x;
    while (*text) {
        if (*text == '\n') {
            x = startX;
            y += font->height + font->spacing;
            ++text;
            continue;
        }
        const EpaperGlyph* glyph = findGlyph(*text);
        if (glyph) {
            drawCharacter(x, y, *glyph, black, *font);
            x += glyph->width + font->spacing;
        }
        ++text;
        if (wrap && x + font->defaultWidth > geometry_.width) {
            x = startX;
            y += font->height + font->spacing;
        }
    }
}

void WaveshareEpaper213::drawCharacter(uint16_t x, uint16_t y, const EpaperGlyph& glyph,
                                       bool black, const EpaperFontDescriptor& font) {
    for (uint8_t col = 0; col < glyph.width; ++col) {
        uint8_t column = glyph.columns[col];
        for (uint8_t row = 0; row < font.height; ++row) {
            if (column & (1 << row)) {
                drawPixel(x + col, y + row, black);
            }
        }
    }
}

const EpaperGlyph* WaveshareEpaper213::findGlyph(char code) const {
    if (!font_ || !font_->glyphs) {
        return nullptr;
    }
    for (size_t idx = 0; idx < font_->glyphCount; ++idx) {
        if (font_->glyphs[idx].code == static_cast<char>(code)) {
            return &font_->glyphs[idx];
        }
    }
    return nullptr;
}

void WaveshareEpaper213::configureSpi() {
    if (spiConfigured_) {
        return;
    }
    if (!connection_.spi) {
        return;
    }
    spi_init(connection_.spi, connection_.baudRateHz ? connection_.baudRateHz : 4 * 1000 * 1000);
    spi_set_format(connection_.spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(connection_.mosiPin, GPIO_FUNC_SPI);
    gpio_set_function(connection_.sckPin, GPIO_FUNC_SPI);

    gpio_init(connection_.csPin);
    gpio_set_dir(connection_.csPin, GPIO_OUT);
    gpio_put(connection_.csPin, 1);

    gpio_init(connection_.dcPin);
    gpio_set_dir(connection_.dcPin, GPIO_OUT);
    gpio_put(connection_.dcPin, 0);

    gpio_init(connection_.rstPin);
    gpio_set_dir(connection_.rstPin, GPIO_OUT);
    gpio_put(connection_.rstPin, 1);

    gpio_init(connection_.busyPin);
    gpio_set_dir(connection_.busyPin, GPIO_IN);
    spiConfigured_ = true;
}

void WaveshareEpaper213::resetHardware() {
    gpio_put(connection_.rstPin, 1);
    sleep_ms(10);
    gpio_put(connection_.rstPin, 0);
    sleep_ms(10);
    gpio_put(connection_.rstPin, 1);
    sleep_ms(10);
}

void WaveshareEpaper213::powerOn() {
    sendCommand(0x01);
    sendData(0x07);
    sendData(0x00);
    sendData(0x2B);
    sendData(0x2B);
    sendData(0x09);

    sendCommand(0x06);
    sendData(0x17);
    sendData(0x17);
    sendData(0x17);

    sendCommand(0x04);
    waitUntilIdle();

    sendCommand(0x00);
    sendData(0x8F);

    sendCommand(0x30);
    sendData(0x3A);

    sendCommand(0x61);
    sendData(static_cast<uint8_t>((geometry_.width >> 8) & 0xFF));
    sendData(static_cast<uint8_t>(geometry_.width & 0xFF));
    sendData(static_cast<uint8_t>((geometry_.height >> 8) & 0xFF));
    sendData(static_cast<uint8_t>(geometry_.height & 0xFF));

    sendCommand(0x82);
    sendData(0x12);

    sendCommand(0x50);
    sendData(0x77);

    updateLookUpTable();
    poweredOn_ = true;
}

void WaveshareEpaper213::sendCommand(uint8_t command) {
    gpio_put(connection_.dcPin, 0);
    gpio_put(connection_.csPin, 0);
    spi_write_blocking(connection_.spi, &command, 1);
    gpio_put(connection_.csPin, 1);
}

void WaveshareEpaper213::sendData(uint8_t data) {
    gpio_put(connection_.dcPin, 1);
    gpio_put(connection_.csPin, 0);
    spi_write_blocking(connection_.spi, &data, 1);
    gpio_put(connection_.csPin, 1);
}

void WaveshareEpaper213::sendData(const uint8_t* data, size_t length) {
    gpio_put(connection_.dcPin, 1);
    gpio_put(connection_.csPin, 0);
    spi_write_blocking(connection_.spi, data, static_cast<int>(length));
    gpio_put(connection_.csPin, 1);
}

void WaveshareEpaper213::waitUntilIdle() const {
    uint8_t busyLevel = connection_.busyActiveHigh ? 0x01 : 0x00;
    while (gpio_get(connection_.busyPin) == busyLevel) {
        sleep_ms(2);
    }
}

void WaveshareEpaper213::updateLookUpTable() {
    sendCommand(0x20);
    sendData(kDefaultLut, sizeof(kDefaultLut));
}

void WaveshareEpaper213::fillBuffer(uint8_t value) {
    std::fill_n(frameBuffer_.get(), bufferSize_, value);
}

} // namespace DcsBios
