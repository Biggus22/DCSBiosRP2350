#include "EPaper213.h"

#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include <algorithm>
#include <string.h>

namespace DcsBios {

namespace {
// 8x16 glyphs for 0-9, '.', and space. Bits set to 1 mean "draw black".
static const uint8_t kDigitFont[12][16] = {
    {0x00,0x3C,0x42,0x46,0x4A,0x52,0x62,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,0x00}, // 0
    {0x00,0x08,0x18,0x28,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x3E,0x00,0x00,0x00,0x00}, // 1
    {0x00,0x3C,0x42,0x42,0x02,0x04,0x08,0x10,0x20,0x40,0x40,0x7E,0x00,0x00,0x00,0x00}, // 2
    {0x00,0x3C,0x42,0x02,0x02,0x1C,0x02,0x02,0x02,0x02,0x42,0x3C,0x00,0x00,0x00,0x00}, // 3
    {0x00,0x04,0x0C,0x14,0x24,0x44,0x84,0xFF,0x04,0x04,0x04,0x0E,0x00,0x00,0x00,0x00}, // 4
    {0x00,0x7E,0x40,0x40,0x7C,0x02,0x02,0x02,0x02,0x02,0x42,0x3C,0x00,0x00,0x00,0x00}, // 5
    {0x00,0x1C,0x20,0x40,0x40,0x7C,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,0x00}, // 6
    {0x00,0x7E,0x02,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x20,0x20,0x00,0x00,0x00,0x00}, // 7
    {0x00,0x3C,0x42,0x42,0x42,0x3C,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,0x00}, // 8
    {0x00,0x3C,0x42,0x42,0x42,0x42,0x3E,0x02,0x02,0x02,0x04,0x38,0x00,0x00,0x00,0x00}, // 9
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00}, // .
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // space
};

const uint8_t* glyphFor(char c) {
    if (c >= '0' && c <= '9') return kDigitFont[c - '0'];
    if (c == '.') return kDigitFont[10];
    return kDigitFont[11];
}

}  // namespace

static constexpr int kScale = 3;   // enlarge glyphs for readability
static constexpr int kSpacing = 4; // pixels between glyphs at scaled size
static constexpr int kMaxChars = 6; // fixed slot width to keep centering stable

Epaper213::Epaper213(const Epaper213Pins& pins, spi_inst_t* spi)
    : pins_(pins), spi_(spi), initialized_(false), windowBytes_(0), windowOffset_(0), windowXByteStart_(0), windowWidthBytes_(0), windowYStart_(0), windowYEnd_(0), windowPrepared_(false), marginCleared_(false), txOffset_(0), sending_(false), commandSent_(false), writingOldPlane_(false), refreshed_(false) {}

bool Epaper213::init(uint32_t baud) {
    gpio_init(pins_.cs);
    gpio_init(pins_.dc);
    gpio_init(pins_.rst);
    gpio_init(pins_.busy);
    gpio_set_dir(pins_.cs, GPIO_OUT);
    gpio_set_dir(pins_.dc, GPIO_OUT);
    gpio_set_dir(pins_.rst, GPIO_OUT);
    gpio_set_dir(pins_.busy, GPIO_IN);
    gpio_pull_up(pins_.busy);

    gpio_put(pins_.cs, 1);
    gpio_put(pins_.dc, 0);
    gpio_put(pins_.rst, 1);

    spi_init(spi_, baud);
    spi_set_format(spi_, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(pins_.sck, GPIO_FUNC_SPI);
    gpio_set_function(pins_.mosi, GPIO_FUNC_SPI);

    reset();
    waitWhileBusy();
    sendCommand(0x12);  // soft reset
    waitWhileBusy();

    sendCommand(0x01);  // driver output control
    sendData(0xF9);
    sendData(0x00);
    sendData(0x00);

    sendCommand(0x11);  // data entry mode
    sendData(0x03);

    setWindows(0, 0, width - 1, height - 1);
    setCursor(0, 0);

    sendCommand(0x3C);  // border waveform
    sendData(0x05);

    sendCommand(0x21);  // display update control
    sendData(0x00);
    sendData(0x80);

    sendCommand(0x18);  // built-in temperature sensor
    sendData(0x80);
    waitWhileBusy();

    initialized_ = true;
    clear();
    clearBuffer(previousFrameBuffer);
    return true;
}

void Epaper213::reset() {
    gpio_put(pins_.rst, 1);
    sleep_ms(20);
    gpio_put(pins_.rst, 0);
    sleep_ms(2);
    gpio_put(pins_.rst, 1);
    sleep_ms(20);
}

void Epaper213::sendCommand(uint8_t cmd) {
    gpio_put(pins_.dc, 0);
    gpio_put(pins_.cs, 0);
    spi_write_blocking(spi_, &cmd, 1);
    gpio_put(pins_.cs, 1);
}

void Epaper213::sendData(uint8_t data) {
    gpio_put(pins_.dc, 1);
    gpio_put(pins_.cs, 0);
    spi_write_blocking(spi_, &data, 1);
    gpio_put(pins_.cs, 1);
}

void Epaper213::sendData(const uint8_t* data, size_t len) {
    if (!data || len == 0) return;
    gpio_put(pins_.dc, 1);
    gpio_put(pins_.cs, 0);
    spi_write_blocking(spi_, data, len);
    gpio_put(pins_.cs, 1);
}

void Epaper213::waitWhileBusy() {
    // Busy is HIGH while the panel is busy; add a timeout so we never block DCS-BIOS
    const uint32_t timeout_ms = 500; // hard ceiling
    absolute_time_t start = get_absolute_time();
    while (gpio_get(pins_.busy)) {
        if (absolute_time_diff_us(start, get_absolute_time()) > timeout_ms * 1000) {
            break; // give up to keep serial alive
        }
        sleep_ms(5);
    }
}

void Epaper213::setWindows(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd) {
    sendCommand(0x44);
    sendData(xStart >> 3);
    sendData(xEnd >> 3);

    sendCommand(0x45);
    sendData(yStart & 0xFF);
    sendData((yStart >> 8) & 0xFF);
    sendData(yEnd & 0xFF);
    sendData((yEnd >> 8) & 0xFF);
}

void Epaper213::setCursor(uint8_t x, uint8_t y) {
    sendCommand(0x4E);
    sendData(x & 0xFF);

    sendCommand(0x4F);
    sendData(y & 0xFF);
    sendData((y >> 8) & 0xFF);
}

void Epaper213::refresh(uint8_t ctrl) {
    sendCommand(0x22);
    sendData(ctrl);
    sendCommand(0x20);
}

void Epaper213::clear() {
    if (!initialized_) return;
    if (gpio_get(pins_.busy)) return; // skip if panel busy
    sendCommand(0x24);
    for (size_t i = 0; i < bufferSize; ++i) {
        sendData(0x00);
    }
    sendCommand(0x26);
    for (size_t i = 0; i < bufferSize; ++i) {
        sendData(0x00);
    }
    refresh(0xF7);
    clearBuffer(previousFrameBuffer);
}

void Epaper213::displayBuffer(const uint8_t* buffer) {
    if (!initialized_ || buffer == nullptr) return;
    if (gpio_get(pins_.busy)) return; // avoid sending while busy
    sendCommand(0x24);
    sendData(buffer, bufferSize);
    sendCommand(0x26);
    sendData(buffer, bufferSize);
    refresh(0xF7);
    memcpy(previousFrameBuffer, buffer, bufferSize);
}

void Epaper213::displayText(const char* text) {
    if (!initialized_) return;
    uint8_t buffer[bufferSize];
    clearBuffer(buffer);
    drawTextLine(buffer, text);
    displayBuffer(buffer);
}

void Epaper213::clearMargin(uint8_t pixels) {
    if (!initialized_) return;
    if (pixels == 0) return;
    if (pixels > width) pixels = width;
    const uint8_t xByteEnd = static_cast<uint8_t>((pixels - 1) / 8);
    setWindows(0, 0, pixels - 1, height - 1);
    setCursor(0, 0);
    sendCommand(0x24);
    // Fill with white
    const size_t bytesPerRow = (width + 7) / 8; // 16
    const size_t bandBytesPerRow = static_cast<size_t>(xByteEnd + 1);
    uint8_t rowBuf[32];
    for (size_t i = 0; i < bandBytesPerRow; ++i) rowBuf[i] = 0xFF;
    for (int y = 0; y < height; ++y) {
        sendData(rowBuf, bandBytesPerRow);
    }
    refresh(0xF7);
}

bool Epaper213::beginDisplayText(const char* text) {
    if (!initialized_ || sending_ || !text) return false;
    clearBuffer(frameBuffer);
    drawTextLine(frameBuffer, text);
    // Use full-frame updates to prevent stale pixels and background noise artifacts.
    windowXByteStart_ = 0;
    windowWidthBytes_ = static_cast<uint8_t>((width + 7) / 8);
    windowYStart_ = 0;
    windowYEnd_ = static_cast<uint16_t>(height - 1);
    windowBytes_ = bufferSize;
    memcpy(windowBuffer, frameBuffer, bufferSize);
    windowPrepared_ = true;
    windowOffset_ = 0;
    txOffset_ = 0;
    sending_ = true;
    commandSent_ = false;
    writingOldPlane_ = true;
    refreshed_ = false;
    return true;
}

bool Epaper213::process() {
    if (!initialized_) return true;
    if (!sending_) return true;
    if (gpio_get(pins_.busy)) return false; // wait until panel is free

    if (!windowPrepared_) return true; // nothing to send

    if (!commandSent_) {
        // Set the partial window
        const uint8_t xStartPix = static_cast<uint8_t>(windowXByteStart_ * 8);
        uint16_t xEndPixRaw = static_cast<uint16_t>(xStartPix) + static_cast<uint16_t>(windowWidthBytes_ * 8) - 1;
        const uint8_t xEndPix = static_cast<uint8_t>(std::min<uint16_t>(width - 1, xEndPixRaw));
        setWindows(xStartPix, windowYStart_, xEndPix, windowYEnd_);
        setCursor(xStartPix, windowYStart_);
        sendCommand(writingOldPlane_ ? 0x26 : 0x24);
        commandSent_ = true;
        return false;
    }

    const size_t chunk = 128;
    const size_t remaining = windowBytes_ - windowOffset_;
    if (remaining > 0) {
        const size_t n = remaining > chunk ? chunk : remaining;
        const uint8_t* src = writingOldPlane_ ? previousFrameBuffer : windowBuffer;
        sendData(src + windowOffset_, n);
        windowOffset_ += n;
        return false;
    }

    // Write old plane first, then new plane.
    if (writingOldPlane_) {
        writingOldPlane_ = false;
        commandSent_ = false;
        windowOffset_ = 0;
        return false;
    }

    if (!refreshed_) {
        // Partial update avoids the full-screen white flash on each frequency change.
        refresh(0xFF);
        refreshed_ = true;
        return false;
    }

    sending_ = false;
    windowPrepared_ = false;
    writingOldPlane_ = false;
    memcpy(previousFrameBuffer, windowBuffer, bufferSize);
    return true;
}

void Epaper213::sleep() {
    if (!initialized_) return;
    sendCommand(0x10);
    sendData(0x01);
    sleep_ms(200);
}

void Epaper213::clearBuffer(uint8_t* buffer) {
    memset(buffer, 0x00, bufferSize);
}

void Epaper213::setPixel(uint8_t* buffer, int x, int y, bool black) {
    // Logical coordinates are landscape; rotate onto hardware portrait buffer
    if (x < 0 || x >= logicalWidth || y < 0 || y >= logicalHeight) return;

    const int hwX = y;                       // maps to hardware width (0..121)
    const int hwY = (logicalWidth - 1) - x;  // maps to hardware height (0..249)

    if (hwX < 0 || hwX >= width || hwY < 0 || hwY >= height) return;

    const int byteIndex = (hwX / 8) + hwY * ((width + 7) / 8);
    const uint8_t mask = 0x80 >> (hwX % 8);
    if (black) {
        buffer[byteIndex] &= static_cast<uint8_t>(~mask);
    } else {
        buffer[byteIndex] |= mask;
    }
}

void Epaper213::drawGlyph(uint8_t* buffer, int originX, int originY, char c) {
    const uint8_t* glyph = glyphFor(c);
    for (int y = 0; y < 16; ++y) {
        uint8_t row = glyph[y];
        for (int x = 0; x < 8; ++x) {
            bool bit = row & (0x80 >> x);
            if (!bit) continue;
            // Scale each lit pixel into a kScale x kScale block
            for (int dy = 0; dy < kScale; ++dy) {
                for (int dx = 0; dx < kScale; ++dx) {
                    setPixel(buffer, originX + x * kScale + dx, originY + y * kScale + dy, false);
                }
            }
        }
    }
}

void Epaper213::drawTextLine(uint8_t* buffer, const char* text) {
    if (!text) return;
    char tmp[7] = {0};
    size_t idx = 0;
    for (const char* p = text; *p && idx < 6; ++p) {
        if ((*p >= '0' && *p <= '9') || *p == '.') {
            tmp[idx++] = *p;
        }
    }
    if (idx == 0) {
        tmp[idx++] = ' ';
    }
    const int glyphWidth = 8 * kScale;
    const int glyphHeight = 16 * kScale;
    const int slotWidth = kMaxChars * (glyphWidth + kSpacing) - kSpacing;
    const int startX = std::max(0, (logicalWidth - slotWidth) / 2);
    const int baseY = std::max(0, (logicalHeight - glyphHeight) / 2);
    const int startY = std::max(0, baseY - 20); // shift text upward by 20 px
    // Center current text within the fixed slot so position stays stable between updates
    const int textWidth = static_cast<int>(idx) * (glyphWidth + kSpacing) - kSpacing;
    int cursorX = startX + std::max(0, (slotWidth - textWidth) / 2);

    for (size_t i = 0; i < idx; ++i) {
        drawGlyph(buffer, cursorX, startY, tmp[i]);
        cursorX += glyphWidth + kSpacing;
    }
}

}  // namespace DcsBios
