#ifndef __DCSBIOS_WAVESHARE_EPAPER_213_H__
#define __DCSBIOS_WAVESHARE_EPAPER_213_H__

#include <stdint.h>
#include <cstddef>
#include <memory>
#include "hardware/spi.h"
#include "pico/stdlib.h"

namespace DcsBios {

struct WaveshareEpaperConnection {
    spi_inst_t* spi = nullptr;
    uint8_t mosiPin = 0;
    uint8_t sckPin = 0;
    uint8_t csPin = 0;
    uint8_t dcPin = 0;
    uint8_t rstPin = 0;
    uint8_t busyPin = 0;
    bool busyActiveHigh = false;
    uint32_t baudRateHz = 4 * 1000 * 1000;
};

struct WaveshareEpaperGeometry {
    uint16_t width;
    uint16_t height;
};

struct EpaperGlyph {
    char code;
    uint8_t width;
    uint8_t columns[5];
};

struct EpaperFontDescriptor {
    const EpaperGlyph* glyphs;
    size_t glyphCount;
    uint8_t height;
    uint8_t spacing;
    uint8_t defaultWidth;
};

class WaveshareEpaper213 {
public:
    static constexpr uint16_t DefaultWidth = 212;
    static constexpr uint16_t DefaultHeight = 104;

    explicit WaveshareEpaper213(const WaveshareEpaperConnection& connection,
                                WaveshareEpaperGeometry geometry = {DefaultWidth, DefaultHeight});

    void init(bool autoClear = true);
    void display(bool waitForIdle = true);
    void clear(bool black = false);
    void sleep();
    void setTextFont(const EpaperFontDescriptor* font);

    void drawPixel(uint16_t x, uint16_t y, bool black = true);
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool black = true);
    void drawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, bool black = true);
    void fillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, bool black = true);
    void drawBitmap(uint16_t x, uint16_t y, uint16_t bitmapWidth, uint16_t bitmapHeight,
                    const uint8_t* bitmap, bool invert = false);
    void drawText(uint16_t x, uint16_t y, const char* text, bool black = true, bool wrap = false);

    static const EpaperFontDescriptor BasicFont;

private:
    void configureSpi();
    void resetHardware();
    void powerOn();
    void sendCommand(uint8_t command);
    void sendData(uint8_t data);
    void sendData(const uint8_t* data, size_t length);
    void waitUntilIdle() const;
    void updateLookUpTable();
    void fillBuffer(uint8_t value);
    const EpaperGlyph* findGlyph(char code) const;
    void drawCharacter(uint16_t x, uint16_t y, const EpaperGlyph& glyph, bool black,
                       const EpaperFontDescriptor& font);

    WaveshareEpaperConnection connection_;
    WaveshareEpaperGeometry geometry_;
    uint16_t lineBytes_;
    size_t bufferSize_;
    std::unique_ptr<uint8_t[]> frameBuffer_;
    const EpaperFontDescriptor* font_;
    bool spiConfigured_;
    bool poweredOn_;
};

} // namespace DcsBios

#endif // __DCSBIOS_WAVESHARE_EPAPER_213_H__
