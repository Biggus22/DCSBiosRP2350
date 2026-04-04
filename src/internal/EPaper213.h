#pragma once

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stddef.h>
#include <stdint.h>

namespace DcsBios {

struct Epaper213Pins {
    uint cs;
    uint dc;
    uint rst;
    uint busy;
    uint sck;
    uint mosi;
};

class Epaper213 {
public:
    static constexpr int width = 122;
    static constexpr int height = 250;
    // Logical landscape space we draw into (rotated onto hardware buffer)
    static constexpr int logicalWidth = 250;
    static constexpr int logicalHeight = 122;
    static constexpr size_t bufferSize = (width * height + 7) / 8;

    explicit Epaper213(const Epaper213Pins& pins = defaultPins(), spi_inst_t* spi = spi1);

    bool init(uint32_t baud = 4'000'000);
    void clear();
    void displayBuffer(const uint8_t* buffer);
    void displayText(const char* text);
    bool beginDisplayText(const char* text);
    bool process();
    void sleep();
    void clearMargin(uint8_t pixels = 32);

    // Default pins mapped to GPIO0–15 for Waveshare RP2040-Zero (SPI1 SCK=10, MOSI=11) to avoid UART0/I2C lines
    static constexpr Epaper213Pins defaultPins() {
        return Epaper213Pins{.cs = 6, .dc = 7, .rst = 8, .busy = 9, .sck = 10, .mosi = 11};
    }

private:
    void sendCommand(uint8_t cmd);
    void sendData(uint8_t data);
    void sendData(const uint8_t* data, size_t len);
    void waitWhileBusy();
    void reset();
    void setWindows(uint8_t xStart, uint8_t yStart, uint8_t xEnd, uint8_t yEnd);
    void setCursor(uint8_t x, uint8_t y);
    void refresh(uint8_t ctrl);

    void clearBuffer(uint8_t* buffer);
    void drawTextLine(uint8_t* buffer, const char* text);
    void drawGlyph(uint8_t* buffer, int originX, int originY, char c);
    void setPixel(uint8_t* buffer, int x, int y, bool black);

    Epaper213Pins pins_;
    spi_inst_t* spi_;
    bool initialized_;
    uint8_t frameBuffer[bufferSize];
    uint8_t previousFrameBuffer[bufferSize];
    uint8_t windowBuffer[bufferSize];
    size_t windowBytes_ = 0;
    size_t windowOffset_ = 0;
    uint8_t windowXByteStart_ = 0;
    uint8_t windowWidthBytes_ = 0;
    uint16_t windowYStart_ = 0;
    uint16_t windowYEnd_ = 0;
    bool windowPrepared_ = false;
    bool marginCleared_ = false;
    size_t txOffset_ = 0;
    bool sending_ = false;
    bool commandSent_ = false;
    bool writingOldPlane_ = false;
    bool refreshed_ = false;
};

}  // namespace DcsBios
