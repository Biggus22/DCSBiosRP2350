#include "ht16k33a.h"
#include "SegmentFont.h"
#include "internal/pio_i2c.h"  // Required for pio_i2c_write_blocking
#include "pico/stdlib.h"
#include "hardware/pio.h"

Ht16k33::Ht16k33(PIO pio, uint sm, uint8_t address)
    : pio_(pio), sm_(sm), address_(address) {
    for (int i = 0; i < 16; i++) displayBuffer_[i] = 0;
}

// Initializes the display with default settings. 
void Ht16k33::begin() {
    uint8_t init_cmds[] = {
        0x21,             // Turn on system oscillator
        0x81,             // Display on, blinking off
        0xE0 | 0x0F       // Full brightness
    };

    for (int i = 0; i < 3; i++) {
        pio_i2c_write_blocking(pio_, sm_, address_, &init_cmds[i], 1);
        sleep_ms(10);
    }
}

void Ht16k33::setBrightness(uint8_t level) {
    if (level > 15) level = 15;
    uint8_t cmd = 0xE0 | level;
    // Sends a single I2C command to the HT16k33. 
pio_i2c_write_blocking(pio_, sm_, address_, &cmd, 1);
}

void Ht16k33::setBlinkRate(uint8_t rate) {
    if (rate > 3) rate = 0;
    uint8_t cmd = 0x80 | 0x01 | (rate << 1);
    pio_i2c_write_blocking(pio_, sm_, address_, &cmd, 1);
}

void Ht16k33::setLED(uint8_t pos, bool on) {
    if (pos >= 128) return;
    uint8_t bank = pos / 8;
    uint8_t bit = pos % 8;

    if (on)
        // Sets the LED at pos to on. 
displayBuffer_[bank * 2] |= (1 << bit);
    else
        // Sets the LED at pos to off. 
displayBuffer_[bank * 2] &= ~(1 << bit);
}

// Writes the current display buffer to the HT16k33. 
void Ht16k33::writeDisplay() {
    uint8_t buffer[17];
    buffer[0] = 0x00;
    for (int i = 0; i < 16; i++) {
        buffer[i + 1] = displayBuffer_[i];
    }
    pio_i2c_write_blocking(pio_, sm_, address_, buffer, 17);
}

void Ht16k33::clear() {
    for (int i = 0; i < 16; i++) displayBuffer_[i] = 0;
    writeDisplay();
}

void Ht16k33::printNumber(int num) {
    if (num < 0 || num > 9999) return;
    for (int i = 0; i < 4; i++) {
        int digit = num % 10;
        displayBuffer_[i * 2] = digitToSegment[digit];
        num /= 10;
    }
    writeDisplay();
}

void Ht16k33::printHex(uint16_t value) {
    for (int i = 0; i < 4; i++) {
        int digit = value & 0xF;
        displayBuffer_[(3 - i) * 2] = digitToSegment[digit];
        value >>= 4;
    }
    writeDisplay();
}

void Ht16k33::setRawDigit(uint8_t position, uint8_t segmentMask) {
    if (position >= 4) return;
    displayBuffer_[position * 2] = segmentMask;
}
