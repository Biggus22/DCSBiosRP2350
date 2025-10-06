#include "TM1637.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <cstring>

// TM1637 command definitions
#define TM1637_CMD_DATA_AUTO 0x40
#define TM1637_CMD_DATA_FIXED 0x44
#define TM1637_CMD_DISPLAY 0x80
#define TM1637_CMD_ADDRESS 0xC0

// Digit to segment mapping for 7-segment display
static const uint8_t digitToSegment[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71, // F
    0x40, // -
    0x00  // blank
};

TM1637::TM1637(uint8_t clkPin, uint8_t dataPin) 
    : _clkPin(clkPin), _dataPin(dataPin), _brightness(7) {
}

void TM1637::begin() {
    gpio_init(_clkPin);
    gpio_init(_dataPin);
    gpio_set_dir(_clkPin, GPIO_OUT);
    gpio_set_dir(_dataPin, GPIO_OUT);
    gpio_put(_clkPin, 0);
    gpio_put(_dataPin, 0);
}

void TM1637::display(const char* text, bool reverseOrder, bool customOrder) {
    int len = strlen(text);
    uint8_t digits[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Initialize with blanks
    
    int pos = 0;
    for (int i = 0; i < len && pos < 6; i++) {
        if (text[i] == '.') {
            if (pos > 0) {
                // Add decimal point to previous digit
                digits[pos-1] |= 0x80;
            }
        } else {
            digits[pos++] = encodeDigit(text[i]);
        }
    }
    
    // Write data to display
    start();
    writeByte(TM1637_CMD_DATA_AUTO);
    stop();
    
    start();
    writeByte(TM1637_CMD_ADDRESS);
    
    if (customOrder) {
        // Custom order: 3, 2, 1, 6, 5, 4
        writeByte(digits[2]); // Position 3
        writeByte(digits[1]); // Position 2
        writeByte(digits[0]); // Position 1
        writeByte(digits[5]); // Position 6
        writeByte(digits[4]); // Position 5
        writeByte(digits[3]); // Position 4
    } else if (reverseOrder) {
        // Send digits in reverse order (D6, D5, D4, D3, D2, D1)
        for (int i = 5; i >= 0; i--) {
            writeByte(digits[i]);
        }
    } else {
        // Send digits in normal order (D1, D2, D3, D4, D5, D6)
        for (int i = 0; i < 6; i++) {
            writeByte(digits[i]);
        }
    }
    
    stop();
    
    // Display control command
    start();
    writeByte(TM1637_CMD_DISPLAY | 0x08 | (_brightness & 0x07));
    stop();
}

void TM1637::displayWithDecimalPoint(const char* text, int decimalPosition, bool reverseOrder, bool customOrder) {
    int len = strlen(text);
    uint8_t digits[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Initialize with blanks

    for (int i = 0; i < len && i < 6; i++) {
        digits[i] = encodeDigit(text[i]);
    }
    
    // Add the decimal point to the specified position
    if (decimalPosition >= 0 && decimalPosition < 6) {
        digits[decimalPosition] |= 0x80;
    }

    // Write data to display
    start();
    writeByte(TM1637_CMD_DATA_AUTO);
    stop();
    
    start();
    writeByte(TM1637_CMD_ADDRESS);
    
    if (customOrder) {
        // Custom order: 3, 2, 1, 6, 5, 4
        writeByte(digits[2]); // Position 3
        writeByte(digits[1]); // Position 2
        writeByte(digits[0]); // Position 1
        writeByte(digits[5]); // Position 6
        writeByte(digits[4]); // Position 5
        writeByte(digits[3]); // Position 4
    } else if (reverseOrder) {
        // Send digits in reverse order (D6, D5, D4, D3, D2, D1)
        for (int i = 5; i >= 0; i--) {
            writeByte(digits[i]);
        }
    } else {
        // Send digits in normal order (D1, D2, D3, D4, D5, D6)
        for (int i = 0; i < 6; i++) {
            writeByte(digits[i]);
        }
    }
    
    stop();
    
    // Display control command
    start();
    writeByte(TM1637_CMD_DISPLAY | 0x08 | (_brightness & 0x07));
    stop();
}

void TM1637::displayRaw(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5, uint8_t digit6) {
    // Write data to display
    start();
    writeByte(TM1637_CMD_DATA_AUTO);
    stop();
    
    start();
    writeByte(TM1637_CMD_ADDRESS);
    writeByte(digit1);
    writeByte(digit2);
    writeByte(digit3);
    writeByte(digit4);
    writeByte(digit5);
    writeByte(digit6);
    stop();
    
    // Display control command
    start();
    writeByte(TM1637_CMD_DISPLAY | 0x08 | (_brightness & 0x07));
    stop();
}

void TM1637::clear() {
    displayRaw(0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
}

void TM1637::setBrightness(uint8_t brightness) {
    _brightness = brightness & 0x07;
}

void TM1637::start() {
    gpio_put(_clkPin, 1);
    gpio_put(_dataPin, 1);
    sleep_us(2);
    gpio_put(_dataPin, 0);
    sleep_us(2);
    gpio_put(_clkPin, 0);
    sleep_us(2);
}

void TM1637::stop() {
    gpio_put(_clkPin, 0);
    sleep_us(2);
    gpio_put(_dataPin, 0);
    sleep_us(2);
    gpio_put(_clkPin, 1);
    sleep_us(2);
    gpio_put(_dataPin, 1);
    sleep_us(2);
}

void TM1637::writeByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        gpio_put(_clkPin, 0);
        sleep_us(2);
        gpio_put(_dataPin, data & 0x01);
        sleep_us(2);
        gpio_put(_clkPin, 1);
        sleep_us(2);
        data >>= 1;
    }
    
    // Wait for ACK
    gpio_put(_clkPin, 0);
    sleep_us(2);
    gpio_set_dir(_dataPin, GPIO_IN);
    sleep_us(2);
    gpio_put(_clkPin, 1);
    sleep_us(2);
    gpio_put(_clkPin, 0);
    sleep_us(2);
    gpio_set_dir(_dataPin, GPIO_OUT);
}

void TM1637::writeCommand(uint8_t command) {
    start();
    writeByte(command);
    stop();
}

uint8_t TM1637::encodeDigit(char digit) {
    switch (digit) {
        case '0': return digitToSegment[0];
        case '1': return digitToSegment[1];
        case '2': return digitToSegment[2];
        case '3': return digitToSegment[3];
        case '4': return digitToSegment[4];
        case '5': return digitToSegment[5];
        case '6': return digitToSegment[6];
        case '7': return digitToSegment[7];
        case '8': return digitToSegment[8];
        case '9': return digitToSegment[9];
        case 'A': case 'a': return digitToSegment[10];
        case 'B': case 'b': return digitToSegment[11];
        case 'C': case 'c': return digitToSegment[12];
        case 'D': case 'd': return digitToSegment[13];
        case 'E': case 'e': return digitToSegment[14];
        case 'F': case 'f': return digitToSegment[15];
        case '-': return digitToSegment[16];
        case ' ': return digitToSegment[17];
        default: return 0x00;
    }
}