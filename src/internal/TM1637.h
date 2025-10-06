#ifndef TM1637_H
#define TM1637_H

#include "pico/stdlib.h"

class TM1637 {
public:
    TM1637(uint8_t clkPin, uint8_t dataPin);
    void begin();
    void display(const char* text, bool reverseOrder = false, bool customOrder = false);
    void displayWithDecimalPoint(const char* text, int decimalPosition, bool reverseOrder = false, bool customOrder = false);
    void displayRaw(uint8_t digit1, uint8_t digit2, uint8_t digit3, uint8_t digit4, uint8_t digit5, uint8_t digit6);
    void clear();
    void setBrightness(uint8_t brightness); // 0-7

private:
    uint8_t _clkPin;
    uint8_t _dataPin;
    uint8_t _brightness;
    
    void start();
    void stop();
    void writeByte(uint8_t data);
    void writeCommand(uint8_t command);
    uint8_t encodeDigit(char digit);
};

#endif // TM1637_H