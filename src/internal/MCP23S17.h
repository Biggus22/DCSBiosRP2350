#ifndef MCP23S17_H
#define MCP23S17_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h" // Include for GPIO_IN/OUT if needed elsewhere, but not for MCP23S17 modes

// MCP23S17 Specific Pin Modes (to align with Pico SDK philosophy but for expander)
// These are internal to how the MCP23S17 operates its pins.
// They are given distinct names to avoid confusion with Pico's native GPIO_IN/GPIO_OUT.
#define _OUTPUT     0
#define _INPUT      1
#define _INPUT_PULLUP 2

// MCP23S17 Register definitions (remaining same)
#define MCP23S17_IODIRA   0x00
#define MCP23S17_IODIRB   0x01
#define MCP23S17_IPOLA    0x02
#define MCP23S17_IPOLB    0x03
#define MCP23S17_GPINTENA 0x04
#define MCP23S17_GPINTENB 0x05
#define MCP23S17_DEFVALA  0x06
#define MCP23S17_DEFVALB  0x07
#define MCP23S17_INTCONA  0x08
#define MCP23S17_INTCONB  0x09
#define MCP23S17_IOCONA   0x0A
#define MCP23S17_IOCONB   0x0B
#define MCP23S17_GPPUA    0x0C
#define MCP23S17_GPPUB    0x0D
#define MCP23S17_INTFA    0x0E
#define MCP23S17_INTFB    0x0F
#define MCP23S17_INTCAPA  0x10
#define MCP23S17_INTCAPB  0x11
#define MCP23S17_GPPA     0x12 
#define MCP23S17_GPIOB    0x13
#define MCP23S17_OLATA    0x14
#define MCP23S17_OLATB    0x15

// IOCON register bits (remaining same)
#define MCP23S17_IOCON_BANK      0x80 // Bank select
#define MCP23S17_IOCON_MIRROR    0x40 // INT output mirror
#define MCP23S17_IOCON_SEQOP     0x20 // Sequential operation enable
#define MCP23S17_IOCON_DISSLW    0x10 // Slew rate control disable
#define MCP23S17_IOCON_HAEN      0x08 // Hardware address enable
#define MCP23S17_IOCON_ODR       0x04 // Open drain interrupt output
#define MCP23S17_IOCON_INTPOL    0x02 // Interrupt output polarity
#define MCP23S17_IOCON_INTPIN    0x01 // Interrupt pin select

class MCP23S17 {
public:
    MCP23S17(spi_inst_t* spi_port, uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t addr);

    void begin();

    // Set direction for a single pin (using MCP23S17_PIN_OUTPUT, MCP23S17_PIN_INPUT, MCP23S17_PIN_INPUT_PULLUP)
    void pinMode(uint8_t pin, uint8_t direction);

    // Write a digital value to a single pin (0 or 1, or equivalent of false/true)
    void digitalWrite(uint8_t pin, uint8_t value);

    // Read a digital value from a single pin (returns 0 or 1)
    uint8_t digitalRead(uint8_t pin);

    // Set pull-up resistor for a single pin (true to enable)
    void pullUp(uint8_t pin, bool enable);

    // Set direction for all pins on both ports (0xAAAA for all output, 0xFFFF for all input)
    void setPortDirection(uint16_t direction);

    // Set pull-up resistors for all pins on both ports (0xFFFF to enable all)
    void setPullUps(uint16_t pullUps);

    // Set output values for all pins on both ports
    void writePort(uint16_t value);

    // Read input values from all pins on both ports
    uint16_t readPort();

    // --- Interrupt Methods ---
    void enableInterruptPin(uint8_t pin, bool enable);
    void setInterruptControlMode(uint8_t pin, bool compareToPrevious);
    void setDefaultComparisonValue(uint8_t pin, uint8_t value);
    void configureInterruptOutput(bool openDrain, bool activeHigh);
    uint16_t getInterruptFlags(); 
    uint16_t getInterruptCapturedValues();

public: 
    spi_inst_t* spi;
    uint8_t cs_pin;
    uint8_t address;

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    uint16_t readRegister16(uint8_t reg);
    void writeRegister16(uint8_t reg, uint16_t value);
};

#endif // MCP23S17_H