#ifndef MCP23S17_H
#define MCP23S17_H

#include "pico/stdlib.h"
#include "hardware/spi.h"

// MCP23S17 Register definitions
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
#define MCP23S17_GPIOA    0x12
#define MCP23S17_GPIOB    0x13
#define MCP23S17_OLATA    0x14
#define MCP23S17_OLATB    0x15

// Constants
#define HIGH 1
#define LOW 0
#define INPUT 1
#define OUTPUT 0
#define INPUT_PULLUP 2

// MCP23S17 SPI Commands
#define MCP23S17_WRITE_CMD 0x40
#define MCP23S17_READ_CMD  0x41

class MCP23S17 {
private:
    spi_inst_t* spi;
    uint8_t cs_pin;
    uint8_t address;
    
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    
public:
    // Constructor matching your usage
    MCP23S17(spi_inst_t* spi_port, uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t addr);
    
    void begin();
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t value);
    uint8_t digitalRead(uint8_t pin);
    void pullUp(uint8_t pin, uint8_t value);
};

#endif // MCP23S17_H