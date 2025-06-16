#include "MCP23S17.h"
#include "hardware/gpio.h"

MCP23S17::MCP23S17(spi_inst_t* spi_port, uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t addr) {
    spi = spi_port;
    cs_pin = cs;
    address = addr;
    
    // Initialize SPI pins
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(miso, GPIO_FUNC_SPI);
    
    // Initialize CS pin
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS high (inactive)
}

void MCP23S17::begin() {
    // Initialize SPI
    spi_init(spi, 1000000); // 1MHz SPI clock
    
    // Configure IOCON register for hardware addressing
    writeRegister(MCP23S17_IOCONA, 0x08); // Enable hardware addressing
    
    // Set all pins as inputs by default
    writeRegister(MCP23S17_IODIRA, 0xFF);
    writeRegister(MCP23S17_IODIRB, 0xFF);
}

void MCP23S17::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[3];
    tx_buf[0] = MCP23S17_WRITE_CMD | ((address & 0x07) << 1);
    tx_buf[1] = reg;
    tx_buf[2] = value;
    
    gpio_put(cs_pin, 0); // CS low (active)
    spi_write_blocking(spi, tx_buf, 3);
    gpio_put(cs_pin, 1); // CS high (inactive)
}

uint8_t MCP23S17::readRegister(uint8_t reg) {
    uint8_t tx_buf[3];
    uint8_t rx_buf[3];
    
    tx_buf[0] = MCP23S17_READ_CMD | ((address & 0x07) << 1);
    tx_buf[1] = reg;
    tx_buf[2] = 0x00; // Dummy byte
    
    gpio_put(cs_pin, 0); // CS low (active)
    spi_write_read_blocking(spi, tx_buf, rx_buf, 3);
    gpio_put(cs_pin, 1); // CS high (inactive)
    
    return rx_buf[2];
}

void MCP23S17::pinMode(uint8_t pin, uint8_t mode) {
    uint8_t reg = (pin < 8) ? MCP23S17_IODIRA : MCP23S17_IODIRB;
    uint8_t bit = pin % 8;
    uint8_t current = readRegister(reg);
    
    if (mode == OUTPUT) {
        current &= ~(1 << bit); // Clear bit for output
    } else {
        current |= (1 << bit);  // Set bit for input
    }
    
    writeRegister(reg, current);
    
    // If INPUT_PULLUP, enable pullup
    if (mode == INPUT_PULLUP) {
        pullUp(pin, HIGH);
    }
}

void MCP23S17::digitalWrite(uint8_t pin, uint8_t value) {
    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t bit = pin % 8;
    uint8_t current = readRegister(reg);
    
    if (value == HIGH) {
        current |= (1 << bit);   // Set bit
    } else {
        current &= ~(1 << bit);  // Clear bit
    }
    
    writeRegister(reg, current);
}

uint8_t MCP23S17::digitalRead(uint8_t pin) {
    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t bit = pin % 8;
    uint8_t current = readRegister(reg);
    
    return (current >> bit) & 0x01;
}

void MCP23S17::pullUp(uint8_t pin, uint8_t value) {
    uint8_t reg = (pin < 8) ? MCP23S17_GPPUA : MCP23S17_GPPUB;
    uint8_t bit = pin % 8;
    uint8_t current = readRegister(reg);
    
    if (value == HIGH) {
        current |= (1 << bit);   // Enable pullup
    } else {
        current &= ~(1 << bit);  // Disable pullup
    }
    
    writeRegister(reg, current);
}