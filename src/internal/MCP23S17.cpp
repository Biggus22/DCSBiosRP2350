#include "MCP23S17.h"
#include "hardware/gpio.h" // Still needed for Pico's native GPIO functions used in constructor
#include "hardware/spi.h"

MCP23S17::MCP23S17(spi_inst_t* spi_port, uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t addr) {
    spi = spi_port;
    cs_pin = cs;
    address = addr;
    
    // Initialize SPI pins using Pico SDK functions
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(miso, GPIO_FUNC_SPI);
    
    // Initialize CS pin using Pico SDK functions
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1); // CS high (inactive)
}

void MCP23S17::begin() {
    // Initialize SPI
    spi_init(spi, 1000000); // 1MHz SPI clock (adjust as needed)
    
    // Configure IOCON register for hardware addressing.
    // Also, enable interrupt latching (INTCC=0, default after reset)
    // and disable sequential operation (SEQOP=1) for simpler register access.
    writeRegister(MCP23S17_IOCONA, MCP23S17_IOCON_HAEN | MCP23S17_IOCON_SEQOP); 
    writeRegister(MCP23S17_IOCONB, MCP23S17_IOCON_HAEN | MCP23S17_IOCON_SEQOP); 
    
    // Set all pins as inputs by default
    writeRegister(MCP23S17_IODIRA, 0xFF);
    writeRegister(MCP23S17_IODIRB, 0xFF);
}

void MCP23S17::pinMode(uint8_t pin, uint8_t mode) {
    uint8_t reg_iodir = (pin < 8) ? MCP23S17_IODIRA : MCP23S17_IODIRB;
    uint8_t bit = pin % 8;
    uint8_t current_iodir = readRegister(reg_iodir);
    
    if (mode == MCP23S17_PIN_OUTPUT) { 
        current_iodir &= ~(1 << bit); // Clear bit for output
    } else { // MCP23S17_PIN_INPUT or MCP23S17_PIN_INPUT_PULLUP
        current_iodir |= (1 << bit);  // Set bit for input
    }
    
    writeRegister(reg_iodir, current_iodir);
    
    // If INPUT_PULLUP, enable pullup
    if (mode == MCP23S17_PIN_INPUT_PULLUP) { 
        pullUp(pin, true); 
    } else {
        pullUp(pin, false); 
    }
}

void MCP23S17::digitalWrite(uint8_t pin, uint8_t value) {
    uint8_t reg_gpio = (pin < 8) ? MCP23S17_GPPA : MCP23S17_GPIOB; 
    uint8_t bit = pin % 8;
    uint8_t current_gpio = readRegister(reg_gpio);
    
    if (value == 1) { // HIGH
        current_gpio |= (1 << bit);   // Set bit
    } else { // LOW
        current_gpio &= ~(1 << bit);  // Clear bit
    }
    
    writeRegister(reg_gpio, current_gpio);
}

uint8_t MCP23S17::digitalRead(uint8_t pin) {
    uint8_t reg_gpio = (pin < 8) ? MCP23S17_GPPA : MCP23S17_GPIOB; 
    uint8_t bit = pin % 8;
    return (readRegister(reg_gpio) >> bit) & 0x01;
}

void MCP23S17::pullUp(uint8_t pin, bool enable) { 
    uint8_t reg_gppu = (pin < 8) ? MCP23S17_GPPUA : MCP23S17_GPPUB;
    uint8_t bit = pin % 8;
    uint8_t current_gppu = readRegister(reg_gppu);

    if (enable) { 
        current_gppu |= (1 << bit);
    } else { 
        current_gppu &= ~(1 << bit);
    }
    writeRegister(reg_gppu, current_gppu);
}

// --- New Method Implementations ---
void MCP23S17::setPortDirection(uint16_t direction) {
    writeRegister16(MCP23S17_IODIRA, direction);
}

void MCP23S17::setPullUps(uint16_t pullup_config) {
    writeRegister16(MCP23S17_GPPUA, pullup_config);
}
// --- End New Method Implementations ---

// --- New Interrupt Method Implementations ---
void MCP23S17::enableInterruptPin(uint8_t pin, bool enable) {
    uint8_t reg_gpinten = (pin < 8) ? MCP23S17_GPINTENA : MCP23S17_GPINTENB;
    uint8_t bit = pin % 8;
    uint8_t current_gpinten = readRegister(reg_gpinten);

    if (enable) {
        current_gpinten |= (1 << bit);
        // When enabling interrupts, also configure how they trigger
        // For typical switch/button: interrupt on change (compare to previous state)
        setInterruptControlMode(pin, true); // true = compare to previous value
        // For pull-up inputs: interrupt when pin goes low (default value 0)
        setDefaultComparisonValue(pin, 0); // 0 = compare against low value
    } else {
        current_gpinten &= ~(1 << bit);
        // Optionally, reset interrupt control and default value when disabling
        setInterruptControlMode(pin, false); // Back to compare to default (might not matter if interrupt is disabled)
        setDefaultComparisonValue(pin, 0); // Reset default to 0
    }
    writeRegister(reg_gpinten, current_gpinten);
}

void MCP23S17::setInterruptControlMode(uint8_t pin, bool compareToPrevious) {
    uint8_t reg_intcon = (pin < 8) ? MCP23S17_INTCONA : MCP23S17_INTCONB;
    uint8_t bit = pin % 8;
    uint8_t current_intcon = readRegister(reg_intcon);

    if (compareToPrevious) { // 0 = compare to previous pin value
        current_intcon &= ~(1 << bit);
    } else { // 1 = compare to default value in DEFVAL
        current_intcon |= (1 << bit);
    }
    writeRegister(reg_intcon, current_intcon);
}

void MCP23S17::setDefaultComparisonValue(uint8_t pin, uint8_t value) {
    uint8_t reg_defval = (pin < 8) ? MCP23S17_DEFVALA : MCP23S17_DEFVALB;
    uint8_t bit = pin % 8;
    uint8_t current_defval = readRegister(reg_defval);

    if (value == 1) { // HIGH
        current_defval |= (1 << bit);
    } else { // LOW
        current_defval &= ~(1 << bit);
    }
    writeRegister(reg_defval, current_defval);
}

void MCP23S17::configureInterruptOutput(bool openDrain, bool activeHigh) {
    uint8_t iocon_value_a = readRegister(MCP23S17_IOCONA);
    uint8_t iocon_value_b = readRegister(MCP23S17_IOCONB);

    if (openDrain) {
        iocon_value_a |= MCP23S17_IOCON_ODR;
        iocon_value_b |= MCP23S17_IOCON_ODR;
    } else {
        iocon_value_a &= ~MCP23S17_IOCON_ODR;
        iocon_value_b &= ~MCP23S17_IOCON_ODR;
    }

    if (activeHigh) {
        iocon_value_a |= MCP23S17_IOCON_INTPOL;
        iocon_value_b |= MCP23S17_IOCON_INTPOL;
    } else {
        iocon_value_a &= ~MCP23S17_IOCON_INTPOL;
        iocon_value_b &= ~MCP23S17_IOCON_INTPOL;
    }
    writeRegister(MCP23S17_IOCONA, iocon_value_a);
    writeRegister(MCP23S17_IOCONB, iocon_value_b);
}

uint16_t MCP23S17::getInterruptFlags() {
    return readRegister16(MCP23S17_INTFA);
}

uint16_t MCP23S17::getInterruptCapturedValues() {
    return readRegister16(MCP23S17_INTCAPA); 
}

// --- Private Helper Methods ---
uint8_t MCP23S17::readRegister(uint8_t reg) {
    uint8_t tx_buf[2] = {(uint8_t)((address << 1) | 0x01), reg}; 
    uint8_t rx_data;
    gpio_put(cs_pin, 0); 
    spi_write_read_blocking(spi, tx_buf, &rx_data, 2); 
    spi_read_blocking(spi, 0, &rx_data, 1); 
    gpio_put(cs_pin, 1); 
    return rx_data;
}

void MCP23S17::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[3] = {(uint8_t)(address << 1), reg, value}; 
    gpio_put(cs_pin, 0); 
    spi_write_blocking(spi, tx_buf, 3);
    gpio_put(cs_pin, 1); 
}

void MCP23S17::writeRegister16(uint8_t reg, uint16_t value) {
    writeRegister(reg, (uint8_t)value); 
    writeRegister(reg + 1, (uint8_t)(value >> 8)); 
}

uint16_t MCP23S17::readRegister16(uint8_t reg) {
    uint8_t val_low = readRegister(reg);
    uint8_t val_high = readRegister(reg + 1);
    return (uint16_t)val_high << 8 | val_low;
}