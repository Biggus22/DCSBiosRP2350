#include "AW9523B.h"
#include <stdio.h>
#include "pico/stdlib.h"

AW9523B::AW9523B(i2c_inst_t* i2c, uint8_t address)
    : _i2c(i2c), _address(address) {}

    bool AW9523B::begin(uint8_t mode) {
        // 1. Configure CTL (ISEL + GPOMD)
        if (!writeRegister(AW9523B_REG_CTL, mode)) return false;
    
        // 2. Set all pins as outputs
        if (!writeRegister(AW9523B_REG_CONFIG_PORT0, 0x00)) return false;
        if (!writeRegister(AW9523B_REG_CONFIG_PORT1, 0x00)) return false;
    
        // 3. Set LED mode bits correctly (IMPORTANT: 0 = LED, 1 = GPIO)
        if (mode == AW9523B_MODE_GPIO) {
            // All GPIO mode
            if (!writeRegister(AW9523B_REG_LED_MODE_PORT0, 0xFF)) return false;
            if (!writeRegister(AW9523B_REG_LED_MODE_PORT1, 0xFF)) return false;
        } else {
            // All LED mode
            if (!writeRegister(AW9523B_REG_LED_MODE_PORT0, 0x00)) return false;
            if (!writeRegister(AW9523B_REG_LED_MODE_PORT1, 0x00)) return false;
        }
    
        // 4. Initialize output values
        if (!writeRegister(AW9523B_REG_OUTPUT_PORT0, 0xFF)) return false;
        if (!writeRegister(AW9523B_REG_OUTPUT_PORT1, 0xFF)) return false;
    
        return true;
    }

// This function modifies the configuration register for a specific pin. It clears the bit for the pin number. It writes the new value to the register.
void AW9523B::setPinOutput(uint8_t pin) {
    uint8_t reg = (pin < 8) ? AW9523B_REG_CONFIG_PORT0 : AW9523B_REG_CONFIG_PORT1;
    uint8_t bit = pin % 8;
    uint8_t current;

    if (readRegister(reg, &current)) {
        current &= ~(1 << bit);
        writeRegister(reg, current);
    }
}

// This function modifies the configuration register for a specific pin. It sets the bit for the pin number. It writes the new value to the register.
void AW9523B::setPinInput(uint8_t pin) {
    uint8_t reg = (pin < 8) ? AW9523B_REG_CONFIG_PORT0 : AW9523B_REG_CONFIG_PORT1;
    uint8_t bit = pin % 8;
    uint8_t current;

    if (readRegister(reg, &current)) {
        current |= (1 << bit);
        writeRegister(reg, current);
    }
}

// This function controls the output state of a pin. It reads the current register value. It sets or clears the bit based on the input value. It writes the resulting value to the register.
void AW9523B::writePin(uint8_t pin, bool value) {
    uint8_t reg = (pin < 8) ? AW9523B_REG_OUTPUT_PORT0 : AW9523B_REG_OUTPUT_PORT1;
    uint8_t bit = pin % 8;
    uint8_t current;

    if (readRegister(reg, &current)) {
        if (value) {
            current |= (1 << bit);
        } else {
            current &= ~(1 << bit);
        }
        writeRegister(reg, current);
    }
}

// This function reads the input register for a specific pin. It returns the value of the bit corresponding to the pin. It returns false if the register read fails.
bool AW9523B::readPin(uint8_t pin) {
    uint8_t reg = (pin < 8) ? AW9523B_REG_INPUT_PORT0 : AW9523B_REG_INPUT_PORT1;
    uint8_t bit = pin % 8;
    uint8_t value;

    if (readRegister(reg, &value)) {
        return value & (1 << bit);
    }
    return false;
}

const uint8_t dimRegisterMap[16] = {
    AW9523B_REG_DIM0,  AW9523B_REG_DIM1,  AW9523B_REG_DIM2,  AW9523B_REG_DIM3,
    AW9523B_REG_DIM4,  AW9523B_REG_DIM5,  AW9523B_REG_DIM6,  AW9523B_REG_DIM7,
    AW9523B_REG_DIM8,  AW9523B_REG_DIM9,  AW9523B_REG_DIM10, AW9523B_REG_DIM11,
    AW9523B_REG_DIM12, AW9523B_REG_DIM13, AW9523B_REG_DIM14, AW9523B_REG_DIM15
};

void AW9523B::setLEDBrightness(uint8_t pin, uint8_t brightness) {
    if (pin > 15) return;
    writeRegister(dimRegisterMap[pin], brightness);

    uint8_t val;
    readRegister(dimRegisterMap[pin], &val);
}

bool AW9523B::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_write_blocking(_i2c, _address, buf, 2, false) == 2;
}

bool AW9523B::readRegister(uint8_t reg, uint8_t* value) {
    if (i2c_write_blocking(_i2c, _address, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(_i2c, _address, value, 1, false) == 1;
}

void AW9523B::enableLED(uint8_t pin) {
    if (pin > 15) return;

    uint8_t led_mode_reg = (pin < 8) ? AW9523B_REG_LED_MODE_PORT0 : AW9523B_REG_LED_MODE_PORT1;
    uint8_t bit = pin % 8;
    uint8_t current;

    if (readRegister(led_mode_reg, &current)) {
        current &= ~(1 << bit);  // Set the bit for this pin
        bool ok = writeRegister(led_mode_reg, current);
    }
}

void AW9523B::forceLEDTestMode() {
    writeRegister(0x11, 0x10);
    uint8_t ctl;
    readRegister(0x11, &ctl);
}