#include "pcf8575.h"

PCF8575::PCF8575(i2c_inst_t* i2c_port, uint8_t address) 
    : i2c_port_(i2c_port), i2c_address_(address), pin_states_(0xFFFF), 
      pin_directions_(0xFFFF), initialized_(false) {
    // Default all pins to input with pull-ups
}

bool PCF8575::writeRegister(uint16_t value) {
    if (!initialized_) return false;
    
    uint8_t data[2];
    data[0] = value & 0xFF;         // Low byte
    data[1] = (value >> 8) & 0xFF;  // High byte
    
    int result = i2c_write_blocking(i2c_port_, i2c_address_, data, 2, false);
    return result == 2;
}

bool PCF8575::readRegister(uint16_t* value) {
    if (!initialized_ || !value) return false;
    
    uint8_t data[2];
    int result = i2c_read_blocking(i2c_port_, i2c_address_, data, 2, false);
    
    if (result == 2) {
        *value = data[0] | (data[1] << 8);
        return true;
    }
    return false;
}

bool PCF8575::begin() {
    if (initialized_) return true;
    
    // Test communication
    if (!isConnected()) {
        return false;
    }
    
    // Initialize all pins as inputs with pull-ups
    pin_states_ = 0xFFFF;
    pin_directions_ = 0xFFFF;
    
    bool success = writeRegister(pin_states_);
    if (success) {
        initialized_ = true;
    }
    
    return success;
}

bool PCF8575::isConnected() {
    uint8_t dummy = 0;
    int result = i2c_write_blocking(i2c_port_, i2c_address_, &dummy, 0, false);
    return result >= 0;
}

bool PCF8575::pinMode(uint8_t pin, uint8_t mode) {
    if (!initialized_ || pin > 15) return false;
    
    uint16_t mask = 1 << pin;
    
    if (mode == GPIO_IN) {
        pin_directions_ |= mask;   // Set bit for input
        pin_states_ |= mask;       // Set high for pull-up
    } else {
        pin_directions_ &= ~mask;  // Clear bit for output
        // Keep current output state
    }
    
    return writeRegister(pin_states_);
}

bool PCF8575::digitalWrite(uint8_t pin, uint8_t value) {
    if (!initialized_ || pin > 15) return false;
    
    // Check if pin is configured as output
    uint16_t mask = 1 << pin;
    if (pin_directions_ & mask) {
        // Pin is configured as input, cannot write
        return false;
    }
    
    if (value) {
        pin_states_ |= mask;
    } else {
        pin_states_ &= ~mask;
    }
    
    return writeRegister(pin_states_);
}

int PCF8575::digitalRead(uint8_t pin) {
    if (!initialized_ || pin > 15) return -1;
    
    uint16_t current_state;
    if (!readRegister(&current_state)) {
        return -1;
    }
    
    return (current_state >> pin) & 1;
}

bool PCF8575::writePort(uint16_t value) {
    if (!initialized_) return false;
    
    // Only update output pins, preserve input pins
    uint16_t output_mask = ~pin_directions_;  // Invert to get output pins
    uint16_t input_mask = pin_directions_;    // Input pins
    
    pin_states_ = (value & output_mask) | (pin_states_ & input_mask);
    
    return writeRegister(pin_states_);
}

bool PCF8575::readPort(uint16_t* value) {
    if (!initialized_ || !value) return false;
    
    return readRegister(value);
}

bool PCF8575::setPortDirections(uint16_t directions) {
    if (!initialized_) return false;
    
    pin_directions_ = directions;
    
    // Set input pins high for pull-ups
    pin_states_ |= pin_directions_;
    
    return writeRegister(pin_states_);
}

uint16_t PCF8575::getPortDirections() const {
    return pin_directions_;
}

bool PCF8575::pullUp(uint8_t pin, bool enable) {
    if (!initialized_ || pin > 15) return false;
    
    uint16_t mask = 1 << pin;
    
    // Check if pin is configured as input
    if (!(pin_directions_ & mask)) {
        // Pin is configured as output, cannot set pull-up
        return false;
    }
    
    if (enable) {
        pin_states_ |= mask;
    } else {
        pin_states_ &= ~mask;
    }
    
    return writeRegister(pin_states_);
}

uint8_t PCF8575::getAddress() const {
    return i2c_address_;
}

i2c_inst_t* PCF8575::getI2CPort() const {
    return i2c_port_;
}

bool PCF8575::reset() {
    if (!initialized_) return false;
    
    // Reset all pins to input with pull-ups
    pin_directions_ = 0xFFFF;
    pin_states_ = 0xFFFF;
    
    return writeRegister(pin_states_);
}