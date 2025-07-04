#ifndef PCF8575_H
#define PCF8575_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>

/**
 * PCF8575 16-bit I2C I/O Expander Library
 * 
 * Features:
 * - 16 I/O pins (P00-P07, P10-P17)
 * - I2C interface
 * - Quasi-bidirectional I/Os
 * - Interrupt output capability
 * - Operating voltage: 2.5V to 5.5V
 * - Default I2C address: 0x20 (can be configured via A0, A1, A2 pins)
 */

class PCF8575 {
private:
    i2c_inst_t* i2c_port_;
    uint8_t i2c_address_;
    uint16_t pin_states_;        // Current pin states (cached)
    uint16_t pin_directions_;    // Pin directions (1 = input, 0 = output)
    bool initialized_;

    /**
     * Write 16-bit value to PCF8575
     * @param value 16-bit value to write
     * @return true if successful, false otherwise
     */
    bool writeRegister(uint16_t value);

    /**
     * Read 16-bit value from PCF8575
     * @param value pointer to store read value
     * @return true if successful, false otherwise
     */
    bool readRegister(uint16_t* value);

public:
    /**
     * Constructor
     * @param i2c_port I2C port instance (i2c0 or i2c1)
     * @param address I2C address (default 0x20)
     */
    PCF8575(i2c_inst_t* i2c_port, uint8_t address = 0x20);

    /**
     * Initialize the PCF8575
     * Must be called after I2C is initialized
     * @return true if successful, false otherwise
     */
    bool begin();

    /**
     * Check if PCF8575 is connected and responding
     * @return true if connected, false otherwise
     */
    bool isConnected();

    /**
     * Set pin mode
     * @param pin Pin number (0-15)
     * @param mode Pin mode (GPIO_IN or GPIO_OUT)
     * @return true if successful, false otherwise
     */
    bool pinMode(uint8_t pin, uint8_t mode);

    /**
     * Write to a digital pin
     * @param pin Pin number (0-15)
     * @param value Pin value (0 or 1)
     * @return true if successful, false otherwise
     */
    bool digitalWrite(uint8_t pin, uint8_t value);

    /**
     * Read from a digital pin
     * @param pin Pin number (0-15)
     * @return Pin value (0 or 1), or -1 on error
     */
    int digitalRead(uint8_t pin);

    /**
     * Write to multiple pins at once
     * @param value 16-bit value representing all pins
     * @return true if successful, false otherwise
     */
    bool writePort(uint16_t value);

    /**
     * Read from all pins at once
     * @param value pointer to store 16-bit value
     * @return true if successful, false otherwise
     */
    bool readPort(uint16_t* value);

    /**
     * Set multiple pins as input/output
     * @param directions 16-bit value (1 = input, 0 = output)
     * @return true if successful, false otherwise
     */
    bool setPortDirections(uint16_t directions);

    /**
     * Get current pin directions
     * @return 16-bit value representing pin directions
     */
    uint16_t getPortDirections() const;

    /**
     * Enable/disable internal pull-up on input pins
     * Note: PCF8575 has weak internal pull-ups on all pins
     * This function sets pins high to enable pull-ups
     * @param pin Pin number (0-15)
     * @param enable true to enable pull-up, false to disable
     * @return true if successful, false otherwise
     */
    bool pullUp(uint8_t pin, bool enable = true);

    /**
     * Get I2C address
     * @return I2C address
     */
    uint8_t getAddress() const;

    /**
     * Get I2C port
     * @return I2C port instance
     */
    i2c_inst_t* getI2CPort() const;

    /**
     * Reset all pins to input mode with pull-ups enabled
     * @return true if successful, false otherwise
     */
    bool reset();

    // Pin mapping constants for easier use
    static const uint8_t P00 = 0;
    static const uint8_t P01 = 1;
    static const uint8_t P02 = 2;
    static const uint8_t P03 = 3;
    static const uint8_t P04 = 4;
    static const uint8_t P05 = 5;
    static const uint8_t P06 = 6;
    static const uint8_t P07 = 7;
    static const uint8_t P10 = 8;
    static const uint8_t P11 = 9;
    static const uint8_t P12 = 10;
    static const uint8_t P13 = 11;
    static const uint8_t P14 = 12;
    static const uint8_t P15 = 13;
    static const uint8_t P16 = 14;
    static const uint8_t P17 = 15;

    // Alternative pin naming
    static const uint8_t PIN_0 = 0;
    static const uint8_t PIN_1 = 1;
    static const uint8_t PIN_2 = 2;
    static const uint8_t PIN_3 = 3;
    static const uint8_t PIN_4 = 4;
    static const uint8_t PIN_5 = 5;
    static const uint8_t PIN_6 = 6;
    static const uint8_t PIN_7 = 7;
    static const uint8_t PIN_8 = 8;
    static const uint8_t PIN_9 = 9;
    static const uint8_t PIN_10 = 10;
    static const uint8_t PIN_11 = 11;
    static const uint8_t PIN_12 = 12;
    static const uint8_t PIN_13 = 13;
    static const uint8_t PIN_14 = 14;
    static const uint8_t PIN_15 = 15;
};

#endif // PCF8575_H