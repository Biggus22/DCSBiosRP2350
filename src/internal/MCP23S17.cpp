#include "MCP23S17.h"
#include <stdio.h>
#include <vector> // Required for std::vector in the manager
#include <algorithm> // For std::find and std::remove if used
#include "pico/time.h" // For busy_wait_us_32

// Helper function to set/clear a bit in a 16-bit value
static uint16_t setBit(uint16_t value, uint8_t bit, bool set) {
    if (set) {
        return value | (1 << bit);
    } else {
        return value & ~(1 << bit);
    }
}

// Helper function to get a bit from a 16-bit value
static bool getBit(uint16_t value, uint8_t bit) {
    return (value >> bit) & 0x01;
}

// Constructor with all SPI pins and ONE interrupt GPIO pin provided
MCP23S17::MCP23S17(spi_inst_t *spi, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t address, int interrupt_gpio_pin)
    : _spi(spi), _cs_pin(cs_pin), _sck_pin(sck_pin), _mosi_pin(mosi_pin), _miso_pin(miso_pin), _address(address), _interrupt_gpio_pin(interrupt_gpio_pin),
      _direction(0xFFFF), _output_state(0x0000), _pullup_state(0x0000), _int_enable(0x0000), _callback_function(nullptr) {
    // Member initialization list for clarity and direct initialization
}

// Constructor with default SPI pins and ONE interrupt GPIO pin
MCP23S17::MCP23S17(spi_inst_t *spi, uint8_t cs_pin, uint8_t address, int interrupt_gpio_pin)
    : _spi(spi), _cs_pin(cs_pin), _sck_pin(DEFAULT_MCP23S17_SCK_PIN), _mosi_pin(DEFAULT_MCP23S17_MOSI_PIN), _miso_pin(DEFAULT_MCP23S17_MISO_PIN), _address(address), _interrupt_gpio_pin(interrupt_gpio_pin),
      _direction(0xFFFF), _output_state(0x0000), _pullup_state(0x0000), _int_enable(0x0000), _callback_function(nullptr) {
    // Member initialization list
}


void MCP23S17::begin() {
    // SPI initialization
    spi_init(_spi, 1000 * 1000); // Initialize SPI at 1 MHz
    gpio_set_function(_miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(_sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(_mosi_pin, GPIO_FUNC_SPI);

    // Manual CS pin setup
    gpio_init(_cs_pin);
    gpio_set_dir(_cs_pin, GPIO_OUT);
    gpio_put(_cs_pin, HIGH); // Deselect chip

    // Configure IOCON register for sequential operation, disable INT-pin config.
    // BANK=0 (addresses are sequential), SEQOP=0 (increment address pointer),
    // HAEN=1 (hardware address enable), ODR=0 (open-drain output disabled), INTPOL=0 (active-low)
    // MIRROR=0 (INTA and INTB are independent)
    writeRegister(MCP23S17_IOCON, 0b00001000); // HAEN bit set

    // Set all pins to input by default
    writeRegister(MCP23S17_IODIRA, 0xFF);
    writeRegister(MCP23S17_IODIRA + 1, 0xFF);

    // Disable all pull-ups
    writeRegister(MCP23S17_GPPU, 0x00);
    writeRegister(MCP23S17_GPPU + 1, 0x00);

    // Disable all interrupts
    writeRegister(MCP23S17_GPINTEN, 0x00);
    writeRegister(MCP23S17_GPINTEN + 1, 0x00);

    // Clear any pending interrupts
    clearInterrupts();

    // Set internal state to match hardware defaults
    _direction = 0xFFFF; // All inputs
    _output_state = 0x0000; // All outputs low (if they were outputs)
    _pullup_state = 0x0000; // No pull-ups
    _int_enable = 0x0000; // No interrupts enabled

    // If an interrupt GPIO pin is provided, set it up
    if (_interrupt_gpio_pin != -1) {
        setupInterrupts(_interrupt_gpio_pin); // Use the existing method
    }
}

uint8_t MCP23S17::getOperationCode(uint8_t reg) {
    // The datasheet shows the operation code as 0100'A2'A1'A0'R/W
    // Where A2,A1,A0 are hardware address bits, and R/W is Read/Write bit.
    // For write: 0b0100_AAAA_A_0 (0x40 | (_address << 1))
    // For read:  0b0100_AAAA_A_1 (0x40 | (_address << 1) | 0x01)
    return 0x40 | (_address << 1); // Base opcode, R/W bit added later
}

uint8_t MCP23S17::getBankedRegister(uint8_t reg, uint8_t pin) {
    // In BANK=0 mode (which we are using), A and B registers are sequential.
    // So, reg is for Port A, and reg + 1 is for Port B.
    if (pin < 8) {
        return reg; // Port A
    } else {
        return reg + 1; // Port B
    }
}

uint8_t MCP23S17::readRegister(uint8_t reg) {
    uint8_t tx_buf[2];
    uint8_t rx_buf[2]; // Need 2 bytes for read: 1 for status/opcode, 1 for data

    tx_buf[0] = getOperationCode(reg) | 0x01; // Read operation (R/W bit = 1)
    tx_buf[1] = reg; // Register address

    gpio_put(_cs_pin, LOW); // Select chip
    // Send opcode + register, then clock in the data
    spi_write_read_blocking(_spi, tx_buf, rx_buf, 2); 
    gpio_put(_cs_pin, HIGH); // Deselect chip
    busy_wait_us_32(1); // Small delay
    return rx_buf[1]; // The actual data is in the second byte received (after sending register address)
}

void MCP23S17::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t tx_buf[3];

    tx_buf[0] = getOperationCode(reg); // Write operation (R/W bit = 0 implicitly)
    tx_buf[1] = reg;    // Register address
    tx_buf[2] = value;  // Data

    gpio_put(_cs_pin, LOW); // Select chip
    spi_write_blocking(_spi, tx_buf, 3); // Send opcode + register + data
    gpio_put(_cs_pin, HIGH); // Deselect chip
    busy_wait_us_32(1); // Small delay
}

void MCP23S17::updateRegisterBit(uint8_t reg, uint8_t pin, bool set) {
    uint16_t* target_state;
    // Determine which internal state variable to update based on the register
    if (reg == MCP23S17_IODIRA) {
        target_state = &_direction;
    } else if (reg == MCP23S17_GPPU) {
        target_state = &_pullup_state;
    } else if (reg == MCP23S17_OLAT) {
        target_state = &_output_state;
    } else if (reg == MCP23S17_GPINTEN) {
        target_state = &_int_enable;
    } else {
        // Handle unexpected register, or add more cases as needed
        printf("Error: Attempted to update unsupported register in updateRegisterBit.\n");
        return;
    }

    *target_state = setBit(*target_state, pin, set);

    uint8_t reg_addr = getBankedRegister(reg, pin);
    uint8_t value_to_write;
    if (pin < 8) { // Port A
        value_to_write = (uint8_t)(*target_state & 0xFF);
    } else { // Port B
        value_to_write = (uint8_t)((*target_state >> 8) & 0xFF);
    }
    writeRegister(reg_addr, value_to_write);
}

void MCP23S17::pinMode(uint8_t pin, uint8_t mode) {
    if (pin >= 16) return; // Invalid pin

    if (mode == INPUT) {
        updateRegisterBit(MCP23S17_IODIRA, pin, true);  // Set bit for input
        updateRegisterBit(MCP23S17_GPPU, pin, false); // Disable pull-up
    } else if (mode == OUTPUT) {
        updateRegisterBit(MCP23S17_IODIRA, pin, false); // Clear bit for output
        // When setting to output, also ensure the output latch is set (e.g., to LOW)
        digitalWrite(pin, LOW); 
    } else if (mode == INPUT_PULLUP) {
        updateRegisterBit(MCP23S17_IODIRA, pin, true);  // Set bit for input
        updateRegisterBit(MCP23S17_GPPU, pin, true);  // Enable pull-up
    }
}

void MCP23S17::digitalWrite(uint8_t pin, uint8_t value) {
    if (pin >= 16) return; // Invalid pin

    // Ensure the pin is configured as an output before writing
    // This is crucial to avoid unintended behavior if trying to write to an input pin.
    if (getBit(_direction, pin)) { // If it's an input (bit is 1 in _direction)
        pinMode(pin, OUTPUT); // Force it to output mode using the correct enum
    }

    updateRegisterBit(MCP23S17_OLAT, pin, (value == HIGH)); // Update output latch register
}

uint8_t MCP23S17::digitalRead(uint8_t pin) {
    if (pin >= 16) return 0; // Invalid pin

    uint8_t reg_addr = getBankedRegister(MCP23S17_GPIO, pin);
    uint8_t value = readRegister(reg_addr);
    
    // Return the specific bit value for the pin (0 or 1)
    return (value >> (pin % 8)) & 0x01;
}

void MCP23S17::enableInterrupt(uint8_t pin) {
    if (pin >= 16) return;
    // Ensure the pin is configured as an input with pull-up if it's not already
    // This makes sense for interrupts from switches/encoders
    pinMode(pin, INPUT_PULLUP);

    // Enable interrupt on change for the pin
    updateRegisterBit(MCP23S17_GPINTEN, pin, true);
    // Configure interrupt control register to compare against previous value (INTCON=0)
    // This means an interrupt is generated on any change from the last read value.
    updateRegisterBit(MCP23S17_INTCON, pin, false);
    clearInterrupts(); // Clear any pending interrupts after configuring
}

void MCP23S17::disableInterrupt(uint8_t pin) {
    if (pin >= 16) return;
    updateRegisterBit(MCP23S17_GPINTEN, pin, false); // Disable interrupt on change
    clearInterrupts(); // Clear any pending interrupts
}

// Missing function definition 1: setInterruptPolarity
void MCP23S17::setInterruptPolarity(uint8_t pin, bool activeHigh) {
    // This function typically sets the INTPOL bit in the IOCON register.
    // The INTPOL bit affects both INTA and INTB outputs globally for the chip.
    // If activeHigh is true, INTPOL bit is set (interrupt active high).
    // If activeHigh is false, INTPOL bit is cleared (interrupt active low).
    
    // Read current IOCON value
    uint8_t iocon_val = readRegister(MCP23S17_IOCON);
    
    if (activeHigh) {
        iocon_val = setBit(iocon_val, 1, true); // Set INTPOL bit (Bit 1 of IOCON)
    } else {
        iocon_val = setBit(iocon_val, 1, false); // Clear INTPOL bit
    }
    writeRegister(MCP23S17_IOCON, iocon_val);
}

// Missing function definition 2: setInterruptCompare
void MCP23S17::setInterruptCompare(uint8_t pin, uint8_t value) {
    // This function sets the reference value for interrupt comparison (DEFVAL register)
    // when INTCON bit for that pin is set to 1 (interrupt on value different from DEFVAL).
    // If INTCON bit is 0 (default), interrupt is on any change from previous pin state.
    
    // Set the DEFVAL bit for the specified pin
    uint8_t defval_reg = readRegister(getBankedRegister(MCP23S17_DEFVAL, pin));
    defval_reg = setBit(defval_reg, pin % 8, (value == HIGH));
    writeRegister(getBankedRegister(MCP23S17_DEFVAL, pin), defval_reg);

    // Also, ensure the INTCON bit for this pin is set to 1 if you want to compare against DEFVAL
    updateRegisterBit(MCP23S17_INTCON, pin, true);
}


void MCP23S17::setupInterrupts(int gpio_pin) {
    if (gpio_pin < 0 || gpio_pin >= 30) return; // Invalid Pico GPIO

    _interrupt_gpio_pin = gpio_pin; // Store the Pico GPIO pin used for INT
    MCP23S17_InterruptManager::getInstance().registerExpander(gpio_pin, this);
}

void MCP23S17::setInterruptCallback(InterruptCallback callback) {
    _callback_function = callback;
}

void MCP23S17::clearInterrupts() {
    // Reading INTCAP registers clears the interrupts
    readRegister(MCP23S17_INTCAP); // Reads INTCAPA
    readRegister(MCP23S17_INTCAP + 1); // Reads INTCAPB
}

// *** MCP23S17_InterruptManager Implementation ***

// Define the static member here
std::map<uint, std::vector<MCP23S17*>> MCP23S17_InterruptManager::_interrupt_mappings;

void MCP23S17_InterruptManager::registerExpander(int gpio_pin, MCP23S17* expander) {
    // Check if Pico GPIO interrupt is already configured for this pin
    // If _interrupt_mappings[gpio_pin] is empty, it means this is the first expander
    // registering for this specific Pico GPIO interrupt.
    if (_interrupt_mappings[gpio_pin].empty()) { // Use .empty() instead of .find()
        // Configure the Pico GPIO as input with pull-up and set up the callback
        gpio_init(gpio_pin);
        gpio_set_dir(gpio_pin, GPIO_IN);
        gpio_pull_up(gpio_pin); // Assuming MCP23S17 INT is active low and needs pull-up
        gpio_set_irq_enabled_with_callback(gpio_pin, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
        printf("Pico GPIO %d configured for interrupt with callback.\n", gpio_pin);
    }
    // Add the expander to the list associated with this Pico GPIO
    _interrupt_mappings[gpio_pin].push_back(expander);
    printf("MCP23S17 (0x%X) registered to Pico GPIO %d interrupt.\n", expander->getAddress(), gpio_pin);
}

void MCP23S17_InterruptManager::deregisterExpander(MCP23S17* expander) {
    if (expander->_interrupt_gpio_pin >= 0) {
        auto& expander_list = _interrupt_mappings[expander->_interrupt_gpio_pin];
        // Use std::remove-erase idiom to safely remove element
        expander_list.erase(std::remove(expander_list.begin(), expander_list.end(), expander), expander_list.end());

        // If no more expanders on this Pico GPIO, disable its interrupt
        if (expander_list.empty()) {
            gpio_set_irq_enabled(expander->_interrupt_gpio_pin, GPIO_IRQ_EDGE_FALL, false);
            _interrupt_mappings.erase(expander->_interrupt_gpio_pin);
            printf("Pico GPIO %d interrupt disabled.\n", expander->_interrupt_gpio_pin);
        }
    }
}

void MCP23S17_InterruptManager::gpio_callback(uint gpio, uint32_t events) {
    // This callback is triggered when the Pico's GPIO (connected to MCP23S17 INT) goes low.
    // It iterates through all MCP23S17s registered to this GPIO and processes their interrupts.
    auto& instance = getInstance();
    if (instance._interrupt_mappings.count(gpio)) {
        for (MCP23S17* expander : instance._interrupt_mappings[gpio]) {
            // Read INTCAP registers to determine which pin caused the interrupt and clear it
            // Reading INTCAP also clears the interrupt flags on the MCP23S17
            uint16_t intf_val_a = expander->readRegister(MCP23S17_INTF); // Reads INTF (interrupt flag)
            uint16_t intf_val_b = expander->readRegister(MCP23S17_INTF + 1); // For Port B

            uint16_t intcap_val_a = expander->readRegister(MCP23S17_INTCAP); // Reads INTCAP (capture)
            uint16_t intcap_val_b = expander->readRegister(MCP23S17_INTCAP + 1); // For Port B

            // Combine into 16-bit values
            uint16_t int_flags = (intf_val_b << 8) | intf_val_a;
            uint16_t captured_values = (intcap_val_b << 8) | intcap_val_a;

            // Call the user's registered callback for each pin that caused the interrupt
            if (expander->_callback_function) { // Check if a callback is set
                for (uint8_t i = 0; i < 16; ++i) {
                    if (getBit(int_flags, i)) { // If this pin caused an interrupt
                        uint8_t pin_value = getBit(captured_values, i);
                        expander->_callback_function(expander, i, pin_value);
                    }
                }
            }
        }
    }
}