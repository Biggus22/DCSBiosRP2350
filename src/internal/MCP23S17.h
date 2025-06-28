#ifndef MCP23S17_H
#define MCP23S17_H

// Include standard Pico SDK and C++ libraries first
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <map>
#include <vector> // Required for std::vector

// Define default values for SPI pins if not provided
// These can be overridden by defining them before including this header,
// for example, via compiler flags (-DDEFAULT_MCP23S17_SPI_PORT=spi1)
#ifndef DEFAULT_MCP23S17_SPI_PORT
#define DEFAULT_MCP23S17_SPI_PORT spi0
#endif

#ifndef DEFAULT_MCP23S17_MISO_PIN
#define DEFAULT_MCP23S17_MISO_PIN 16
#endif
#ifndef DEFAULT_MCP23S17_CS_PIN
#define DEFAULT_MCP23S17_CS_PIN 17
#endif
#ifndef DEFAULT_MCP23S17_SCK_PIN
#define DEFAULT_MCP23S17_SCK_PIN 18
#endif
#ifndef DEFAULT_MCP23S17_MOSI_PIN
#define DEFAULT_MCP23S17_MOSI_PIN 19
#endif

// MCP23S17 Register Addresses
#define MCP23S17_IODIRA   0x00
#define MCP23S17_IPOL     0x02
#define MCP23S17_GPINTEN  0x04
#define MCP23S17_DEFVAL   0x06
#define MCP23S17_INTCON   0x08
#define MCP23S17_IOCON    0x0A // IOCON.BANK=0 for sequential addresses
#define MCP23S17_GPPU     0x0C
#define MCP23S17_INTF     0x0E
#define MCP23S17_INTCAP   0x10
#define MCP23S17_GPIO     0x12
#define MCP23S17_OLAT     0x14

// Direction defines for pinMode
#define INPUT             0x1
#define OUTPUT            0x0
#define INPUT_PULLUP      0x2

// Value defines for digitalRead/Write
#define LOW               0x0
#define HIGH              0x1

class MCP23S17; // Forward declaration
class MCP23S17_InterruptManager; // Forward declaration of the friend class

// Define the type for the interrupt callback function
// The callback will receive a pointer to the MCP23S17 instance that triggered it,
// the pin that changed, and the new value of that pin.
typedef void (*InterruptCallback)(MCP23S17* source_expander, uint8_t pin, uint8_t value);


class MCP23S17 {
public:
    // Constructor with explicit SPI pins and interrupt pin
    MCP23S17(spi_inst_t *spi, uint8_t cs_pin, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t address, int interrupt_gpio_pin = -1);

    // Constructor with default SPI pins and explicit interrupt pin
    MCP23S17(spi_inst_t *spi, uint8_t cs_pin, uint8_t address, int interrupt_gpio_pin = -1);

    void begin(); // Initializes SPI and sets up the expander

    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t value);
    uint8_t digitalRead(uint8_t pin);

    void enableInterrupt(uint8_t pin);
    void disableInterrupt(uint8_t pin);
    void setupInterrupts(int gpio_pin); // Configures the Pico GPIO for interrupt
    void setInterruptCallback(InterruptCallback callback); // Set the user-defined callback

    // Add declarations for the missing functions
    void setInterruptPolarity(uint8_t pin, bool activeHigh);
    void setInterruptCompare(uint8_t pin, uint8_t value);


    uint8_t getAddress() const { return _address; }

private:
    spi_inst_t *_spi;
    uint8_t _cs_pin;
    uint8_t _sck_pin;
    uint8_t _mosi_pin;
    uint8_t _miso_pin;
    uint8_t _address;
    int _interrupt_gpio_pin; // The Pico GPIO connected to the INT pin of the MCP23S17

    uint16_t _direction; // Keeps track of pin directions (IODIRA/B)
    uint16_t _output_state; // Keeps track of output register (OLATA/B)
    uint16_t _pullup_state; // Keeps track of pull-up register (GPPUA/B)
    uint16_t _int_enable; // Keeps track of interrupt enable register (GPINTENA/B)

    InterruptCallback _callback_function; // Stores the user-defined callback

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void updateRegisterBit(uint8_t reg, uint8_t pin, bool set);
    uint8_t getBankedRegister(uint8_t reg, uint8_t pin);
    uint8_t getOperationCode(uint8_t reg);

    void clearInterrupts(); // Clears pending interrupts on the MCP23S17
    
    // Declare MCP23S17_InterruptManager as a friend class to allow it to access private members
    friend class MCP23S17_InterruptManager;
};


// Singleton to manage all MCP23S17 instances and their shared interrupt pin
class MCP23S17_InterruptManager {
public:
    static MCP23S17_InterruptManager& getInstance() {
        static MCP23S17_InterruptManager instance;
        return instance;
    }

    void registerExpander(int gpio_pin, MCP23S17* expander);
    void deregisterExpander(MCP23S17* expander);

    // Static callback function for Pico GPIO interrupts - MOVED TO PUBLIC
    static void gpio_callback(uint gpio, uint32_t events);

private:
    MCP23S17_InterruptManager() = default;
    ~MCP23S17_InterruptManager() = default;
    MCP23S17_InterruptManager(const MCP23S17_InterruptManager&) = delete;
    MCP23S17_InterruptManager& operator=(const MCP23S17_InterruptManager&) = delete;

    // Mapping from Pico GPIO pin to a list of MCP23S17 expanders using that pin
    // Make this static to match the definition in the .cpp file
    static std::map<uint, std::vector<MCP23S17*>> _interrupt_mappings;
};


#endif // MCP23S17_H