#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/MCP23S17.h"

// SPI Configuration
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// MCP23S17 Configuration
#define MCP23S17_ADDRESS 0x20  // Hardware address (A2=A1=A0=0)

// MCP23S17 instance
MCP23S17 ioExpander(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, MCP23S17_ADDRESS);

// Custom switch class for MCP23S17
class MCP23S17Switch2Pos {
private:
    const char* control_name;
    MCP23S17* expander;
    uint8_t pin;
    bool last_state;
    
public:
    MCP23S17Switch2Pos(const char* control, MCP23S17* exp, uint8_t pin_num) 
        : control_name(control), expander(exp), pin(pin_num), last_state(false) {}
    
    void pollInput() {
        bool current_state = expander->digitalRead(pin);
        if (current_state != last_state) {
            last_state = current_state;
            DcsBios::sendDcsBiosMessage(control_name, current_state ? "1" : "0");
        }
    }
};

// Custom rotary encoder class for MCP23S17
class MCP23S17RotaryEncoder {
private:
    const char* control_name;
    const char* dec_action;
    const char* inc_action;
    MCP23S17* expander;
    uint8_t pin_a;
    uint8_t pin_b;
    bool last_a;
    bool last_b;
    
public:
    MCP23S17RotaryEncoder(const char* control, const char* dec_step, const char* inc_step, MCP23S17* exp, uint8_t pin_a_num, uint8_t pin_b_num)
        : control_name(control), dec_action(dec_step), inc_action(inc_step), expander(exp), pin_a(pin_a_num), pin_b(pin_b_num), last_a(false), last_b(false) {}
    
    void pollInput() {
        bool current_a = expander->digitalRead(pin_a);
        bool current_b = expander->digitalRead(pin_b);
        
        // Check for state change on pin A
        if (current_a != last_a) {
            // Pin A changed, check direction
            if (current_a) {
                // Rising edge on A
                if (current_b) {
                    // B is high, counter-clockwise
                    DcsBios::sendDcsBiosMessage(control_name, dec_action);
                } else {
                    // B is low, clockwise
                    DcsBios::sendDcsBiosMessage(control_name, inc_action);
                }
            }
        }
        
        last_a = current_a;
        last_b = current_b;
    }
};

// Create switch on MCP23S17 GPA0 (pin 0)
MCP23S17Switch2Pos pltPitotHeat("PLT_PITOT_HEAT", &ioExpander, 0);

// Create rotary encoder on MCP23S17 GPA1 and GPA2 (pins 1 and 2)
MCP23S17RotaryEncoder pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", "-3200", "+3200", &ioExpander, 1, 2);



int main()
{
    stdio_init_all();
    sleep_ms(1000); // Allow USB to settle

    // Heartbeat init
    DcsBios::initHeartbeat(HEARTBEAT_LED);

    // Determine board mode for USB/RS485
    uint8_t boardAddress = 0xF;
    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    DcsBios::currentBoardMode = board;

    printf("Board address: 0x%X\n", boardAddress);
    switch (board.mode)
    {
    case DcsBios::BoardModeType::HOST:
        printf("HOST MODE\n");
        break;
    case DcsBios::BoardModeType::SLAVE:
        printf("SLAVE MODE\n");
        break;
    case DcsBios::BoardModeType::USB_ONLY:
        printf("USB MODE\n");
        break;
    case DcsBios::BoardModeType::RS485_TERMINAL:
        printf("RS485 TERMINAL\n");
        break;
    default:
        printf("INVALID ADDRESS\n");
        break;
    }

    // Initialize MCP23S17
    printf("Initializing MCP23S17...\n");
    ioExpander.begin();
    
    // Configure GPA0 as input with pull-up for the switch
    ioExpander.pinMode(0, INPUT_PULLUP);
    // Configure GPA1 and GPA2 as inputs with pull-up for the rotary encoder
    ioExpander.pinMode(1, INPUT_PULLUP);
    ioExpander.pinMode(2, INPUT_PULLUP);
    printf("MCP23S17 initialized, GPA0-2 configured as inputs with pull-up\n");

    // Launch core1 for DCS-BIOS RS485/USB task
    multicore_launch_core1(DcsBios::core1_task);

    // DCS-BIOS init
    DcsBios::setup();
    printf("DCS-BIOS setup complete\n");

    while (true)
    {
        DcsBios::loop();              // Main DCS-BIOS handler
        pltPitotHeat.pollInput();    // Poll the MCP23S17 switch
        pltIntLightConsoleBrightness.pollInput();   // Poll the MCP23S17 rotary encoder
        DcsBios::updateHeartbeat();   // Blink status LED (if connected)
        sleep_us(10);                 // Slight delay
    }
}