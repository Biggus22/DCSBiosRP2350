#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "internal/X27_Stepper.h"

//uart_inst_t *rs485_uart = uart0;

// Direct use of X27_Stepper library for STI6606 control
x27_motor_t pltO2FlowMotor;
bool motor_initialized = false;
bool test_mode = true;  // Default to enabled for testing
uint32_t last_test_update = 0;
const uint32_t test_interval = 5000; // 5ms between steps in test mode
float test_angle = 0.0f;
float test_direction = 1.0f; // 1.0 for forward, -1.0 for backward
const float test_range = 180.0f; // Sweep from 0 to 180 degrees

// Initialize the motor
void init_pltO2FlowMotor() {
    x27_vid6606_config_t vid_cfg = {
        .pin_step = 12,      // GPIO12 - F(scx)D (Step)
        .pin_dir = 13,       // GPIO13 - CW/CCWD (Direction)
        .pin_reset = 26      // GPIO26 - Reset
    };

    x27_init_vid6606(&pltO2FlowMotor, &vid_cfg, X27_MODE_MICRO_STEP);
    x27_home(&pltO2FlowMotor);
    motor_initialized = true;
}

// Update the motor (call this in main loop)
void update_pltO2FlowMotor() {
    if (motor_initialized) {
        if (test_mode) {
            // Test mode: sweep back and forth slowly
            uint32_t current_time = time_us_32();
            if (current_time - last_test_update > test_interval) {
                // Update test angle
                test_angle += test_direction * 0.5f; // Move 0.5 degrees per update

                // Change direction at limits
                if (test_angle >= test_range) {
                    test_direction = -1.0f;
                    test_angle = test_range;
                } else if (test_angle <= 0.0f) {
                    test_direction = 1.0f;
                    test_angle = 0.0f;
                }

                // Set the motor to the test angle
                x27_set_angle(&pltO2FlowMotor, test_angle);

                last_test_update = current_time;
            }
        }

        x27_update(&pltO2FlowMotor);
    }
}

// Set the motor position
void set_pltO2FlowMotorPosition(int position) {
    if (motor_initialized) {
        test_mode = false; // Disable test mode when setting position manually
        x27_set_position(&pltO2FlowMotor, position);
    }
}

// Disable test mode (to be called when you want to disable testing)
void disable_test_mode() {
    test_mode = false;
}

// Enable test mode
void enable_test_mode() {
    test_mode = true;
}




// The motor is initialized in main function

// DCS-BIOS callback function for oxygen flow gauge
void onPltO2FlowChange(unsigned int value) {
    // Map DCS-BIOS value (0-65535) to X27 stepper position (0-X27_MAX_POSITION)
    int stepperPos = (value * X27_MAX_POSITION) / 65535;
    set_pltO2FlowMotorPosition(stepperPos);
}
DcsBios::IntegerBuffer pltO2FlowBuffer(0x2b32, 0xffff, 0, onPltO2FlowChange); // Correct address for pltO2Flow

// DCS-BIOS callback function for instrument panel backlighting (commented out)
/*
void onPltIntLightInstrumentPanelChange(unsigned int newValue) {
    uint8_t intensity = (uint8_t)((newValue * 255) / 65535);
    // Control backlight directly - completely separate from stepper
    gpio_put(15, intensity > 0 ? 1 : 0); // GPIO15 for backlight
}
DcsBios::IntegerBuffer pltIntLightInstrumentPanelBuffer(0x2d88, 0xffff, 0, onPltIntLightInstrumentPanelChange);
*/

// DCS-BIOS callback function for F-4E console lighting (red) - for backlighting (separate from WS2812 control)
void onPltIntLightConsoleForBacklightChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    gpio_put(15, intensity > 0 ? 1 : 0); // GPIO15 for backlight
}
DcsBios::IntegerBuffer pltIntLightConsoleForBacklightBuffer(0x2D8A, 0xffff, 0, onPltIntLightConsoleForBacklightChange);

int main()
{
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready

    uint8_t boardAddress = 0xF;

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
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
        printf("STANDALONE USB MODE\n");
        break;
    case DcsBios::BoardModeType::RS485_TERMINAL:
        printf("RS485 TERMINAL MODE\n");
        break;
    default:
        printf("INVALID ADDRESS\n");
        break;
    }
    DcsBios::currentBoardMode = board;
    //DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Initialize the stepper motor
    init_pltO2FlowMotor();
    printf("Stepper motor initialized!\n");

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");


    DcsBios::setup(); // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        update_pltO2FlowMotor();    // Update stepper motor
        sleep_us(100);              // Shorter delay to allow more frequent updates
    }
}