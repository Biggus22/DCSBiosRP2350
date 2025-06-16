#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

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


// Pin definitions
#define CONSOLE_LIGHTING_LED 25   // Console LED on GPIO 25
#define O2_PSI_SERVO_PIN 15         // Servo on GPIO 16 (slice 0)
#define CONSOLE_LIGHTING_POT_PIN 26 // ADC0 input for potentiometer (same as CONSOLE_LIGHTING_INPUT)
#define PITOT_HEAT_ON_PIN 14 // GPIO for pitot heat on/off


// Servo timing constants (SG90)
#define O2_PSI_MAX 167931 // Calibrated so 26869 = ~80 PSI
#define O2_PSI_MIN 0
#define O2_PSI_GAUGE_MAX 500.0f
#define O2_PSI_SERVO_MIN_ANGLE 0.0f     // Angle for 5000 PSI (needle left)
#define O2_PSI_SERVO_MAX_ANGLE 180.0f   // Angle for 0 PSI (needle right)
#define O2_PSI_SERVO_MIN_US 150         // min pulse (µs)
#define O2_PSI_SERVO_MAX_US 3400        // max pulse (µs)
#define O2_PSI_SERVO_PWM_DIVIDER 125.0f // gives 1µs resolution at 125MHz clock
#define O2_PWM_TOP 20000                // 20ms period / 1µs resolution = 20000
#define CONSOLE_DIMMING_DEADZONE 10     // Deadzone for brightness changes

unsigned int lastBrightness = 0;

// Function to set servo angle from 0–180 degrees
void set_servo_angle(float angle)
{
    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > 180.0f)
        angle = 180.0f;

    float pulse_width_us = O2_PSI_SERVO_MIN_US +
                           (angle / 180.0f) * (O2_PSI_SERVO_MAX_US - O2_PSI_SERVO_MIN_US);

    // Convert µs pulse width to PWM level
    float duty_cycle = pulse_width_us / 20000.0f; // 20ms period
    uint16_t level = (uint16_t)(duty_cycle * O2_PWM_TOP);

    pwm_set_gpio_level(O2_PSI_SERVO_PIN, level);
}

void onPltO2PressureChange(unsigned int newValue)
{
    // Clamp the value to max for safety
    if (newValue > O2_PSI_MAX)
        newValue = O2_PSI_MAX;

    // Map DCS value to PSI
    float psi = (float)newValue / O2_PSI_MAX * O2_PSI_GAUGE_MAX;

    // Reverse map PSI to servo angle (right = 0 PSI, left = 500 PSI)
    float angle = O2_PSI_SERVO_MAX_ANGLE - (psi / O2_PSI_GAUGE_MAX * (O2_PSI_SERVO_MAX_ANGLE - O2_PSI_SERVO_MIN_ANGLE));

    // Send to servo
    set_servo_angle(angle);
}
DcsBios::IntegerBuffer pltO2PressureBuffer(0x2b34, 0xffff, 0, onPltO2PressureChange);

// DCS-BIOS input callback for console brightness LED
void onPltIntLightConsoleBrightnessChange(unsigned int newValue)
{
    if (abs((int)newValue - (int)lastBrightness) >= CONSOLE_DIMMING_DEADZONE)
    {
        lastBrightness = newValue;
        uint16_t pwmVal = newValue >> 8; // Map 0–65535 to 0–255
        pwm_set_gpio_level(CONSOLE_LIGHTING_LED, pwmVal);
    }
}
DcsBios::IntegerBuffer pltIntLightConsoleBrightnessBuffer(0x2d6e, 0xffff, 0, onPltIntLightConsoleBrightnessChange);

// DCS-BIOS potentiometer input from ADC0 to DCS
DcsBios::PotentiometerEWMA<POLL_EVERY_TIME, 1024> pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS", CONSOLE_LIGHTING_POT_PIN);

// If the correct class is Switch2Pos, ensure the header is included; otherwise, use the correct class name as defined in your DcsBios library.
// Example fix if the class is named Switch2PosT (common in some DCS-BIOS versions):
DcsBios::Switch2Pos pitotHeatSwitch("PITOT_HEAT", PITOT_HEAT_ON_PIN);


void sweep_servo()
{
    // Sweep from right (0 PSI / 180°) to left (500 PSI / 0°)
    for (float angle = 180.0f; angle >= 0.0f; angle -= 1.0f)
    {
        set_servo_angle(angle);
        sleep_ms(10); // Slow enough to see
    }

    sleep_ms(300); // Pause at end

    // Sweep back to 90° (idle midpoint)
    for (float angle = 0.0f; angle <= 90.0f; angle += 1.0f)
    {
        set_servo_angle(angle);
        sleep_ms(10);
    }

    sleep_ms(300);
}

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

    // Launch core1 for DCS-BIOS RS485/USB task
    multicore_launch_core1(DcsBios::core1_task);

    // Initialize onboard LED PWM
    gpio_set_function(CONSOLE_LIGHTING_LED, GPIO_FUNC_PWM);
    uint led_slice = pwm_gpio_to_slice_num(CONSOLE_LIGHTING_LED);
    pwm_set_wrap(led_slice, 255); // 8-bit PWM for LED
    pwm_set_enabled(led_slice, true);

    // Init ADC for analog pot (used by DCSBios::PotentiometerEWMA)
    adc_init();
    adc_gpio_init(CONSOLE_LIGHTING_POT_PIN); // ADC0

    // Initialize servo PWM (GPIO 16 → slice 0)
    gpio_set_function(O2_PSI_SERVO_PIN, GPIO_FUNC_PWM);
    uint servo_slice = pwm_gpio_to_slice_num(O2_PSI_SERVO_PIN);
    pwm_config servo_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&servo_cfg, O2_PSI_SERVO_PWM_DIVIDER); // 1µs resolution
    pwm_config_set_wrap(&servo_cfg, O2_PWM_TOP);                 // 20ms period (50Hz)
    pwm_init(servo_slice, &servo_cfg, true);

    // Perform gauge sweep
    sweep_servo();

    // Set initial servo position
    // set_servo_angle(90); // Midpoint

    // DCS-BIOS init
    DcsBios::setup();
    printf("DCS-BIOS setup complete\n");

    while (true)
    {
        DcsBios::loop();            // Main DCS-BIOS handler
        DcsBios::updateHeartbeat(); // Blink status LED (if connected)
        sleep_us(10);               // Slight delay
    }
}
