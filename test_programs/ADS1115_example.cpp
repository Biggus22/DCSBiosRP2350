#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

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

// I2C Configuration for ADS1115
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define ADS1115_ADDRESS 0x48

// MCP23S17 Configuration
#define MCP23S17_ADDRESS 0x00  // Hardware address (A2=A1=A0=0)

// Pin definitions
#define CONSOLE_LIGHTING_LED 25   // Console LED on GPIO 25
#define O2_PSI_SERVO_PIN 15         // Servo on GPIO 16 (slice 0)

// ADS1115 Register addresses
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01
#define ADS1115_REG_LO_THRESH 0x02
#define ADS1115_REG_HI_THRESH 0x03

// ADS1115 Configuration values - Updated to 6.144V range
#define ADS1115_CONFIG_OS_SINGLE    (1 << 15)  // Start single conversion
#define ADS1115_CONFIG_MUX_SINGLE_0 (4 << 12)  // AIN0 vs GND
#define ADS1115_CONFIG_PGA_6_144V   (0 << 9)   // +/-6.144V range (updated from 2.048V)
#define ADS1115_CONFIG_MODE_SINGLE  (1 << 8)   // Single-shot mode
#define ADS1115_CONFIG_DR_128SPS    (0 << 5)   // 128 samples per second
#define ADS1115_CONFIG_CMODE_TRAD   (0 << 4)   // Traditional comparator
#define ADS1115_CONFIG_CPOL_ACTVLOW (0 << 3)   // ALERT/RDY pin low when active
#define ADS1115_CONFIG_CLAT_NONLAT  (0 << 2)   // Non-latching comparator
#define ADS1115_CONFIG_CQUE_NONE    (3 << 0)   // Disable comparator

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

// MCP23S17 instance
MCP23S17 ioExpander(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, MCP23S17_ADDRESS);

// ADS1115 Functions
bool ads1115_write_register(uint8_t reg, uint16_t value) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8) & 0xFF;  // MSB
    data[2] = value & 0xFF;         // LSB
    
    int result = i2c_write_blocking(I2C_PORT, ADS1115_ADDRESS, data, 3, false);
    if (result != 3) {
        printf("ADS1115: Write failed, result=%d, expected=3\n", result);
        return false;
    }
    return true;
}

bool ads1115_read_register(uint8_t reg, uint16_t *value) {
    uint8_t reg_addr = reg;
    uint8_t data[2];
    
    // Write register address
    int result = i2c_write_blocking(I2C_PORT, ADS1115_ADDRESS, &reg_addr, 1, true);
    if (result != 1) return false;
    
    // Read register value
    result = i2c_read_blocking(I2C_PORT, ADS1115_ADDRESS, data, 2, false);
    if (result != 2) return false;
    
    *value = (data[0] << 8) | data[1];
    return true;
}

uint16_t ads1115_read_adc() {
    // Configure ADS1115 for single-shot conversion on AIN0 with 6.144V range
    uint16_t config = ADS1115_CONFIG_OS_SINGLE |
                      ADS1115_CONFIG_MUX_SINGLE_0 |
                      ADS1115_CONFIG_PGA_6_144V |       // Updated to 6.144V range
                      ADS1115_CONFIG_MODE_SINGLE |
                      (4 << 5) |                        // 250 SPS for faster conversion
                      ADS1115_CONFIG_CMODE_TRAD |
                      ADS1115_CONFIG_CPOL_ACTVLOW |
                      ADS1115_CONFIG_CLAT_NONLAT |
                      ADS1115_CONFIG_CQUE_NONE;
    
    if (!ads1115_write_register(ADS1115_REG_CONFIG, config)) {
        return 0;
    }
    
    sleep_ms(8); // Reduced wait time for faster conversion (250 SPS = 4ms + margin)
    
    // Read conversion result
    uint16_t result;
    if (!ads1115_read_register(ADS1115_REG_CONVERSION, &result)) {
        return 0;
    }
    
    return result;
}

// Simplified potentiometer class for ADS1115 with updated calibration
class ADS1115Potentiometer {
private:
    const char* control_name;
    uint8_t last_value;
    uint16_t ewma_value;
    bool initialized;
    
public:
    ADS1115Potentiometer(const char* control) 
        : control_name(control), last_value(0), ewma_value(0), initialized(false) {}
    
    void pollInput() {
        static uint32_t last_poll = 0;
        uint32_t now = time_us_32();
        
        // Poll every 20ms
        if (now - last_poll < 20000) return;
        last_poll = now;
        
        uint16_t raw_value = ads1115_read_adc();
        
        // Convert signed 16-bit to positive value
        int16_t signed_result = (int16_t)raw_value;
        if (signed_result < 0) signed_result = 0;
        
        // Initialize EWMA filter
        if (!initialized) {
            ewma_value = signed_result;
            initialized = true;
        } else {
            // EWMA filter: alpha = 0.5 (128/256)
            ewma_value = ((128 * signed_result) + (128 * ewma_value)) >> 8;
        }
        
        // Updated calibration values based on your measurements
        const uint16_t ADC_MIN = 25;     // Your measured minimum
        const uint16_t ADC_MAX = 17000;  // Your measured maximum
        const uint16_t ADC_RANGE = ADC_MAX - ADC_MIN;  // 16975
        
        // Map ADC range to 0-255 output
        uint8_t final_value;
        
        if (ewma_value <= ADC_MIN) {
            final_value = 0;
        } else if (ewma_value >= ADC_MAX) {
            final_value = 255;
        } else {
            // Linear mapping: (input - min) * 255 / range
            uint32_t adjusted = ewma_value - ADC_MIN;
            final_value = (uint8_t)((adjusted * 255) / ADC_RANGE);
        }
        
        // Only send if value changed
        if (final_value != last_value) {
            last_value = final_value;
            char value_str[8];
            snprintf(value_str, sizeof(value_str), "%u", final_value);
            DcsBios::sendDcsBiosMessage(control_name, value_str);
        }
    }
};

// Create simplified ADS1115 potentiometer instance
ADS1115Potentiometer pltIntLightConsoleBrightness("PLT_INT_LIGHT_CONSOLE_BRIGHTNESS");

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
        // Map from 0-255 input to 0-255 PWM (direct mapping now)
        uint16_t pwmVal = newValue > 255 ? 255 : newValue;
        pwm_set_gpio_level(CONSOLE_LIGHTING_LED, pwmVal);
    }
}
DcsBios::IntegerBuffer pltIntLightConsoleBrightnessBuffer(0x2d6e, 0xffff, 0, onPltIntLightConsoleBrightnessChange);

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

// Create pitot heat switch on MCP23S17 GPA0 (pin 0)
MCP23S17Switch2Pos pitotHeatSwitch("PLT_PITOT_HEAT", &ioExpander, 0);

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

    // Initialize I2C for ADS1115
    printf("Initializing I2C on SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz I2C speed
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C initialized for ADS1115 at 400kHz\n");
    
    // Wait for I2C to settle
    sleep_ms(100);
    
    // Test ADS1115 communication
    printf("Testing ADS1115 communication...\n");
    uint16_t test_value;
    if (ads1115_read_register(ADS1115_REG_CONFIG, &test_value)) {
        printf("ADS1115 config register: 0x%04X\n", test_value);
    } else {
        printf("Warning: Failed to communicate with ADS1115\n");
    }

    // Initialize MCP23S17
    ioExpander.begin();
    
    // Configure GPA0 as input with pull-up for the pitot heat switch
    ioExpander.pinMode(0, INPUT_PULLUP);

    // Launch core1 for DCS-BIOS RS485/USB task
    multicore_launch_core1(DcsBios::core1_task);

    // Initialize onboard LED PWM
    gpio_set_function(CONSOLE_LIGHTING_LED, GPIO_FUNC_PWM);
    uint led_slice = pwm_gpio_to_slice_num(CONSOLE_LIGHTING_LED);
    pwm_set_wrap(led_slice, 255); // 8-bit PWM for LED
    pwm_set_enabled(led_slice, true);

    // Initialize servo PWM (GPIO 15 → slice 7 channel B)
    gpio_set_function(O2_PSI_SERVO_PIN, GPIO_FUNC_PWM);
    uint servo_slice = pwm_gpio_to_slice_num(O2_PSI_SERVO_PIN);
    pwm_config servo_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&servo_cfg, O2_PSI_SERVO_PWM_DIVIDER); // 1µs resolution
    pwm_config_set_wrap(&servo_cfg, O2_PWM_TOP);                 // 20ms period (50Hz)
    pwm_init(servo_slice, &servo_cfg, true);

    // Perform gauge sweep
    sweep_servo();

    // DCS-BIOS init
    DcsBios::setup();
    printf("DCS-BIOS setup complete\n");

    while (true)
    {
        DcsBios::loop();                                    // Main DCS-BIOS handler
        pitotHeatSwitch.pollInput();                        // Poll the MCP23S17 switch
        pltIntLightConsoleBrightness.pollInput();          // Poll ADS1115 potentiometer
        DcsBios::updateHeartbeat();                        // Blink status LED (if connected)
        sleep_us(10);                                      // Slight delay
    }
}