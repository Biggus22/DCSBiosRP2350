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
#include "internal/ws2812.h"
#include "internal/X27_stepper.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

#define NUM_LEDS 20          // Total number of SK6812 LEDs

WS2812 externalLeds(pio0, 0, 3, false); // Global WS2812 object for external NeoPixels on pin 3

uart_inst_t *rs485_uart = uart0;

// --- CD4021 Shift Register ---
// CLK=GPIO4, CS=/PL=GPIO5, DATA=GPIO6
// Switches have external pull-ups, active-LOW when pressed
const uint8_t SR_CLK_PIN = 4;
const uint8_t SR_CS_PIN = 5;
const uint8_t SR_DATA_PIN = 6;
const unsigned long SR_DEBOUNCE_MS = 30;

static uint8_t srStableState = 0;
static uint8_t srPendingState = 0;
static unsigned long srLastDebounceTime = 0;
static unsigned long srLastPrintTime = 0;

// Bit mapping (D0 = bit 0 after reversal):
// D0: Pitot Heat Off
// D1: Pitot Heat On
// D2: Right Generator On
// D3: Right Generator External Power
// D4: Left Generator On
// D5: Left Generator External Power
// D6: Rain Removal On (TODO: DCS-BIOS control name TBD)
// D7: Rain Removal Off (TODO: DCS-BIOS control name TBD)

static uint8_t readShiftRegister() {
    uint8_t val = 0;
    // CS HIGH = parallel load (CD4021 requires min 10us load pulse)
    gpio_put(SR_CS_PIN, 1);
    sleep_us(10);
    // CS LOW = shift mode
    gpio_put(SR_CS_PIN, 0);
    sleep_us(2);

    // CD4021 shifts out D7 first. We read 8 bits and reverse so D0 is in bit 0.
    for (int i = 7; i >= 0; i--) {
        int bit = gpio_get(SR_DATA_PIN);
        if (bit) val |= (1 << i);
        gpio_put(SR_CLK_PIN, 1);
        sleep_us(2);
        gpio_put(SR_CLK_PIN, 0);
        sleep_us(2);
    }
    return val;
}

static void pollShiftRegisterInputs() {
    uint8_t cur = readShiftRegister();
    unsigned long now = to_ms_since_boot(get_absolute_time());

    if (cur != srPendingState) {
        srPendingState = cur;
        srLastDebounceTime = now;
    }

    if ((now - srLastDebounceTime) >= SR_DEBOUNCE_MS && srPendingState != srStableState) {
        uint8_t changed = srPendingState ^ srStableState;
        uint8_t prevState = srStableState;
        srStableState = srPendingState;

        // Pitot Heat (2-pos toggle): D0=off, D1=on
        if (changed & 0x03) {
            if (!(srStableState & 0x01)) {
                // D0 active (off)
                DcsBios::tryToSendDcsBiosMessage("PLT_PITOT_HEAT", "0");
            } else if (!(srStableState & 0x02)) {
                // D1 active (on)
                DcsBios::tryToSendDcsBiosMessage("PLT_PITOT_HEAT", "1");
            }
        }

        // Right Generator (3-pos maintained): D2=on, D3=ext power
        if (changed & 0x0C) {
            bool d2Active = !(srStableState & 0x04);
            bool d3Active = !(srStableState & 0x08);
            if (d2Active) {
                DcsBios::tryToSendDcsBiosMessage("PLT_ELECTRICS_GENERATOR_R", "0");
            } else if (d3Active) {
                DcsBios::tryToSendDcsBiosMessage("PLT_ELECTRICS_GENERATOR_R", "2");
            } else {
                DcsBios::tryToSendDcsBiosMessage("PLT_ELECTRICS_GENERATOR_R", "1");
            }
        }

        // Left Generator (3-pos maintained): D4=on, D5=ext power
        if (changed & 0x30) {
            bool d4Active = !(srStableState & 0x10);
            bool d5Active = !(srStableState & 0x20);
            if (d4Active) {
                DcsBios::tryToSendDcsBiosMessage("PLT_ELECTRICS_GENERATOR_L", "0");
            } else if (d5Active) {
                DcsBios::tryToSendDcsBiosMessage("PLT_ELECTRICS_GENERATOR_L", "2");
            } else {
                DcsBios::tryToSendDcsBiosMessage("PLT_ELECTRICS_GENERATOR_L", "1");
            }
        }

        // Rain Removal (TODO): D6=on, D7=off
        // Placeholder for when DCS-BIOS control names are available
        if (changed & 0xC0) {
            // TODO: Implement rain removal DCS-BIOS messages once control names are known
            // D6 active (0) -> rain removal on
            // D7 active (0) -> rain removal off
        }
    }

    // Periodic heartbeat: print stable state every 2 seconds
    if (now - srLastPrintTime >= 2000) {
        srLastPrintTime = now;
    }
}

// --- X27 Stepper Motor (Cabin Pressure) ---
constexpr x27_gpio_config_t CABIN_PRESSURE_MOTOR_CFG = {
    .pin_coil1_a = 10,
    .pin_coil1_b = 11,
    .pin_coil2_a = 12,
    .pin_coil2_b = 13,
};
constexpr uint HALL_SENSOR_PIN = 9;
constexpr x27_step_mode_t CP_MOTOR_STEP_MODE = X27_MODE_HALF_STEP;
constexpr uint32_t CP_MOTOR_STEP_DELAY_US = 2200;

x27_motor_t g_cabinPressureMotor = {};
bool g_cabinPressureReady = false;

void onPltHydraulicPc1Change(unsigned int newValue) {
    if (!g_cabinPressureReady) return;
    x27_set_position(&g_cabinPressureMotor,
        (int32_t)(((uint32_t)newValue * (uint32_t)X27_MAX_POSITION) / 65535u));
}
DcsBios::IntegerBuffer pltHydraulicPc1Buffer(F_4E_PLT_HYDRAULIC_PC1, onPltHydraulicPc1Change);

// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);
    
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(intensity, 0, 0, 0)); // Red for F-4
    }
    externalLeds.show();
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);


// DCS-BIOS callback function for F-14 console lighting (red)
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t brightness = 0;
    
    if (consoleBrightness <= 8) {
        uint8_t percentages[9] = {0, 13, 25, 38, 50, 63, 75, 88, 100};
        brightness = (percentages[consoleBrightness] * 255) / 100;
    } else {
        brightness = (uint8_t)((consoleBrightness * 255) / 65535);
    }
    
    for (int i = 0; i < NUM_LEDS; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0)); // Red for F-14
    }
    externalLeds.show();
}

// Declare the IntegerBuffer for F-14 console lighting
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);



int main()
{
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready

    adc_init();
    adc_gpio_init(28);

    // Initialize CD4021 shift register pins
    gpio_init(SR_CLK_PIN);
    gpio_set_dir(SR_CLK_PIN, GPIO_OUT);
    gpio_put(SR_CLK_PIN, 0);

    gpio_init(SR_CS_PIN);
    gpio_set_dir(SR_CS_PIN, GPIO_OUT);
    gpio_put(SR_CS_PIN, 0);

    gpio_init(SR_DATA_PIN);
    gpio_set_dir(SR_DATA_PIN, GPIO_IN);
    gpio_pull_up(SR_DATA_PIN);
    printf("CD4021 pins: CLK=%u CS=%u DATA=%u\n", SR_CLK_PIN, SR_CS_PIN, SR_DATA_PIN);

    printf("Initializing WS2812 on GPIO3...\n");
    externalLeds.begin(NUM_LEDS);
    printf("WS2812 init done, flashing green...\n");

    // Power-on Green Flash for 1 second
    for (int i = 0; i < NUM_LEDS; i++)
    {
        externalLeds.setPixel(i, externalLeds.rgbw(0, 201, 0, 0)); // Set all pixels to green (R=0, G=255, B=0, W=0)
    }
    externalLeds.show();
    sleep_ms(5000);

    // Clear LEDs after the flash
    externalLeds.clear();
    externalLeds.show();
    printf("Green flash complete, clearing LEDs\n");

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
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");

    // Initialize cabin pressure stepper motor
    if (x27_init_gpio(&g_cabinPressureMotor, &CABIN_PRESSURE_MOTOR_CFG, CP_MOTOR_STEP_MODE)) {
        // Configure hall sensor for X27 homing (A3341, active-high)
        x27_config_homing_sensor(&g_cabinPressureMotor, HALL_SENSOR_PIN, false, false);
        x27_set_speed(&g_cabinPressureMotor, CP_MOTOR_STEP_DELAY_US);
        printf("Homing cabin pressure motor...\n");
        bool homed = x27_home_with_sensor(&g_cabinPressureMotor, -1, 1300);
        if (homed) {
            printf("Cabin pressure motor homed successfully\n");
        } else {
            printf("Cabin pressure motor homing timed out, setting position to 0\n");
        }
        g_cabinPressureMotor.current_position = 0;
        g_cabinPressureMotor.target_position = 0;
        g_cabinPressureReady = true;
    } else {
        printf("Cabin pressure motor init failed\n");
    }

    sleep_ms(1000); // Show startup message for 1 second
    DcsBios::setup(); // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        pollShiftRegisterInputs();  // Poll CD4021 shift register inputs
        if (g_cabinPressureReady) {
            x27_update(&g_cabinPressureMotor);
        }
        sleep_us(10);
    }
}
