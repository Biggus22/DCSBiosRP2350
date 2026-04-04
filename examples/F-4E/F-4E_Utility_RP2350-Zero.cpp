#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "pico/multicore.h"
#include "../../src/DcsBios.h"
#include "../../src/internal/FoxConfig.h"
#include "../../src/internal/Leds.h"
#include "../../src/internal/Switches.h"
#include "../../src/internal/heartbeat.h"
#include "../../src/internal/DeviceAddress.h"
#include "../../src/internal/BoardMode.h"
#include "../../src/internal/rs485.h"
#include "../../src/internal/ws2812.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"



#define NUM_LEDS 17          // Total number of SK6812 LEDs
#define ANTI_SKID_LED_INDEX (NUM_LEDS - 1)

WS2812 externalLeds(pio0, 0, 0, false); // Global WS2812 object for external NeoPixels on pin 0

static bool g_antiSkidInop = false;

static uint16_t pulseWidthUsToPwmLevel(uint32_t us) {
    // 20 ms servo period with wrap 39062 -> approximately 1.95 ticks per microsecond.
    uint32_t level = (us * 39062) / 20000;
    if (level > 39062) {
        level = 39062;
    }
    return (uint16_t)level;
}

static void initServoPwmPin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.f);
    pwm_config_set_wrap(&config, 39062);
    pwm_init(slice, &config, true);
}

static void setServoPulseUs(uint pin, uint16_t pulseWidthUs) {
    pwm_set_gpio_level(pin, pulseWidthUsToPwmLevel(pulseWidthUs));
}

static void runServoSweepIfEnabled(bool enabled, const uint8_t* pins, size_t pinCount,
                                   uint16_t minUs, uint16_t maxUs, uint16_t stepUs,
                                   uint16_t stepDelayMs) {
    if (!enabled || pins == nullptr || pinCount == 0 || stepUs == 0 || minUs >= maxUs) {
        return;
    }

    for (size_t i = 0; i < pinCount; i++) {
        initServoPwmPin(pins[i]);
        setServoPulseUs(pins[i], minUs);
    }
    sleep_ms(250);

    for (uint16_t us = minUs; us <= maxUs; us = (uint16_t)(us + stepUs)) {
        for (size_t i = 0; i < pinCount; i++) {
            setServoPulseUs(pins[i], us);
        }
        sleep_ms(stepDelayMs);
        if ((uint16_t)(us + stepUs) < us) {
            break;
        }
    }

    for (int us = (int)maxUs; us >= (int)minUs; us -= (int)stepUs) {
        for (size_t i = 0; i < pinCount; i++) {
            setServoPulseUs(pins[i], (uint16_t)us);
        }
        sleep_ms(stepDelayMs);
    }

    uint16_t midUs = (uint16_t)(minUs + ((maxUs - minUs) / 2));
    for (size_t i = 0; i < pinCount; i++) {
        setServoPulseUs(pins[i], midUs);
    }
    sleep_ms(150);
}

static void applyAntiSkidLed() {
    if (g_antiSkidInop) {
        // Amber warning indicator
        externalLeds.setPixel(ANTI_SKID_LED_INDEX, externalLeds.rgbw(255, 96, 0, 0));
    } else {
        // Off when not in warning state
        externalLeds.setPixel(ANTI_SKID_LED_INDEX, externalLeds.rgbw(0, 0, 0, 0));
    }
    externalLeds.show();
}

static void runStartupSweepIfEnabled(bool enabled, uint32_t color, uint16_t stepDelayMs) {
    if (!enabled) {
        return;
    }

    externalLeds.clear();
    for (int i = 0; i < ANTI_SKID_LED_INDEX; i++) {
        externalLeds.setPixel(i, color);
        externalLeds.show();
        sleep_ms(stepDelayMs);
        externalLeds.setPixel(i, externalLeds.rgbw(0, 0, 0, 0));
    }
    externalLeds.show();
}

//uart_inst_t *rs485_uart = uart0;


// DCS-BIOS callback function for F-4E console lighting (red)
void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    uint8_t intensity = (uint8_t)((consoleBrightness * 255) / 65535);

    for (int i = 0; i < ANTI_SKID_LED_INDEX; i++) {
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
    
    for (int i = 0; i < ANTI_SKID_LED_INDEX; i++) {
        externalLeds.setPixel(i, externalLeds.rgbw(brightness, 0, 0, 0)); // Red for F-14
    }
    externalLeds.show();
}

// Declare the IntegerBuffer for F-14 console lighting
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);

// Anti-skid INOP warning for the last LED in the string
void onPltGearAntiSkidInopChange(unsigned int newValue) {
    g_antiSkidInop = (newValue != 0);
    applyAntiSkidLed();
}
DcsBios::IntegerBuffer pltGearAntiSkidInopBuffer(0x2ac0, 0x2000, 13, onPltGearAntiSkidInopChange);

// Servo output for oxygen flow valve - using original Arduino DCS-BIOS values
DcsBios::ServoOutput pltO2Flow(0x2b32, 9, 620, 2800);
DcsBios::ServoOutput pltO2Liters(0x2b36, 11, 544, 2400);
DcsBios::ServoOutput pltO2Pressure(0x2b34, 13, 544, 2400);
const uint8_t o2ServoPins[3] = {9, 11, 13};

// O2 mixture switch (2-position) on GPIO pins 12 and 10
const uint8_t pltO2MixturePins[2] = {12, 10};
DcsBios::SyncingSwitchMultiPosT<POLL_EVERY_TIME, 2> pltO2Mixture("PLT_O2_MIXTURE", pltO2MixturePins,
    F_4E_PLT_O2_MIXTURE, 50);

// O2 supply switch (2-position) on GPIO pin 14
DcsBios::SyncingSwitch2PosT<POLL_EVERY_TIME> pltO2Supply("PLT_O2_SUPPLY", 14,
    F_4E_PLT_O2_SUPPLY);

int main()
{
    stdio_init_all();                      // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED); // Initialize heartbeat LED
    sleep_ms(2000);                        // Wait for USB CDC to be ready

    externalLeds.begin(NUM_LEDS);

    // Set true to run an initial servo sweep before entering normal DCS-BIOS control.
    const bool enableStartupServoSweep = true;
    runServoSweepIfEnabled(enableStartupServoSweep, o2ServoPins, 3, 620, 2400, 20, 8);

    // Set true to run the startup sweep before the static power-on flash.
    const bool enableStartupSweep = true;
    runStartupSweepIfEnabled(enableStartupSweep, externalLeds.rgbw(0, 201, 0, 0), 40);

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

    // Explicitly reference the function inside the DcsBios namespace
    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");


    DcsBios::setup(); // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        sleep_us(10);
    }
}