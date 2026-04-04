#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include "pico/time.h"
#include <string.h>
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
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "internal/X27_stepper.h"

uart_inst_t *rs485_uart = uart0; // Control UART in main

// VVI stepper setup (X27 via GPIO driver)
static x27_motor_t vvi_motor;
static const x27_gpio_config_t VVI_CFG = {
    .pin_coil1_a = 2,
    .pin_coil1_b = 3,
    .pin_coil2_a = 4,
    .pin_coil2_b = 5,
};

static inline int32_t vvi_raw_to_steps(uint16_t raw) {
    // Map 0..65535 to 0..X27_MAX_POSITION (945). Center will sit near 472.
    return (int32_t)((raw * X27_MAX_POSITION) / 65535u);
}

void onPltVviNeedleChange(unsigned int newValue) {
    // DCS BIOS delivers 0..65535, no shift. Move the stepper accordingly.
    x27_set_position(&vvi_motor, vvi_raw_to_steps((uint16_t)newValue));
}

DcsBios::IntegerBuffer pltVviNeedleBuffer(0x2bc4, 0xffff, 0, onPltVviNeedleChange);

void onPltIntLightVviChange(unsigned int newValue) {
    // Map 0..65535 to 0..255 PWM brightness for the external LED string on GPIO15
    uint8_t level = (uint8_t)((newValue * 255u) / 65535u);
    pwm_set_gpio_level(15, level);
}
DcsBios::IntegerBuffer pltIntLightVviBuffer(0x2d7a, 0xffff, 0, onPltIntLightVviChange);

int main()
{
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(2000);

    // Initialize external LED group on GPIO15 using PWM for brightness control
    const uint LED_PIN_EXT = 15;
    gpio_set_function(LED_PIN_EXT, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(LED_PIN_EXT);
    pwm_set_wrap(slice, 255);
    pwm_set_clkdiv(slice, 64.0f); // slower clock for smoother low-level dimming
    pwm_set_enabled(slice, true);
    pwm_set_gpio_level(LED_PIN_EXT, 0);

    uint8_t boardAddress = 0xF;

    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);

    DcsBios::currentBoardMode = board;
    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);

    multicore_launch_core1(DcsBios::core1_task);

    DcsBios::setup();

    // Initialize VVI stepper
    x27_init_gpio(&vvi_motor, &VVI_CFG, X27_MODE_HALF_STEP); // smoother motion
    x27_set_speed(&vvi_motor, 2500); // 2.5ms per step
    x27_set_position(&vvi_motor, vvi_raw_to_steps(0)); // start at bottom

    while (true)
    {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        x27_update(&vvi_motor);
        sleep_us(10);
    }
}