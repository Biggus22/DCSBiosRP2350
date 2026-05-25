#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdio.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "DcsBios.h"
#include "internal/BoardMode.h"
#include "internal/FoxConfig.h"
#include "internal/heartbeat.h"
#include "internal/rs485.h"
#include "internal/X27_stepper.h"

namespace {

constexpr uint32_t RS485_BAUD = 250000;
constexpr unsigned char RS485_SLAVE_ADDRESS = 0x01;

// Board pin assumptions from current X27 DRV8833 board bring-up:
//   Motor A: verified on this PCB
//   Motor B: verified on this PCB
//   Motor C: default mapping for third DRV8833 stage; adjust if your board differs
constexpr x27_gpio_config_t MOTOR_A_CFG = {
    .pin_coil1_a = 3,
    .pin_coil1_b = 4,
    .pin_coil2_a = 5,
    .pin_coil2_b = 6,
};

constexpr x27_gpio_config_t MOTOR_B_CFG = {
    .pin_coil1_a = 9,
    .pin_coil1_b = 10,
    .pin_coil2_a = 11,
    .pin_coil2_b = 12,
};

constexpr x27_gpio_config_t MOTOR_C_CFG = {
    .pin_coil1_a = 14,
    .pin_coil1_b = 15,
    .pin_coil2_a = 26,
    .pin_coil2_b = 27,
};

constexpr x27_step_mode_t MOTOR_STEP_MODE = X27_MODE_HALF_STEP;
constexpr uint32_t MOTOR_STEP_DELAY_US = 2200;
constexpr bool RUN_MECHANICAL_HOME_ON_BOOT = true;
constexpr signed char HOME_DIRECTION = -1;
constexpr uint32_t HOME_MAX_STEPS = 1300;

// Flip per-motor direction here if a gauge runs backward.
constexpr bool MOTOR_A_DIRECTION_INVERTED = false;
constexpr bool MOTOR_B_DIRECTION_INVERTED = false;
constexpr bool MOTOR_C_DIRECTION_INVERTED = false;

x27_motor_t g_motorA = {};
x27_motor_t g_motorB = {};
x27_motor_t g_motorC = {};
bool g_motorAReady = false;
bool g_motorBReady = false;
bool g_motorCReady = false;

uart_inst_t* rs485_uart = uart0;

int32_t rawToX27Steps(unsigned int raw) {
    return (int32_t)(((uint32_t)raw * (uint32_t)X27_MAX_POSITION) / 65535u);
}

bool initMotor(x27_motor_t* motor,
               bool* ready,
               const x27_gpio_config_t* cfg,
               const char* name,
               bool directionInverted) {
    if (!x27_init_gpio(motor, cfg, MOTOR_STEP_MODE)) {
        printf("%s init failed\n", name);
        *ready = false;
        return false;
    }

    x27_set_speed(motor, MOTOR_STEP_DELAY_US);
    x27_set_direction_inverted(motor, directionInverted);

    if (RUN_MECHANICAL_HOME_ON_BOOT) {
        x27_home_to_stop(motor, HOME_DIRECTION, HOME_MAX_STEPS);
    }

    motor->current_position = 0;
    motor->target_position = 0;

    printf("%s ready on pins %u %u %u %u\n",
           name,
           cfg->pin_coil1_a,
           cfg->pin_coil1_b,
           cfg->pin_coil2_a,
           cfg->pin_coil2_b);

    *ready = true;
    return true;
}

void updateMotors() {
    if (g_motorAReady) x27_update(&g_motorA);
    if (g_motorBReady) x27_update(&g_motorB);
    if (g_motorCReady) x27_update(&g_motorC);
}

}  // namespace

void onPltVviNeedleChange(unsigned int newValue) {
    if (!g_motorAReady) return;
    x27_set_position(&g_motorA, rawToX27Steps(newValue));
}
DcsBios::IntegerBuffer pltVviNeedleBuffer(0x2bc4, 0xffff, 0, onPltVviNeedleChange);

void onPltRadarAltNeedleChange(unsigned int newValue) {
    if (!g_motorBReady) return;
    x27_set_position(&g_motorB, rawToX27Steps(newValue));
}
DcsBios::IntegerBuffer pltRadarAltNeedleBuffer(0x2a8a, 0xffff, 0, onPltRadarAltNeedleChange);

void onPltO2FlowChange(unsigned int newValue) {
    if (!g_motorCReady) return;
    x27_set_position(&g_motorC, rawToX27Steps(newValue));
}
DcsBios::IntegerBuffer pltO2FlowBuffer(0x2b32, 0xffff, 0, onPltO2FlowChange);

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(1500);

    DcsBios::BoardMode board = DcsBios::determineBoardMode(RS485_SLAVE_ADDRESS);
    DcsBios::currentBoardMode = board;

    if (board.mode != DcsBios::BoardModeType::SLAVE) {
        printf("Invalid board mode for address 0x%X\n", RS485_SLAVE_ADDRESS);
        while (true) {
            DcsBios::updateHeartbeat();
            sleep_ms(100);
        }
    }

    printf("Starting RS485 slave address 0x%X\n", RS485_SLAVE_ADDRESS);
    printf("RS485 UART0: TX=%d RX=%d EN=%d baud=%lu\n",
           UART0_TX,
           UART0_RX,
           RS485_EN,
           (unsigned long)RS485_BAUD);

    DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, RS485_BAUD);
    multicore_launch_core1(DcsBios::core1_task);
    DcsBios::setup();

    initMotor(&g_motorA, &g_motorAReady, &MOTOR_A_CFG, "Motor A (VVI)", MOTOR_A_DIRECTION_INVERTED);
    initMotor(&g_motorB, &g_motorBReady, &MOTOR_B_CFG, "Motor B (Radar Alt)", MOTOR_B_DIRECTION_INVERTED);
    initMotor(&g_motorC, &g_motorCReady, &MOTOR_C_CFG, "Motor C (O2 Flow)", MOTOR_C_DIRECTION_INVERTED);

    while (true) {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        updateMotors();
        sleep_us(20);
    }

    return 0;
}
