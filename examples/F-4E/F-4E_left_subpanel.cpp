#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "DcsBios.h"
#include "internal/FoxConfig.h"
#include "internal/Leds.h"
#include "internal/heartbeat.h"
#include "internal/BoardMode.h"
#include "internal/rs485.h"
#include "internal/ws2812.h"
#include "internal/X27_stepper.h"
#include "hardware/pwm.h"

// Left subpanel hardware map:
// RS485: TX=GPIO0, RX=GPIO1, DE/RE=GPIO2
// PWM LEDs: GPIO4, GPIO5
// X27 + MX1508 #1: coils GPIO6-9, sensor GPIO10 (opto)
// X27 + MX1508 #2: coils GPIO11-14, sensor GPIO15 (hall)
// X27 + MX1508 #3: coils GPIO16-19, sensor GPIO20 (hall)
// Servos: GPIO21-25
// RGB strip data: GPIO33, count=9

static constexpr uint8_t RGB_PIN = 33;
static constexpr int NUM_RGB_LEDS = 9;
static constexpr uint8_t PWM_LED_PINS[] = {4, 5};
static constexpr uint8_t SERVO_PINS[] = {21, 22, 23, 24, 25};
static constexpr size_t NUM_SERVOS = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);
static constexpr int SERVO_MIN_US = 1000;
static constexpr int SERVO_MAX_US = 2000;
static constexpr int SERVO_STEP_US = 8;

static constexpr int32_t MOTOR_SWEEP_MIN = 20;
static constexpr int32_t MOTOR_SWEEP_MAX = 320;
static constexpr uint32_t MOTOR_STEP_DELAY_US = 1000;
static constexpr uint32_t MOTOR_HOME_MAX_STEPS = 2200;
static constexpr int32_t X27_STEPS_PER_REV_LOCAL = X27_MAX_POSITION;
static constexpr bool DIAG_SKIP_BLOCKING_STARTUP = true;

struct ServoChannel {
    uint pin;
    int pulse_us;
    int delta_us;
};

struct MotorChannel {
    const char* name;
    x27_motor_t motor;
    x27_gpio_config_t cfg;
    uint sensor_pin;
    bool sensor_active_high;
    bool use_sensor_homing;
    bool connected;
    bool controlled_by_dcs;
    int32_t min_pos;
    int32_t max_pos;
    bool target_high;
    absolute_time_t next_target_switch;
};

WS2812 panelRgb(pio0, 0, RGB_PIN, false);
uart_inst_t *rs485_uart = uart0;

static ServoChannel servos[NUM_SERVOS];
static MotorChannel motors[] = {
    {"MOTOR_A", {}, {6, 7, 8, 9}, 10, false, true, true, true, MOTOR_SWEEP_MIN, MOTOR_SWEEP_MAX, true, at_the_end_of_time},
    {"MOTOR_B", {}, {11, 12, 13, 14}, 15, false, true, true, true, MOTOR_SWEEP_MIN, MOTOR_SWEEP_MAX, true, at_the_end_of_time},
    {"MOTOR_C", {}, {16, 17, 18, 19}, 20, false, false, false, false, MOTOR_SWEEP_MIN, MOTOR_SWEEP_MAX, true, at_the_end_of_time},
};

static MotorChannel* motorA = &motors[0];
static MotorChannel* motorB = &motors[1];

struct VviCalibrationPoint {
    float value;
    float angle_deg;
};

static constexpr VviCalibrationPoint VVI_POS_CAL[] = {
    {0.0f, 270.0f},
    {1.0f, 341.0f},
    {2.0f, 19.0f},
    {4.0f, 54.5f},
    {6.0f, 78.7f},
};

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float wrapAngle360(float angle_deg) {
    while (angle_deg < 0.0f) angle_deg += 360.0f;
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    return angle_deg;
}

static float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

static float interpolateVviPositiveAngle(float v_abs) {
    v_abs = clampf(v_abs, 0.0f, 6.0f);
    for (size_t i = 0; i + 1 < sizeof(VVI_POS_CAL) / sizeof(VVI_POS_CAL[0]); ++i) {
        const VviCalibrationPoint &p0 = VVI_POS_CAL[i];
        const VviCalibrationPoint &p1 = VVI_POS_CAL[i + 1];
        if (v_abs >= p0.value && v_abs <= p1.value) {
            float t = (v_abs - p0.value) / (p1.value - p0.value);
            float a0 = p0.angle_deg;
            float a1 = p1.angle_deg;
            if (a1 < a0) {
                a1 += 360.0f;
            }
            return wrapAngle360(lerp(a0, a1, t));
        }
    }
    return VVI_POS_CAL[sizeof(VVI_POS_CAL) / sizeof(VVI_POS_CAL[0]) - 1].angle_deg;
}

static int32_t angleToX27Position(float angle_deg) {
    float relative = angle_deg - 270.0f;
    relative = wrapAngle360(relative);
    // Panel mechanics are mirrored; invert direction while keeping 270 deg as zero.
    relative = wrapAngle360(360.0f - relative);
    // Never command the full-rotation endpoint step; it can look like wrap past 360.
    int32_t steps = (int32_t)((relative / 360.0f) * (float)X27_STEPS_PER_REV_LOCAL + 0.5f);
    if (steps < 0) steps = 0;
    if (steps > X27_MAX_POSITION) steps = X27_MAX_POSITION - 1;
    return steps;
}

void onPltVviNeedleChange(unsigned int newValue) {
    if (!motorA->motor.initialized || !motorA->connected) {
        return;
    }

    // Map 0..65535 onto approximately -6..+6 vertical velocity scale.
    float normalized = ((float)newValue / 65535.0f) * 12.0f - 6.0f;
    normalized = clampf(normalized, -6.0f, 6.0f);

    float angle = interpolateVviPositiveAngle(normalized < 0.0f ? -normalized : normalized);
    if (normalized < 0.0f) {
        // Mirror negative values onto lower half of the gauge.
        angle = wrapAngle360(180.0f - angle);
    }

    x27_set_position(&motorA->motor, angleToX27Position(angle));
}
DcsBios::IntegerBuffer pltVviNeedleBuffer(0x2bc4, 0xffff, 0, onPltVviNeedleChange);

void onPltAoaGaugeNeedleChange(unsigned int newValue) {
    if (!motorB->motor.initialized || !motorB->connected) {
        return;
    }

    // Map 0..65535 to AoA gauge scale 0..30.
    float aoa = ((float)newValue / 65535.0f) * 30.0f;
    aoa = clampf(aoa, 0.0f, 30.0f);

    // User calibration points:
    // 0 -> 270 deg (9 o'clock), 10 -> 180 deg (6 o'clock), 20 -> 90 deg (3 o'clock), 30 -> 0 deg.
    float angle = wrapAngle360(270.0f - (aoa / 30.0f) * 270.0f);
    x27_set_position(&motorB->motor, angleToX27Position(angle));
}
DcsBios::IntegerBuffer pltAoaGaugeNeedleBuffer(0x2a94, 0xffff, 0, onPltAoaGaugeNeedleChange);

static volatile uint8_t g_console_brightness = 0;
static bool g_rgb_startup_done = false;

static void setRgbFromConsoleBrightness() {
    // Green console backlight style.
    const uint8_t r = 0;
    const uint8_t g = g_console_brightness;
    const uint8_t b = 0;
    for (int i = 0; i < NUM_RGB_LEDS; ++i) {
        panelRgb.setPixel(i, panelRgb.rgbw(r, g, b, 0));
    }
    panelRgb.show();
}

static void setPwmLedBrightness(uint8_t level) {
    for (uint i = 0; i < sizeof(PWM_LED_PINS); ++i) {
        pwm_set_gpio_level(PWM_LED_PINS[i], level);
    }
}

void onPltIntLightConsoleChange(unsigned int consoleBrightness) {
    g_console_brightness = (uint8_t)((consoleBrightness * 255u) / 65535u);
    setPwmLedBrightness(g_console_brightness);
    if (g_rgb_startup_done) {
        setRgbFromConsoleBrightness();
    }
}
DcsBios::IntegerBuffer pltIntLightConsoleBuffer(F_4E_PLT_INT_LIGHT_CONSOLE, onPltIntLightConsoleChange);

// Kept for cross-aircraft quick tests where F-14 output is available.
void onF14PltIntLightConsoleChange(unsigned int consoleBrightness) {
    onPltIntLightConsoleChange(consoleBrightness);
}
DcsBios::IntegerBuffer f14PltIntLightConsoleBuffer(F_14_PLT_LIGHT_INTENT_CONSOLE, onF14PltIntLightConsoleChange);

static void initPwmLedPins() {
    for (uint i = 0; i < sizeof(PWM_LED_PINS); ++i) {
        uint pin = PWM_LED_PINS[i];
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pin);
        pwm_set_wrap(slice, 255);
        pwm_set_clkdiv(slice, 64.0f);
        pwm_set_enabled(slice, true);
        pwm_set_gpio_level(pin, 0);
    }
}

static void initServos() {
    for (size_t i = 0; i < NUM_SERVOS; ++i) {
        ServoChannel &s = servos[i];
        s.pin = SERVO_PINS[i];
        s.pulse_us = SERVO_MIN_US + (int)i * 80;
        if (s.pulse_us > SERVO_MAX_US) {
            s.pulse_us = SERVO_MAX_US;
        }
        s.delta_us = SERVO_STEP_US;

        gpio_set_function(s.pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(s.pin);
        pwm_set_clkdiv(slice, 125.0f); // 1 MHz PWM clock (1 tick = 1 us)
        pwm_set_wrap(slice, 20000 - 1); // 20 ms period (50 Hz)
        pwm_set_enabled(slice, true);
        pwm_set_gpio_level(s.pin, s.pulse_us);
    }
}

static void updateServos() {
    static absolute_time_t next_update = nil_time;
    if (is_nil_time(next_update)) {
        next_update = make_timeout_time_ms(20);
    }
    if (absolute_time_diff_us(get_absolute_time(), next_update) > 0) {
        return;
    }

    for (size_t i = 0; i < NUM_SERVOS; ++i) {
        ServoChannel &s = servos[i];
        s.pulse_us += s.delta_us;
        if (s.pulse_us >= SERVO_MAX_US) {
            s.pulse_us = SERVO_MAX_US;
            s.delta_us = -SERVO_STEP_US;
        } else if (s.pulse_us <= SERVO_MIN_US) {
            s.pulse_us = SERVO_MIN_US;
            s.delta_us = SERVO_STEP_US;
        }
        pwm_set_gpio_level(s.pin, s.pulse_us);
    }
    next_update = make_timeout_time_ms(20);
}

static void initAndHomeMotors() {
    for (uint i = 0; i < (sizeof(motors) / sizeof(motors[0])); ++i) {
        MotorChannel &m = motors[i];
        if (!m.connected) {
            printf("%s not connected, skipping init\n", m.name);
            continue;
        }

        bool ok = x27_init_gpio(&m.motor, &m.cfg, X27_MODE_FULL_STEP);
        if (!ok) {
            printf("%s init failed\n", m.name);
            continue;
        }
        x27_set_speed(&m.motor, MOTOR_STEP_DELAY_US);

        bool sensor_ok = false;
        if (m.use_sensor_homing) {
            sensor_ok = x27_config_homing_sensor(&m.motor, (int)m.sensor_pin, m.sensor_active_high, true);
        }
        bool homed = false;
        if (m.use_sensor_homing && sensor_ok) {
            homed = x27_home_with_sensor(&m.motor, -1, MOTOR_HOME_MAX_STEPS);
        }
        if (!homed && !m.use_sensor_homing) {
            x27_home_to_stop(&m.motor, -1, 500);
        }

        // Hall/opto trigger is the configured 270-degree (9 o'clock) zero reference.
        if (homed) {
            m.motor.current_position = 0;
            m.motor.target_position = 0;
        }

        m.target_high = true;
        if (!m.controlled_by_dcs) {
            x27_set_position(&m.motor, m.max_pos);
        }
        m.next_target_switch = make_timeout_time_ms(1200);
        printf("%s ready, homed=%d sensor=%d\n", m.name, homed ? 1 : 0, sensor_ok ? 1 : 0);
    }
}

static void updateMotors() {
    absolute_time_t now = get_absolute_time();
    for (uint i = 0; i < (sizeof(motors) / sizeof(motors[0])); ++i) {
        MotorChannel &m = motors[i];
        if (!m.motor.initialized) {
            continue;
        }

        x27_update(&m.motor);

        if (m.controlled_by_dcs) {
            continue;
        }

        if (!x27_is_at_target(&m.motor)) {
            continue;
        }
        if (absolute_time_diff_us(now, m.next_target_switch) > 0) {
            continue;
        }

        m.target_high = !m.target_high;
        x27_set_position(&m.motor, m.target_high ? m.max_pos : m.min_pos);
        m.next_target_switch = make_timeout_time_ms(1400);
    }
}

int main()
{
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(1500);

    panelRgb.begin(NUM_RGB_LEDS);
    initPwmLedPins();
    initServos();
    if (!DIAG_SKIP_BLOCKING_STARTUP) {
        initAndHomeMotors();
    } else {
        // Non-blocking startup path for comms diagnostics.
        for (uint i = 0; i < (sizeof(motors) / sizeof(motors[0])); ++i) {
            MotorChannel &m = motors[i];
            if (!m.connected) continue;
            bool ok = x27_init_gpio(&m.motor, &m.cfg, X27_MODE_FULL_STEP);
            if (ok) {
                x27_set_speed(&m.motor, MOTOR_STEP_DELAY_US);
                m.motor.current_position = 0;
                m.motor.target_position = 0;
            }
        }
        printf("DIAG: blocking startup skipped\n");
    }

    // Green startup indication for a few seconds before normal console-lighting behavior.
    for (int i = 0; i < NUM_RGB_LEDS; ++i) {
        panelRgb.setPixel(i, panelRgb.rgbw(0, 180, 0, 0));
    }
    panelRgb.show();
    setPwmLedBrightness(255);
    if (!DIAG_SKIP_BLOCKING_STARTUP) {
        sleep_ms(3000);
    } else {
        sleep_ms(100);
    }

    g_rgb_startup_done = true;
    setPwmLedBrightness(g_console_brightness);
    setRgbFromConsoleBrightness();

    // USB mode for direct DCS-BIOS over USB serial.
    uint8_t boardAddress = USB_MODE;
    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    DcsBios::currentBoardMode = board;
    if (board.mode != DcsBios::BoardModeType::USB_ONLY) {
        DcsBios::init_rs485_uart(rs485_uart, UART0_TX, UART0_RX, RS485_EN, 250000);
        printf("DCS-BIOS mode: RS485\n");
    } else {
        printf("DCS-BIOS mode: USB_ONLY\n");
    }

    multicore_launch_core1(DcsBios::core1_task);
    DcsBios::setup();

    while (true)
    {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        updateMotors();
        updateServos();
        sleep_us(200);
    }
}