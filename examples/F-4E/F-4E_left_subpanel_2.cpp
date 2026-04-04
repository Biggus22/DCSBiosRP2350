#include <stdio.h>
#include <stdint.h>
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
#include "internal/X27_stepper.h"

#define VVI_LIGHT_PWM_PIN 4
#define AOA_LIGHT_PWM_PIN 5
// Motor A: VVI gauge (X27 + MX1508)
#define VVI_COIL_1A_PIN 6
#define VVI_COIL_1B_PIN 7
#define VVI_COIL_2A_PIN 8
#define VVI_COIL_2B_PIN 9
#define VVI_HOME_SENSOR_PIN 10
#define VVI_STEP_DELAY_US 4000
#define VVI_HOME_MAX_STEPS 2200
#define VVI_HOME_STEP_DELAY_US 2800
#define VVI_STEP_MODE X27_MODE_FULL_STEP
#define VVI_POSITION_SCALE_TRIM 1.00f
#define VVI_HOME_SEARCH_SCALE_TRIM 1.00f
// Motor B: AoA gauge (X27 + MX1508)
#define AOA_COIL_1A_PIN 11
#define AOA_COIL_1B_PIN 12
#define AOA_COIL_2A_PIN 13
#define AOA_COIL_2B_PIN 14
#define AOA_HOME_SENSOR_PIN 15
#define AOA_STEP_DELAY_US 2800
#define AOA_HOME_MAX_STEPS 2200
#define AOA_HOME_STEP_DELAY_US 3000
#define AOA_STEP_MODE X27_MODE_FULL_STEP
#define AOA_POSITION_SCALE_TRIM 1.00f
#define AOA_HOME_SEARCH_SCALE_TRIM 1.00f
#define AOA_DIRECTION_INVERTED false
#define STARTUP_GREEN_LEVEL 201
#define STARTUP_FLASH_DURATION_MS 5000
#define STARTUP_FLASH_INTERVAL_MS 250
#define GAUGE_SWEEP_TIMEOUT_MS 7000
#define STARTUP_SWEEP_STEP_DELAY_US 4000
#define STARTUP_SWEEP_DWELL_MS 600

uart_inst_t* rs485_uart = uart0; // Control UART in main

static uint vvi_light_pwm_slice = 0;
static uint vvi_light_pwm_chan = 0;
static uint aoa_light_pwm_slice = 0;
static uint aoa_light_pwm_chan = 0;
static unsigned char vviLightLevel = 0;
static unsigned char aoaLightLevel = 0;

static float modePositionScale(x27_step_mode_t mode) {
    switch (mode) {
        case X27_MODE_HALF_STEP:
            return 2.0f;
        case X27_MODE_FULL_STEP:
        case X27_MODE_MICRO_STEP:
        default:
            return 1.0f;
    }
}

static float vviPositionScale() {
    return modePositionScale(VVI_STEP_MODE) * VVI_POSITION_SCALE_TRIM;
}

static float aoaPositionScale() {
    float modeScale = 1.0f;
    switch (AOA_STEP_MODE) {
        case X27_MODE_HALF_STEP:
        case X27_MODE_MICRO_STEP:
            modeScale = 1.5f;
            break;
        case X27_MODE_FULL_STEP:
        default:
            modeScale = 1.0f;
            break;
    }
    return modeScale * AOA_POSITION_SCALE_TRIM;
}

static uint32_t modeAdjustedMaxSteps(uint32_t baseSteps, x27_step_mode_t mode) {
    float scaled = (float)baseSteps * modePositionScale(mode);
    uint32_t adjusted = (uint32_t)(scaled + 0.5f);
    if (adjusted < baseSteps) adjusted = baseSteps;
    return adjusted;
}

static uint32_t vviHomeSearchMaxSteps() {
    float scaled = (float)modeAdjustedMaxSteps(VVI_HOME_MAX_STEPS, VVI_STEP_MODE) * VVI_HOME_SEARCH_SCALE_TRIM;
    uint32_t adjusted = (uint32_t)(scaled + 0.5f);
    if (adjusted < VVI_HOME_MAX_STEPS) adjusted = VVI_HOME_MAX_STEPS;
    return adjusted;
}

static uint32_t aoaHomeSearchMaxSteps() {
    float scaled = (float)modeAdjustedMaxSteps(AOA_HOME_MAX_STEPS, AOA_STEP_MODE) * AOA_HOME_SEARCH_SCALE_TRIM;
    uint32_t adjusted = (uint32_t)(scaled + 0.5f);
    if (adjusted < AOA_HOME_MAX_STEPS) adjusted = AOA_HOME_MAX_STEPS;
    return adjusted;
}

static void applyPanelLighting() {
    pwm_set_chan_level(vvi_light_pwm_slice, vvi_light_pwm_chan, vviLightLevel);
    pwm_set_chan_level(aoa_light_pwm_slice, aoa_light_pwm_chan, aoaLightLevel);
}

static x27_motor_t vviMotor;
static bool vviMotorReady = false;
static bool vviFirstDataSynced = false;
static x27_motor_t aoaMotor;
static bool aoaMotorReady = false;
static bool aoaFirstDataSynced = false;
// vviMotorConfig is Motor A (VVI), aoaMotorConfig is Motor B (AoA).
static const x27_gpio_config_t vviMotorConfig = {
    VVI_COIL_1A_PIN,
    VVI_COIL_1B_PIN,
    VVI_COIL_2A_PIN,
    VVI_COIL_2B_PIN
};
static const x27_gpio_config_t aoaMotorConfig = {
    AOA_COIL_1A_PIN,
    AOA_COIL_1B_PIN,
    AOA_COIL_2A_PIN,
    AOA_COIL_2B_PIN
};

static constexpr float VVI_ZERO_ANGLE_DEG = 110.0f;
static constexpr float VVI_RAW_MIN_ANGLE_DEG = 110.0f;
static constexpr float VVI_RAW_MAX_ANGLE_DEG = 333.0f; // Maps max to 669 steps with 1080 steps/rev and 110-degree zero.
static constexpr int32_t AOA_MIN_POSITION_STEPS = 0;
static constexpr int32_t AOA_FULL_STEP_TRAVEL_STEPS = 544; // Required: max DCS value -> +544 steps from AoA zero in FULL_STEP mode.

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

static int32_t angleToX27Position(float angle_deg) {
    float relative = angle_deg - VVI_ZERO_ANGLE_DEG;
    if (relative < 0.0f) relative = 0.0f;
    float stepsf = (relative / 360.0f) * (float)X27_MAX_POSITION;
    stepsf *= vviPositionScale();
    int32_t steps = (int32_t)(stepsf + 0.5f);
    if (steps < 0) steps = 0;
    if (steps >= X27_MAX_POSITION) steps = X27_MAX_POSITION - 1;
    return steps;
}

static float rawToVviAngle(unsigned int rawValue) {
    float t = (float)rawValue / 65535.0f;
    return lerp(VVI_RAW_MIN_ANGLE_DEG, VVI_RAW_MAX_ANGLE_DEG, t);
}

static bool zeroVviGaugeAtSensorReference() {
    bool sensor_ok = x27_config_homing_sensor(&vviMotor, VVI_HOME_SENSOR_PIN, false, true);
    bool homed = false;

    if (sensor_ok) {
        bool sensor_now = (gpio_get(VVI_HOME_SENSOR_PIN) == 0);
        if (sensor_now) {
            homed = true;
            printf("VVI home sensor active at startup (GPIO %d)\n", VVI_HOME_SENSOR_PIN);
        } else {
            homed = x27_home_with_sensor(&vviMotor, -1, vviHomeSearchMaxSteps());
            if (homed) {
                printf("VVI home sensor triggered during -1 search (GPIO %d)\n", VVI_HOME_SENSOR_PIN);
            }
            if (!homed) {
                homed = x27_home_with_sensor(&vviMotor, +1, vviHomeSearchMaxSteps());
                if (homed) {
                    printf("VVI home sensor triggered during +1 search (GPIO %d)\n", VVI_HOME_SENSOR_PIN);
                }
            }
        }
    }

    int32_t vviRestPos = angleToX27Position(VVI_RAW_MIN_ANGLE_DEG); // Known reference from the sensor home.
    if (!homed) {
        printf("VVI home sensor not found; parking to known reference\n");
    }

    x27_set_position(&vviMotor, vviRestPos);
    x27_wait_complete(&vviMotor);
    vviMotor.current_position = vviRestPos;
    vviMotor.target_position = vviRestPos;
    x27_set_speed(&vviMotor, VVI_STEP_DELAY_US);
    printf("VVI zeroed at known reference, homed=%d sensor=%d\n", homed ? 1 : 0, sensor_ok ? 1 : 0);
    return homed;
}

void onPltVviNeedleChange(unsigned int newValue) {
    if (!vviMotorReady || !vviMotor.initialized) {
        return;
    }

    if (!vviFirstDataSynced) {
        x27_set_speed(&vviMotor, VVI_HOME_STEP_DELAY_US);
        bool homed = zeroVviGaugeAtSensorReference();
        x27_set_speed(&vviMotor, VVI_STEP_DELAY_US);
        vviFirstDataSynced = true;
        printf("VVI first-data sync complete (homed=%d)\n", homed ? 1 : 0);
    }

    float angle_deg = rawToVviAngle(newValue);
    int32_t position = angleToX27Position(angle_deg);
    x27_set_position(&vviMotor, position);
}
DcsBios::IntegerBuffer pltVviNeedleBuffer(0x2bc4, 0xffff, 0, onPltVviNeedleChange);

static int32_t aoaToPosition(float aoa) {
    aoa = clampf(aoa, 0.0f, 30.0f);
    float t = aoa / 30.0f;
    float scaledTravel = (float)AOA_FULL_STEP_TRAVEL_STEPS * aoaPositionScale();
    int32_t position = AOA_MIN_POSITION_STEPS + (int32_t)(t * scaledTravel + 0.5f);
    if (position < 0) position = 0;
    if (position > X27_MAX_POSITION) position = X27_MAX_POSITION;
    return position;
}

void onPltAoaGaugeNeedleChange(unsigned int newValue) {
    if (!aoaMotorReady || !aoaMotor.initialized) {
        return;
    }

    if (!aoaFirstDataSynced) {
        bool sensor_ok = x27_config_homing_sensor(&aoaMotor, AOA_HOME_SENSOR_PIN, false, true);
        bool homed = false;
        if (sensor_ok) {
            bool sensor_now = (gpio_get(AOA_HOME_SENSOR_PIN) == 0);
            if (sensor_now) {
                homed = true;
            } else {
                x27_set_speed(&aoaMotor, AOA_HOME_STEP_DELAY_US);
                homed = x27_home_with_sensor(&aoaMotor, -1, aoaHomeSearchMaxSteps());
                if (!homed) {
                    homed = x27_home_with_sensor(&aoaMotor, +1, aoaHomeSearchMaxSteps());
                }
                x27_set_speed(&aoaMotor, AOA_STEP_DELAY_US);
            }
        }

        int32_t aoaRestPos = aoaToPosition(0.0f);
        aoaMotor.current_position = aoaRestPos;
        aoaMotor.target_position = aoaRestPos;
        aoaFirstDataSynced = true;
        printf("AOA first-data sync complete (sensor=%d homed=%d)\n", sensor_ok ? 1 : 0, homed ? 1 : 0);
    }

    // 0..65535 -> 0..30 AoA units
    float aoa = ((float)newValue / 65535.0f) * 30.0f;
    x27_set_position(&aoaMotor, aoaToPosition(aoa));
}
DcsBios::IntegerBuffer pltAoaGaugeNeedleBuffer(0x2a94, 0xffff, 0, onPltAoaGaugeNeedleChange);

static void initVviGaugeMotor() {
    bool ok = x27_init_gpio(&vviMotor, &vviMotorConfig, VVI_STEP_MODE);
    if (!ok) {
        printf("VVI motor init failed\n");
        return;
    }

    x27_set_direction_inverted(&vviMotor, true);

    x27_set_speed(&vviMotor, VVI_HOME_STEP_DELAY_US);
    vviMotorReady = true;
    bool homed = zeroVviGaugeAtSensorReference();
    vviFirstDataSynced = homed;
    printf("VVI motor ready, homed=%d sensor=%d mode=%d scale=%.2f homeMax=%lu\n", homed ? 1 : 0, vviMotor.homing_configured ? 1 : 0, (int)VVI_STEP_MODE, (double)vviPositionScale(), (unsigned long)vviHomeSearchMaxSteps());
}

static void initAoaGaugeMotor() {
    bool ok = x27_init_gpio(&aoaMotor, &aoaMotorConfig, AOA_STEP_MODE);
    if (!ok) {
        printf("AOA motor init failed\n");
        return;
    }
    // AoA must travel counterclockwise to zero for calibration consistency.
    x27_set_direction_inverted(&aoaMotor, AOA_DIRECTION_INVERTED);
    x27_set_speed(&aoaMotor, AOA_HOME_STEP_DELAY_US);

    bool sensor_ok = x27_config_homing_sensor(&aoaMotor, AOA_HOME_SENSOR_PIN, false, true);
    bool homed = false;
    if (sensor_ok) {
        bool sensor_now = (gpio_get(AOA_HOME_SENSOR_PIN) == 0);
        if (sensor_now) {
            homed = true;
            printf("AOA home sensor active at startup (GPIO %d)\n", AOA_HOME_SENSOR_PIN);
        } else {
            homed = x27_home_with_sensor(&aoaMotor, -1, aoaHomeSearchMaxSteps());
            if (homed) {
                printf("AOA home sensor triggered during -1 search (GPIO %d)\n", AOA_HOME_SENSOR_PIN);
            }
            if (!homed) {
                homed = x27_home_with_sensor(&aoaMotor, +1, aoaHomeSearchMaxSteps());
                if (homed) {
                    printf("AOA home sensor triggered during +1 search (GPIO %d)\n", AOA_HOME_SENSOR_PIN);
                }
            }
        }
    }

    int32_t aoaRestPos = aoaToPosition(0.0f); // AoA zero at 9 o'clock.
    if (homed) {
        aoaMotor.current_position = aoaRestPos;
        aoaMotor.target_position = aoaRestPos;
    } else {
        printf("AOA home sensor not found; parking to zero reference\n");
    }
    x27_set_position(&aoaMotor, aoaRestPos);
    x27_wait_complete(&aoaMotor);
    aoaMotor.current_position = aoaRestPos;
    aoaMotor.target_position = aoaRestPos;
    aoaMotorReady = true;
    x27_set_speed(&aoaMotor, AOA_STEP_DELAY_US);
    printf("AOA motor ready, homed=%d sensor=%d mode=%d scale=%.2f homeMax=%lu\n", homed ? 1 : 0, sensor_ok ? 1 : 0, (int)AOA_STEP_MODE, (double)aoaPositionScale(), (unsigned long)aoaHomeSearchMaxSteps());
}

static void setStartupGreen(bool enabled) {
    unsigned char level = enabled ? STARTUP_GREEN_LEVEL : 0;
    pwm_set_chan_level(vvi_light_pwm_slice, vvi_light_pwm_chan, level);
    pwm_set_chan_level(aoa_light_pwm_slice, aoa_light_pwm_chan, level);
}

static void runMotorsUntilTargetOrTimeout(uint32_t timeoutMs) {
    absolute_time_t deadline = make_timeout_time_ms(timeoutMs);
    while (true) {
        bool allAtTarget = true;

        if (vviMotorReady) {
            x27_update(&vviMotor);
            if (!x27_is_at_target(&vviMotor)) {
                allAtTarget = false;
            }
        }
        if (aoaMotorReady) {
            x27_update(&aoaMotor);
            if (!x27_is_at_target(&aoaMotor)) {
                allAtTarget = false;
            }
        }

        if (allAtTarget) {
            return;
        }
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return;
        }
        sleep_us(200);
    }
}

static void runFullGaugeSweep() {
    int32_t sweepFar = angleToX27Position(0.0f);

    if (aoaMotorReady) {
        x27_set_speed(&aoaMotor, STARTUP_SWEEP_STEP_DELAY_US);
    }

    if (aoaMotorReady) {
        x27_set_position(&aoaMotor, sweepFar);
    }
    runMotorsUntilTargetOrTimeout(GAUGE_SWEEP_TIMEOUT_MS);
    sleep_ms(STARTUP_SWEEP_DWELL_MS);

    if (aoaMotorReady) {
        x27_set_position(&aoaMotor, aoaMotor.current_position);
    }
    runMotorsUntilTargetOrTimeout(GAUGE_SWEEP_TIMEOUT_MS);
    sleep_ms(STARTUP_SWEEP_DWELL_MS);

    if (aoaMotorReady) {
        x27_set_speed(&aoaMotor, AOA_STEP_DELAY_US);
    }
}

void onPltIntLightInstrumentPanelChange(unsigned int newValue) {
    vviLightLevel = (unsigned char)((newValue * 255) / 65535);
    aoaLightLevel = (unsigned char)((newValue * 255) / 65535);
    applyPanelLighting();
}
DcsBios::IntegerBuffer pltIntLightInstrumentPanelBuffer(0x2d88, 0xffff, 0, onPltIntLightInstrumentPanelChange);


int main() {
    stdio_init_all();  // Initialize USB CDC
    DcsBios::initHeartbeat(HEARTBEAT_LED);  // Initialize heartbeat LED
    sleep_ms(2000);   // Wait for USB CDC to be ready

    gpio_set_function(VVI_LIGHT_PWM_PIN, GPIO_FUNC_PWM);
    vvi_light_pwm_slice = pwm_gpio_to_slice_num(VVI_LIGHT_PWM_PIN);
    vvi_light_pwm_chan = pwm_gpio_to_channel(VVI_LIGHT_PWM_PIN);
    pwm_set_clkdiv(vvi_light_pwm_slice, 4.0f);
    pwm_set_wrap(vvi_light_pwm_slice, 255);
    pwm_set_enabled(vvi_light_pwm_slice, true);

    gpio_set_function(AOA_LIGHT_PWM_PIN, GPIO_FUNC_PWM);
    aoa_light_pwm_slice = pwm_gpio_to_slice_num(AOA_LIGHT_PWM_PIN);
    aoa_light_pwm_chan = pwm_gpio_to_channel(AOA_LIGHT_PWM_PIN);
    pwm_set_clkdiv(aoa_light_pwm_slice, 4.0f);
    pwm_set_wrap(aoa_light_pwm_slice, 255);
    pwm_set_enabled(aoa_light_pwm_slice, true);

    setStartupGreen(true);

    uint64_t startupGreenBeginUs = to_us_since_boot(get_absolute_time());

    initVviGaugeMotor();
    initAoaGaugeMotor();

    // Do not hold coils energized during startup light timing sequence.
    if (vviMotorReady) {
        x27_sleep(&vviMotor);
    }
    if (aoaMotorReady) {
        x27_sleep(&aoaMotor);
    }

    uint64_t startupElapsedMs = (to_us_since_boot(get_absolute_time()) - startupGreenBeginUs) / 1000u;
    if (startupElapsedMs < 3000u) {
        sleep_ms((uint32_t)(3000u - startupElapsedMs));
    }

    absolute_time_t flashDeadline = make_timeout_time_ms(STARTUP_FLASH_DURATION_MS);
    bool flashOn = false;
    while (absolute_time_diff_us(get_absolute_time(), flashDeadline) > 0) {
        flashOn = !flashOn;
        setStartupGreen(flashOn);
        sleep_ms(STARTUP_FLASH_INTERVAL_MS);
    }
    setStartupGreen(false);
    applyPanelLighting();

    runFullGaugeSweep();

    unsigned char boardAddress = 0xF; // Keep as 0xF for now, as the user is debugging buffer overflow on slave
                                // This will be changed to SLAVE_MODE_MIN in a later step if needed.

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

    DcsBios::setup(); // Initialize DCS-BIOS framework
    printf("DCS-BIOS setup complete!\n");
    while (true)
    {
        DcsBios::loop();            // Handle input, output, and LED updates
        DcsBios::updateHeartbeat(); // Update heartbeat LED
        if (vviMotorReady) {
            x27_update(&vviMotor);
        }
        if (aoaMotorReady) {
            x27_update(&aoaMotor);
        }
        sleep_us(10);
    }
}
