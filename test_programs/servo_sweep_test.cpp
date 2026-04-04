#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "DcsBios.h"
#include "internal/heartbeat.h"
#include "internal/BoardMode.h"

// ========================= User-configurable settings =========================
static constexpr uint SERVO_PIN = 16; // GPIO pin for the test servo signal wire.

static constexpr bool ENABLE_STARTUP_SWEEP = true; // true = run startup sweep, false = skip.
static constexpr uint16_t SWEEP_MIN_US = 400; // Startup sweep minimum pulse width in microseconds.
static constexpr uint16_t SWEEP_MAX_US = 2400; // Startup sweep maximum pulse width in microseconds.
static constexpr uint16_t SWEEP_STEP_US = 20; // Startup sweep increment per step in microseconds.
static constexpr uint16_t SWEEP_STEP_DELAY_MS = 8; // Delay between sweep steps in milliseconds.
static constexpr uint16_t SWEEP_END_PAUSE_MS = 2000; // Pause at each sweep endpoint (min and max).

static constexpr bool ENABLE_SERIAL_RANGE_TUNER = true; // true = enter serial tuning mode before DCS starts.
static constexpr uint16_t SERIAL_TUNER_START_US = SWEEP_MIN_US; // Initial pulse width when tuner starts.
static constexpr uint16_t SERIAL_TUNER_MIN_US = 400; // Lowest allowed pulse while tuning.
static constexpr uint16_t SERIAL_TUNER_MAX_US = 3200; // Highest allowed pulse while tuning.
static constexpr uint16_t SERIAL_TUNER_STEP_US = 10; // Pulse change per 'u' (up) or 'd' (down) command.
static constexpr uint16_t SERIAL_TUNER_POLL_DELAY_MS = 2; // Idle polling delay for serial input.
static constexpr size_t SERIAL_TUNER_CMD_BUFFER_LEN = 40; // Max serial command length for tuner parsing.

static constexpr int DCS_SERVO_MIN_US = 400; // DCS mapping minimum pulse width in microseconds.
static constexpr int DCS_SERVO_MAX_US = 2400; // DCS mapping maximum pulse width in microseconds.
static constexpr unsigned int DCS_PLT_PNEUMATIC_GAUGE_ADDR = 0x2aea; // DCS-BIOS address for the gauge.

static constexpr uint8_t BOARD_ADDRESS = 0xF; // 0xF = standalone USB mode in this project.
static constexpr uint32_t USB_STARTUP_DELAY_MS = 2000; // USB serial startup delay after boot.
// ============================================================================

static uint16_t pulseWidthUsToPwmLevel(uint32_t us) {
    uint32_t level = (us * 39062u) / 20000u;
    if (level > 39062u) {
        level = 39062u;
    }
    return static_cast<uint16_t>(level);
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

static uint16_t clampPulseUs(uint16_t pulseWidthUs, uint16_t minUs, uint16_t maxUs) {
    if (pulseWidthUs < minUs) {
        return minUs;
    }
    if (pulseWidthUs > maxUs) {
        return maxUs;
    }
    return pulseWidthUs;
}

static bool updatePulseByDelta(uint16_t* currentPulseUs, int32_t deltaUs) {
    int32_t next = static_cast<int32_t>(*currentPulseUs) + deltaUs;
    if (next < 0) {
        next = 0;
    }
    if (next > 65535) {
        next = 65535;
    }

    uint16_t clamped = clampPulseUs(static_cast<uint16_t>(next), SERIAL_TUNER_MIN_US, SERIAL_TUNER_MAX_US);
    if (clamped == *currentPulseUs) {
        return false;
    }

    *currentPulseUs = clamped;
    return true;
}

static bool parsePulseValue(const char* text, uint16_t* outPulseUs) {
    while (*text != '\0' && isspace(static_cast<unsigned char>(*text))) {
        ++text;
    }

    if (*text == '\0') {
        return false;
    }

    char* end = nullptr;
    long value = strtol(text, &end, 10);
    if (end == text) {
        return false;
    }

    while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) {
        ++end;
    }

    if (*end != '\0' || value < 0 || value > 65535) {
        return false;
    }

    *outPulseUs = static_cast<uint16_t>(value);
    return true;
}

static void printSerialTunerHelp() {
    printf("\nSerial range tuner commands:\n");
    printf("  u : increase by SERIAL_TUNER_STEP_US\n");
    printf("  d : decrease by SERIAL_TUNER_STEP_US\n");
    printf("  i : increase by 1 us\n");
    printf("  k : decrease by 1 us\n");
    printf("  s <us> : set/go-to a specific pulse width\n");
    printf("  <us> : shortcut for go-to (example: 1500)\n");
    printf("  p : print current pulse\n");
    printf("  m : print suggestion for min constants\n");
    printf("  x : print suggestion for max constants\n");
    printf("  h/? : show this help\n");
    printf("  g/q/e : exit tuner and continue startup\n");
    printf("  aliases kept: + - [ ]\n");
    printf("Note: some terminals are line-buffered, so press Enter after commands.\n\n");
}

static void processSerialTunerCommand(const char* commandText, uint16_t* currentPulseUs, bool* shouldExit) {
    while (*commandText != '\0' && isspace(static_cast<unsigned char>(*commandText))) {
        ++commandText;
    }

    if (*commandText == '\0') {
        return;
    }

    bool updated = false;
    char cmd = static_cast<char>(tolower(static_cast<unsigned char>(*commandText)));

    switch (cmd) {
    case 'u':
    case '+':
        updated = updatePulseByDelta(currentPulseUs, SERIAL_TUNER_STEP_US);
        break;
    case 'd':
    case '-':
        updated = updatePulseByDelta(currentPulseUs, -static_cast<int32_t>(SERIAL_TUNER_STEP_US));
        break;
    case 'i':
    case ']':
        updated = updatePulseByDelta(currentPulseUs, 1);
        break;
    case 'k':
    case '[':
        updated = updatePulseByDelta(currentPulseUs, -1);
        break;
    case 'p':
        printf("Current pulse: %u us\n", *currentPulseUs);
        break;
    case 'm':
        printf("Min suggestion -> SWEEP_MIN_US=%u, DCS_SERVO_MIN_US=%u\n", *currentPulseUs, *currentPulseUs);
        break;
    case 'x':
        printf("Max suggestion -> SWEEP_MAX_US=%u, DCS_SERVO_MAX_US=%u\n", *currentPulseUs, *currentPulseUs);
        break;
    case 'h':
    case '?':
        printSerialTunerHelp();
        break;
    case 'g':
    case 'q':
    case 'e':
        *shouldExit = true;
        return;
    case 's': {
        uint16_t requestedPulseUs = 0;
        if (!parsePulseValue(commandText + 1, &requestedPulseUs)) {
            printf("Invalid set command. Use: s <us>\n");
            break;
        }
        uint16_t clamped = clampPulseUs(requestedPulseUs, SERIAL_TUNER_MIN_US, SERIAL_TUNER_MAX_US);
        if (clamped != requestedPulseUs) {
            printf("Requested %u us clamped to %u us (limits %u..%u)\n", requestedPulseUs, clamped, SERIAL_TUNER_MIN_US, SERIAL_TUNER_MAX_US);
        }
        updated = (clamped != *currentPulseUs);
        *currentPulseUs = clamped;
        break;
    }
    default: {
        uint16_t requestedPulseUs = 0;
        if (parsePulseValue(commandText, &requestedPulseUs)) {
            uint16_t clamped = clampPulseUs(requestedPulseUs, SERIAL_TUNER_MIN_US, SERIAL_TUNER_MAX_US);
            if (clamped != requestedPulseUs) {
                printf("Requested %u us clamped to %u us (limits %u..%u)\n", requestedPulseUs, clamped, SERIAL_TUNER_MIN_US, SERIAL_TUNER_MAX_US);
            }
            updated = (clamped != *currentPulseUs);
            *currentPulseUs = clamped;
        } else {
            printf("Unknown command: %s\n", commandText);
            printf("Type h for help.\n");
        }
        break;
    }
    }

    if (updated) {
        setServoPulseUs(SERVO_PIN, *currentPulseUs);
        printf("Pulse: %u us\n", *currentPulseUs);
    }
}

static void runSerialRangeTunerIfEnabled() {
    if (!ENABLE_SERIAL_RANGE_TUNER) {
        return;
    }

    initServoPwmPin(SERVO_PIN);
    uint16_t currentPulseUs = clampPulseUs(SERIAL_TUNER_START_US, SERIAL_TUNER_MIN_US, SERIAL_TUNER_MAX_US);
    setServoPulseUs(SERVO_PIN, currentPulseUs);

    printf("\nSerial range tuner enabled on GPIO%u\n", SERVO_PIN);
    printf("Current pulse: %u us (limits: %u..%u us)\n", currentPulseUs, SERIAL_TUNER_MIN_US, SERIAL_TUNER_MAX_US);
    printSerialTunerHelp();

    char commandBuffer[SERIAL_TUNER_CMD_BUFFER_LEN] = {0};
    size_t commandLength = 0;

    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            sleep_ms(SERIAL_TUNER_POLL_DELAY_MS);
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (commandLength > 0) {
                commandBuffer[commandLength] = '\0';
                bool shouldExit = false;
                processSerialTunerCommand(commandBuffer, &currentPulseUs, &shouldExit);
                if (shouldExit) {
                    printf("Exiting serial range tuner at %u us\n\n", currentPulseUs);
                    return;
                }
                commandLength = 0;
            }
            continue;
        }

        if (commandLength == 0) {
            char singleCharCommand[2] = {static_cast<char>(ch), '\0'};
            bool shouldExit = false;
            if (strchr("uUdDiIkK+-[]pPmMxXhH?gGqQeE", ch) != nullptr) {
                processSerialTunerCommand(singleCharCommand, &currentPulseUs, &shouldExit);
                if (shouldExit) {
                    printf("Exiting serial range tuner at %u us\n\n", currentPulseUs);
                    return;
                }
                continue;
            }
        }

        if (isprint(static_cast<unsigned char>(ch)) && commandLength < (SERIAL_TUNER_CMD_BUFFER_LEN - 1)) {
            commandBuffer[commandLength++] = static_cast<char>(ch);
        }
    }
}

static void runStartupSweepIfEnabled() {
    if (!ENABLE_STARTUP_SWEEP || SWEEP_STEP_US == 0 || SWEEP_MIN_US >= SWEEP_MAX_US) {
        return;
    }

    initServoPwmPin(SERVO_PIN);
    setServoPulseUs(SERVO_PIN, SWEEP_MIN_US);
    sleep_ms(SWEEP_END_PAUSE_MS);

    for (uint16_t us = SWEEP_MIN_US; us <= SWEEP_MAX_US; us = static_cast<uint16_t>(us + SWEEP_STEP_US)) {
        setServoPulseUs(SERVO_PIN, us);
        sleep_ms(SWEEP_STEP_DELAY_MS);
        if (static_cast<uint16_t>(us + SWEEP_STEP_US) < us) {
            break;
        }
    }
    sleep_ms(SWEEP_END_PAUSE_MS);

    for (int us = static_cast<int>(SWEEP_MAX_US); us >= static_cast<int>(SWEEP_MIN_US); us -= static_cast<int>(SWEEP_STEP_US)) {
        setServoPulseUs(SERVO_PIN, static_cast<uint16_t>(us));
        sleep_ms(SWEEP_STEP_DELAY_MS);
    }
    sleep_ms(SWEEP_END_PAUSE_MS);
}

// DCS-BIOS driven servo test
DcsBios::ServoOutput pltPneumaticGauge(DCS_PLT_PNEUMATIC_GAUGE_ADDR, SERVO_PIN, DCS_SERVO_MIN_US, DCS_SERVO_MAX_US);

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    sleep_ms(USB_STARTUP_DELAY_MS);

    runStartupSweepIfEnabled();
    runSerialRangeTunerIfEnabled();

    uint8_t boardAddress = BOARD_ADDRESS;
    DcsBios::BoardMode board = DcsBios::determineBoardMode(boardAddress);
    printf("Board address: 0x%X\n", boardAddress);

    switch (board.mode) {
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

    multicore_launch_core1(DcsBios::core1_task);
    printf("Core 1 task launched!\n");

    DcsBios::setup();
    printf("DCS-BIOS setup complete!\n");

    while (true) {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        sleep_us(10);
    }
}