#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/gpio.h"
#include "hardware/uart.h"

#include "internal/FoxConfig.h"
#include "internal/rs485_arduino.h"
#include "internal/ws2812.h"

#if defined(PICO_DEFAULT_WS2812_PIN) && (HEARTBEAT_LED == PICO_DEFAULT_WS2812_PIN)
#define USE_WS2812_STATUS_LED 1
#else
#define USE_WS2812_STATUS_LED 0
#endif

// RP2350 Zero RS485 master with two independent buses:
// - BUS_1 (RP2350 slaves): native DCS-BIOS RP2350 framing on UART0
//   TX=GPIO0 (S_TX0), RX=GPIO1 (S_RX0), EN=GPIO2 (SLAVE1)
// - BUS_2 (Arduino slaves): Arduino RS485 framing on UART1
//   TX=GPIO4 (S_TX1), RX=GPIO5 (S_RX1), EN=GPIO6 (SLAVE2)

namespace {
constexpr uint32_t RS485_BAUD = 250000;
constexpr uint BUS2_TX_PIN = 4;
constexpr uint BUS2_RX_PIN = 5;
constexpr uint BUS2_EN_PIN = 6;
constexpr unsigned char RP2350_SLAVE_ADDR_MAX = 0x0F;
constexpr size_t BUS1_MSG_BUFFER_SIZE = 128;
constexpr int64_t LINK_ACTIVE_WINDOW_US = 1500000;  // 1.5s activity window.

struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

enum class StatusState {
    RunningNoLink,
    DcsOnly,
    Bus1Only,
    Bus2Only,
    Bus1AndBus2,
    AllGood
};

struct Rs485Port {
    uart_inst_t* uart;
    uint txPin;
    uint rxPin;
    uint enPin;
};

static WS2812 statusLed(pio0, 0, HEARTBEAT_LED, false);
static bool statusLedReady = false;

// Initializes the status LED or the heartbeat LED. It sets the brightness and color based on the defined hardware.
void initStatusLed() {
#if USE_WS2812_STATUS_LED
    statusLedReady = statusLed.begin(1);
    if (statusLedReady) {
        statusLed.setBrightness(48);
        statusLed.setPixel(0, statusLed.rgb(0, 0, 0));
        statusLed.show();
    }
#else
    gpio_init(HEARTBEAT_LED);
    gpio_set_dir(HEARTBEAT_LED, GPIO_OUT);
    gpio_put(HEARTBEAT_LED, 0);
#endif
}

// Sets the status LED color based on the current state. The function maps the current state to a specific RGB color.
void setStatusColor(const Rgb& c) {
#if USE_WS2812_STATUS_LED
    if (!statusLedReady) return;
    statusLed.setPixel(0, statusLed.rgb(c.r, c.g, c.b));
    statusLed.show();
#else
    const bool on = (c.r | c.g | c.b) != 0;
    gpio_put(HEARTBEAT_LED, on ? 1 : 0);
#endif
}

StatusState determineStatus(bool usbActive, bool bus1Active, bool bus2Active) {
    if (usbActive && bus1Active && bus2Active) return StatusState::AllGood;
    if (bus1Active && bus2Active) return StatusState::Bus1AndBus2;
    if (bus1Active) return StatusState::Bus1Only;
    if (bus2Active) return StatusState::Bus2Only;
    if (usbActive) return StatusState::DcsOnly;
    return StatusState::RunningNoLink;
}

void updateStatusLed(bool usbSeen,
                     bool bus1Seen,
                     bool bus2Seen,
                     absolute_time_t lastUsbActivity,
                     absolute_time_t lastBus1Activity,
                     absolute_time_t lastBus2Activity,
                     absolute_time_t now,
                     absolute_time_t& lastStatusLedUpdate) {
    if (absolute_time_diff_us(lastStatusLedUpdate, now) < 100000) {
        return;  // Limit LED updates to 10Hz.
    }

    const bool usbActive = usbSeen && (absolute_time_diff_us(lastUsbActivity, now) < LINK_ACTIVE_WINDOW_US);
    const bool bus1Active = bus1Seen && (absolute_time_diff_us(lastBus1Activity, now) < LINK_ACTIVE_WINDOW_US);
    const bool bus2Active = bus2Seen && (absolute_time_diff_us(lastBus2Activity, now) < LINK_ACTIVE_WINDOW_US);

    const StatusState state = determineStatus(usbActive, bus1Active, bus2Active);

    // RunningNoLink blinks blue, all other states are steady colors.
    if (state == StatusState::RunningNoLink) {
        const bool blinkOn = ((to_ms_since_boot(now) / 500) % 2) == 0;
        setStatusColor(blinkOn ? Rgb{0, 0, 24} : Rgb{0, 0, 0});
    } else if (state == StatusState::DcsOnly) {
        setStatusColor({0, 24, 24});      // Cyan: DCS traffic seen.
    } else if (state == StatusState::Bus1Only) {
        setStatusColor({24, 0, 24});      // Magenta: BUS_1 active only.
    } else if (state == StatusState::Bus2Only) {
        setStatusColor({24, 10, 0});      // Amber: BUS_2 active only.
    } else if (state == StatusState::Bus1AndBus2) {
        setStatusColor({24, 24, 0});      // Yellow: both buses active.
    } else {
        setStatusColor({0, 24, 0});       // Green: DCS + BUS_1 + BUS_2 active.
    }

    lastStatusLedUpdate = now;
}

// Waits for the UART transmission to complete. The function loops until the UART becomes writable. It then pauses for 200 microseconds.
inline void waitUartTxComplete(const Rs485Port& port) {
    while (!uart_is_writable(port.uart)) {
    }
    sleep_us(200);
}

// Initializes the RS485 port hardware. It sets the UART format and configures the GPIO pins. It starts the port in receive mode.
void initRs485Port(const Rs485Port& port, uint32_t baudrate) {
    uart_init(port.uart, baudrate);
    gpio_set_function(port.txPin, GPIO_FUNC_UART);
    gpio_set_function(port.rxPin, GPIO_FUNC_UART);
    gpio_init(port.enPin);
    gpio_set_dir(port.enPin, GPIO_OUT);
    gpio_put(port.enPin, 0);  // Start in receive mode.
    uart_set_format(port.uart, 8, 1, UART_PARITY_NONE);
}

inline void rs485TxEnable(const Rs485Port& port) {
    gpio_put(port.enPin, 1);
}

inline void rs485TxDisable(const Rs485Port& port) {
    gpio_put(port.enPin, 0);
}

void rs485SendChar(const Rs485Port& port, char c) {
    rs485TxEnable(port);
    uart_putc_raw(port.uart, c);
    waitUartTxComplete(port);
    rs485TxDisable(port);
}

void rs485SendBytes(const Rs485Port& port, const unsigned char* data, size_t len, bool flush) {
    if (!data || len == 0) {
        if (flush) {
            waitUartTxComplete(port);
            sleep_us(400);
            rs485TxDisable(port);
        }
        return;
    }

    rs485TxEnable(port);
    for (size_t i = 0; i < len; ++i) {
        while (!uart_is_writable(port.uart)) {
        }
        uart_putc_raw(port.uart, data[i]);
    }

    if (flush) {
        waitUartTxComplete(port);
        sleep_us(400);
        rs485TxDisable(port);
    } else {
        waitUartTxComplete(port);
    }
}

inline bool rs485ReceiveAvailable(const Rs485Port& port) {
    return uart_is_readable(port.uart);
}

inline unsigned char rs485ReceiveByte(const Rs485Port& port) {
    return static_cast<unsigned char>(uart_getc(port.uart));
}

void sendArduinoFrame(const Rs485Port& port, unsigned char address, unsigned char msgType, const unsigned char* data, unsigned char length) {
    sleep_us(DCSBIOS_RS485_ARDUINO_INTERFRAME_GAP_US);

    unsigned char header[3] = {address, msgType, length};
    rs485SendBytes(port, header, sizeof(header), false);

    if (length > 0 && data) {
        rs485SendBytes(port, data, length, false);
    }

    const unsigned char checksum = 0;
    rs485SendBytes(port, &checksum, 1, true);
}
}  // namespace

int main() {
    stdio_init_all();
    sleep_ms(1500);  // Allow USB CDC to enumerate.

    initStatusLed();

    const Rs485Port bus1Rp2350 = {
        .uart = uart0,
        .txPin = UART0_TX,
        .rxPin = UART0_RX,
        .enPin = RS485_EN,
    };

    const Rs485Port bus2Arduino = {
        .uart = uart1,
        .txPin = BUS2_TX_PIN,
        .rxPin = BUS2_RX_PIN,
        .enPin = BUS2_EN_PIN,
    };

    initRs485Port(bus1Rp2350, RS485_BAUD);
    initRs485Port(bus2Arduino, RS485_BAUD);

    using namespace DcsBios::ArduinoRs485;
    ActiveSlaveSet activeSlaves;
    PollConfig pollCfg;
    pollCfg.activeOnly = (DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY != 0);
    pollCfg.maxMisses = DCSBIOS_RS485_ARDUINO_MAX_MISSES;
    activeSlaves.setConfig(pollCfg);

    FrameReceiver arduinoRx;
    Frame arduinoFrame{};

    unsigned char arduinoTxBuffer[DCSBIOS_RS485_ARDUINO_FRAME_MAX] = {0};
    unsigned char arduinoTxLen = 0;

    absolute_time_t lastUsbRx = get_absolute_time();
    absolute_time_t lastArduinoPoll = get_absolute_time();
    absolute_time_t arduinoResponseStart = get_absolute_time();
    absolute_time_t lastBus1Activity = get_absolute_time();
    absolute_time_t lastBus2Activity = get_absolute_time();
    absolute_time_t lastStatusLedUpdate = get_absolute_time();
    bool waitingArduinoResponse = false;
    unsigned char waitingArduinoAddress = 0;
    bool usbSeen = false;
    bool bus1Seen = false;
    bool bus2Seen = false;

    char rp2350SlaveBuffer[BUS1_MSG_BUFFER_SIZE] = {0};
    unsigned char rp2350SlavePos = 0;
    bool receivingRp2350Message = false;

    printf("\n==============================================\n");
    printf(" RP2350 Zero Dual RS485 Master\n");
    printf("==============================================\n");
    printf("BUS_1 RP2350 slaves: UART0 TX=%d RX=%d EN=%d\n", UART0_TX, UART0_RX, RS485_EN);
    printf("BUS_2 Arduino slaves: UART1 TX=%d RX=%d EN=%d\n", BUS2_TX_PIN, BUS2_RX_PIN, BUS2_EN_PIN);
    printf("Baud: %lu 8N1\n", static_cast<unsigned long>(RS485_BAUD));
    printf("Arduino poll interval: %d us\n", DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US);
    printf("Arduino timeout: %d us\n", DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US);
    printf("LED status colors:\n");
    printf("  Blue blink: program running, no traffic\n");
    printf("  Cyan: DCS traffic only\n");
    printf("  Magenta: BUS_1 activity only\n");
    printf("  Amber: BUS_2 activity only\n");
    printf("  Yellow: BUS_1 + BUS_2 active\n");
    printf("  Green: DCS + BUS_1 + BUS_2 active\n");
    printf("==============================================\n");

    printf("Dual-bus bridge running.\n");

    while (true) {
        const absolute_time_t now = get_absolute_time();

        // USB CDC RX -> BUS_1 broadcast (native RP2350 slaves) and BUS_2 frame buffer (Arduino slaves)
        int usbCh = getchar_timeout_us(0);
        if (usbCh != PICO_ERROR_TIMEOUT) {
            const unsigned char byte = static_cast<unsigned char>(usbCh);

            rs485SendChar(bus1Rp2350, static_cast<char>(byte));
            usbSeen = true;
            bus1Seen = true;
            lastUsbRx = now;
            lastBus1Activity = now;

            if (arduinoTxLen < DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                arduinoTxBuffer[arduinoTxLen++] = byte;
            }

            if (arduinoTxLen >= DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                sendArduinoFrame(bus2Arduino, 0, 0, arduinoTxBuffer, arduinoTxLen);
                arduinoTxLen = 0;
            }

        }

        if (arduinoTxLen > 0 && absolute_time_diff_us(lastUsbRx, now) > 200) {
            sendArduinoFrame(bus2Arduino, 0, 0, arduinoTxBuffer, arduinoTxLen);
            arduinoTxLen = 0;
        }

        // BUS_1 RX (RP2350 slaves): receive event messages prefixed by 0x01..0x0F and forward event lines to USB.
        while (rs485ReceiveAvailable(bus1Rp2350)) {
            const char c = static_cast<char>(rs485ReceiveByte(bus1Rp2350));

            if (!receivingRp2350Message) {
                if (static_cast<unsigned char>(c) <= RP2350_SLAVE_ADDR_MAX) {
                    receivingRp2350Message = true;
                    rp2350SlavePos = 0;
                    bus1Seen = true;
                    lastBus1Activity = now;
                } else {
                    putchar(c);
                    bus1Seen = true;
                    lastBus1Activity = now;
                }
            } else {
                if (c == '\n' || rp2350SlavePos >= (BUS1_MSG_BUFFER_SIZE - 2)) {
                    for (unsigned char i = 0; i < rp2350SlavePos; ++i) {
                        putchar(rp2350SlaveBuffer[i]);
                    }
                    putchar('\n');

                    receivingRp2350Message = false;
                    rp2350SlavePos = 0;
                } else {
                    rp2350SlaveBuffer[rp2350SlavePos++] = c;
                }
            }
        }

        // BUS_2 RX (Arduino slaves): parse framed response and forward payload bytes to USB.
        while (rs485ReceiveAvailable(bus2Arduino)) {
            const unsigned char byte = rs485ReceiveByte(bus2Arduino);
            if (arduinoRx.pushByte(byte, arduinoFrame)) {
                if (waitingArduinoResponse && arduinoFrame.address == waitingArduinoAddress && arduinoFrame.msgType == 0) {
                    activeSlaves.recordResponse(arduinoFrame.address);
                    waitingArduinoResponse = false;
                    bus2Seen = true;
                    lastBus2Activity = now;

                    for (unsigned int i = 0; i < arduinoFrame.length; ++i) {
                        putchar(static_cast<char>(arduinoFrame.data[i]));
                    }
                }
            }
        }

        // BUS_2 poll loop for Arduino slaves.
        if (waitingArduinoResponse) {
            if (absolute_time_diff_us(arduinoResponseStart, now) > DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US) {
                activeSlaves.recordTimeout(waitingArduinoAddress);
                waitingArduinoResponse = false;
            }
        } else {
            if (absolute_time_diff_us(lastArduinoPoll, now) > DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US) {
                const unsigned char addr = activeSlaves.nextAddress();
                sendArduinoFrame(bus2Arduino, addr, 0, nullptr, 0);
                waitingArduinoResponse = true;
                waitingArduinoAddress = addr;
                lastArduinoPoll = now;
                arduinoResponseStart = now;
            }
        }

        updateStatusLed(
            usbSeen,
            bus1Seen,
            bus2Seen,
            lastUsbRx,
            lastBus1Activity,
            lastBus2Activity,
            now,
            lastStatusLedUpdate);

        sleep_us(10);
    }
}
