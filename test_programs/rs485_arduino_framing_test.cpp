#include "pico/stdio.h"
#include "pico/stdlib.h"

#include "internal/rs485_arduino.h"

using DcsBios::ArduinoRs485::ActiveSlaveSet;
using DcsBios::ArduinoRs485::Frame;
using DcsBios::ArduinoRs485::FrameReceiver;
using DcsBios::ArduinoRs485::PollConfig;

extern "C" int printf(const char* format, ...);

int main() {
    stdio_init_all();

    FrameReceiver rx;
    Frame frame;

    const uint8_t bytes[] = {5, 0, 3, 'A', 'B', 'C', 0};
    bool gotFrame = false;
    for (uint8_t byte : bytes) {
        if (rx.pushByte(byte, frame)) {
            gotFrame = true;
        }
    }

    if (gotFrame && frame.address == 5 && frame.msgType == 0 && frame.length == 3 &&
        frame.data[0] == 'A' && frame.data[1] == 'B' && frame.data[2] == 'C') {
        printf("FrameReceiver OK\n");
    } else {
        printf("FrameReceiver FAIL\n");
    }

    ActiveSlaveSet active;
    PollConfig cfg;
    cfg.activeOnly = true;
    cfg.maxMisses = 2;
    active.setConfig(cfg);

    active.recordResponse(5);
    printf("Active after response: %d\n", active.isActive(5) ? 1 : 0);

    active.recordTimeout(5);
    active.recordTimeout(5);
    printf("Active after timeouts: %d\n", active.isActive(5) ? 1 : 0);

    while (true) {
        sleep_ms(1000);
    }

    return 0;
}
