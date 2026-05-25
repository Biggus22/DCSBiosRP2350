#include "heartbeat.h"
#include "pico/stdlib.h"
#include "ws2812.h"

namespace DcsBios {
    static uint heartbeatPin = 0;
    static bool heartbeatState = false;
    static uint32_t lastHeartbeat = 0;
    static bool useWs2812Heartbeat = false;
    static WS2812* heartbeatWs2812 = nullptr;

    static inline void setWs2812Heartbeat(bool on) {
        if (!heartbeatWs2812) return;
        if (on) {
            // Dim green blink to keep the onboard LED unobtrusive.
            heartbeatWs2812->setPixel(0, heartbeatWs2812->rgb(0, 24, 0));
        } else {
            heartbeatWs2812->setPixel(0, heartbeatWs2812->rgb(0, 0, 0));
        }
        heartbeatWs2812->show();
    }

    void initHeartbeat(int pin) {
        heartbeatPin = pin;
        heartbeatState = false;
        lastHeartbeat = to_ms_since_boot(get_absolute_time());

#ifdef PICO_DEFAULT_WS2812_PIN
        if (heartbeatPin == PICO_DEFAULT_WS2812_PIN) {
            if (!heartbeatWs2812) {
                heartbeatWs2812 = new WS2812(pio0, 0, heartbeatPin, false);
            }
            if (heartbeatWs2812 && heartbeatWs2812->begin(1)) {
                useWs2812Heartbeat = true;
                setWs2812Heartbeat(false);
                return;
            }
        }
#endif

        useWs2812Heartbeat = false;
        gpio_init(heartbeatPin);
        gpio_set_dir(heartbeatPin, GPIO_OUT);
        gpio_put(heartbeatPin, 0); // Ensure off at startup
    }

    void updateHeartbeat() {
        const uint32_t interval = 500; // ms
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (now - lastHeartbeat >= interval) {
            heartbeatState = !heartbeatState;
            if (useWs2812Heartbeat) {
                setWs2812Heartbeat(heartbeatState);
            } else {
                gpio_put(heartbeatPin, heartbeatState);
            }
            lastHeartbeat = now;
        }
    }
}
