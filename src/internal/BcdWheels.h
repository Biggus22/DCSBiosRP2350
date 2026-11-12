#ifndef __DCSBIOS_BCDWHEELS_H
#define __DCSBIOS_BCDWHEELS_H

#include <stdint.h>
#include <cstdio>
#include "pico/stdlib.h"
#include "PollingInput.h"
#include "ExportStreamListener.h"

namespace DcsBios {

template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class BcdWheelT : public PollingInput, public ResettableInput {
private:
    const char* msg_;
    uint pinA_, pinB_, pinC_, pinD_;
    char lastState_;

    char readState() {
        int total = 0;
        if (gpio_get(pinA_) == 0) total += 1;
        if (gpio_get(pinB_) == 0) total += 2;
        if (pinC_ != 255 && gpio_get(pinC_) == 0) total += 4;
        if (pinD_ != 255 && gpio_get(pinD_) == 0) total += 8;
        return total;
    }

    void resetState() {
        lastState_ = (lastState_ == 0) ? -1 : 0;
    }

    void pollInput() {
        char state = readState();
        if (state != lastState_) {
            char szBody[2] = { static_cast<char>(state + '0'), 0 };
            if (tryToSendDcsBiosMessage(msg_, szBody)) lastState_ = state;
        }
    }

public:
    BcdWheelT(const char* msg, uint pinA, uint pinB, uint pinC = 255, uint pinD = 255) :
        PollingInput(pollIntervalMs), msg_(msg),
        pinA_(pinA), pinB_(pinB), pinC_(pinC), pinD_(pinD)
    {
        gpio_init(pinA_); gpio_pull_up(pinA_);
        gpio_init(pinB_); gpio_pull_up(pinB_);
        if (pinC_ != 255) { gpio_init(pinC_); gpio_pull_up(pinC_); }
        if (pinD_ != 255) { gpio_init(pinD_); gpio_pull_up(pinD_); }
        lastState_ = readState();
    }

    void resetThisState() { this->resetState(); }
};
typedef BcdWheelT<> BcdWheel;

// Four-line coded rotary switch that reports its position as a number.
// BcdWheelT only emits a single decimal digit, which cannot express positions
// above 9. Pins are active LOW (switch common to GND) and weighted 1/2/4/8.
// Detents above maxPosition are never reported, so a 16-position switch
// driving a 13-position control simply ignores its last three positions.
template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class BcdWheelValueT : public PollingInput, public ResettableInput {
private:
    const char* msg_;
    uint pinA_, pinB_, pinC_, pinD_;
    unsigned int maxPosition_;
    int lastState_;
    int debounceCandidate_;
    unsigned long debounceDelay_;
    unsigned long lastDebounceTime_ = 0;

    int readState_() const {
        int total = 0;
        if (gpio_get(pinA_) == 0) total += 1;
        if (gpio_get(pinB_) == 0) total += 2;
        if (gpio_get(pinC_) == 0) total += 4;
        if (gpio_get(pinD_) == 0) total += 8;
        return total;
    }

    void resetState() {
        lastState_ = -1;
        debounceCandidate_ = -1;
        lastDebounceTime_ = 0;
    }

    void pollInput() {
        const int state = readState_();
        if ((unsigned int)state > maxPosition_) return;  // detent outside the control's range

        const unsigned long now = to_ms_since_boot(get_absolute_time());
        if (state != debounceCandidate_) {
            debounceCandidate_ = state;
            lastDebounceTime_ = now;
        }
        if ((now - lastDebounceTime_) >= debounceDelay_) {
            if (debounceCandidate_ != lastState_) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%d", debounceCandidate_);
                if (tryToSendDcsBiosMessage(msg_, buf)) lastState_ = debounceCandidate_;
            }
        }
    }

public:
    BcdWheelValueT(const char* msg, uint pinA, uint pinB, uint pinC, uint pinD,
                   unsigned int maxPosition = 15, unsigned long debounceDelay = 50)
        : PollingInput(pollIntervalMs), msg_(msg),
          pinA_(pinA), pinB_(pinB), pinC_(pinC), pinD_(pinD),
          maxPosition_(maxPosition), lastState_(-1), debounceCandidate_(-1),
          debounceDelay_(debounceDelay)
    {
        gpio_init(pinA_); gpio_pull_up(pinA_);
        gpio_init(pinB_); gpio_pull_up(pinB_);
        gpio_init(pinC_); gpio_pull_up(pinC_);
        gpio_init(pinD_); gpio_pull_up(pinD_);
        lastState_ = readState_();
    }

    void resetThisState() { this->resetState(); }
    int getState() const { return lastState_; }
};
typedef BcdWheelValueT<> BcdWheelValue;

template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class RadioPresetT : public PollingInput, public ResettableInput {
private:
    const char* msg_;
    uint pinA_, pinB_, pinC_, pinD_, pinE_;
    char lastState_;

    char readState() {
        int total = 0;
        if (gpio_get(pinA_) == 0) total += 1;
        if (gpio_get(pinB_) == 0) total += 2;
        if (gpio_get(pinC_) == 0) total += 4;
        if (gpio_get(pinD_) == 0) total += 8;
        if (gpio_get(pinE_) == 0) total += 16;
        return total;
    }

    void resetState() {
        lastState_ = (lastState_ == 0) ? -1 : 0;
    }

    void pollInput() {
        static const char* values[] = {
            "0", "0.05", "0.1", "0.15", "0.2", "0.25", "0.3", "0.35",
            "0.4", "0.45", "0.5", "0.55", "0.6", "0.65", "0.7", "0.75",
            "0.8", "0.85", "0.9", "0.95", "1"
        };
        char state = readState();
        if (state != lastState_ && state < 21) {
            if (tryToSendDcsBiosMessage(msg_, values[state])) {
                lastState_ = state;
            }
        }
    }

public:
    RadioPresetT(const char* msg, uint pinA, uint pinB, uint pinC, uint pinD, uint pinE) :
        PollingInput(pollIntervalMs),
        msg_(msg), pinA_(pinA), pinB_(pinB), pinC_(pinC), pinD_(pinD), pinE_(pinE)
    {
        gpio_init(pinA_); gpio_pull_up(pinA_);
        gpio_init(pinB_); gpio_pull_up(pinB_);
        gpio_init(pinC_); gpio_pull_up(pinC_);
        gpio_init(pinD_); gpio_pull_up(pinD_);
        gpio_init(pinE_); gpio_pull_up(pinE_);
        lastState_ = readState();
    }

    void resetThisState() { this->resetState(); }
};
typedef RadioPresetT<> RadioPreset;

} // namespace DcsBios

#endif
