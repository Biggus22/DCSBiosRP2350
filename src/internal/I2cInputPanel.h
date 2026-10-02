#ifndef __DCSBIOS_I2CINPUTPANEL_H
#define __DCSBIOS_I2CINPUTPANEL_H

#include <stdint.h>
#include <cstdio>

#include "pico/stdlib.h"

#include "I2cBus.h"
#include "I2cFrame.h"
#include "PollingInput.h"

namespace DcsBios {

// Byte offsets inside a remote panel's STATUS_REQ response payload.
// A panel is free to omit trailing fields; the poller validates against the
// payload length it was constructed with.
enum I2cInputField : uint8_t {
    I2C_INPUT_STATE = 0,  // 1 byte bitfield of active-LOW input pins
    I2C_INPUT_ADC_0 = 1,  // 2 bytes little-endian, raw 10-bit
    I2C_INPUT_ADC_1 = 3,  // 2 bytes little-endian, raw 10-bit
};

// Polls a remote I2C input panel and caches its latest decoded state.
//
// The existing I2C subsystem is write-only: I2cOutputListener moves sim->panel
// data and only fires on changes, whereas panel->sim inputs have to be polled.
// This class owns the transaction; the I2cInput* input classes below read the
// cache and turn it into DCS-BIOS messages with the same debounce / smoothing
// semantics as the GPIO-backed classes in Switches.h and Potentiometers.h.
class I2cInputPanel {
public:
    static constexpr uint8_t MAX_ADC_CHANNELS = 2;
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = 10;
    // Consecutive failures between bus-recovery attempts.
    static constexpr uint32_t RECOVER_BUS_EVERY = 20;

    I2cInputPanel(I2cBus *bus, uint8_t slaveAddr, uint8_t regId, uint8_t cmd,
                   uint8_t payloadLen, uint32_t pollIntervalMs = 20,
                   uint32_t timeoutMs = DEFAULT_TIMEOUT_MS);

    // Rate limited: performs at most one transaction per pollIntervalMs.
    // Call once per pass through DcsBios::loop(), before it.
    void poll();

    bool online() const { return online_; }
    uint32_t failCount() const { return failCount_; }
    uint8_t readState() const { return state_; }
    bool readAdc(uint8_t field, uint16_t *out) const;

    uint8_t slaveAddr() const { return slaveAddr_; }
    uint8_t payloadLen() const { return payloadLen_; }

private:
    static bool decode_(const uint8_t *rx, uint8_t rxLen,
                        uint8_t expectedReg, uint8_t expectedCmd,
                        uint8_t expectedPayloadLen,
                        uint8_t *state, uint16_t *adc);

    I2cBus *bus_;
    uint8_t slaveAddr_;
    uint8_t regId_;
    uint8_t cmd_;
    uint8_t payloadLen_;
    uint32_t pollIntervalMs_;
    uint32_t timeoutMs_;
    unsigned long lastPollMs_;
    uint8_t state_;
    uint16_t adc_[MAX_ADC_CHANNELS];
    bool online_;
    uint32_t failCount_;
};

// Two-position switch on one bit of the panel's state byte. Mirrors Switch2PosT:
// pins are active LOW, and the DCS-BIOS value is the inverse of the raw level.
template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class I2cInputSwitch2PosT : PollingInput, public ResettableInput {
private:
    const char *msg_;
    I2cInputPanel *panel_;
    uint8_t bit_;
    bool reverse_;
    char debounceSteadyState_;
    char lastState_;
    unsigned long debounceDelay_;
    unsigned long lastDebounceTime_ = 0;

    void resetState() { lastState_ = (lastState_ == 0) ? -1 : 0; }

    void pollInput() {
        if (!panel_->online()) return;

        char state = ((panel_->readState() & bit_) == 0) ? 1 : 0;
        if (reverse_) state = !state;

        unsigned long now = to_ms_since_boot(get_absolute_time());
        if (state != debounceSteadyState_) {
            lastDebounceTime_ = now;
            debounceSteadyState_ = state;
        }
        if ((now - lastDebounceTime_) >= debounceDelay_) {
            if (debounceSteadyState_ != lastState_) {
                if (tryToSendDcsBiosMessage(msg_, state == 1 ? "0" : "1")) {
                    lastState_ = debounceSteadyState_;
                }
            }
        }
    }

public:
    I2cInputSwitch2PosT(I2cInputPanel *panel, uint8_t bit, const char *msg,
                         bool reverse = false, unsigned long debounceDelay = 50)
        : PollingInput(pollIntervalMs), msg_(msg), panel_(panel), bit_(bit),
          reverse_(reverse), debounceSteadyState_(-1), lastState_(-1),
          debounceDelay_(debounceDelay) {}

    void SetControl(const char *msg) { msg_ = msg; }
    void resetThisState() override { this->resetState(); }
    bool getState() const { return lastState_ != 0; }
};

typedef I2cInputSwitch2PosT<> I2cInputSwitch2Pos;

// Three-position switch on two bits of the panel's state byte. Mirrors
// Switch3Pos2PinT: both bits low is the centre position, A thrown = 0,
// B thrown = 2, and "both active" is the in-between transit state, which holds
// the last reported position rather than reporting a spurious one.
template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class I2cInputSwitch3Pos2BitT : PollingInput, public ResettableInput {
private:
    const char *msg_;
    I2cInputPanel *panel_;
    uint8_t bitA_;
    uint8_t bitB_;
    int8_t lastState_;
    int8_t debounceCandidate_;
    unsigned long debounceDelay_;
    unsigned long lastDebounceTime_ = 0;
    unsigned long centerStableSince_ = 0;

    static int8_t mapLogical_(bool aActive, bool bActive) {
        if (aActive && bActive)    return -1;  // transient during travel: hold last
        if (aActive && !bActive)   return 0;   // A thrown
        if (!aActive && !bActive)  return 1;   // centre
        return 2;                              // B thrown
    }

    void resetState() {
        lastState_ = -1;
        debounceCandidate_ = -1;
        lastDebounceTime_ = 0;
        centerStableSince_ = 0;
    }

    void pollInput() {
        if (!panel_->online()) return;

        const uint8_t stateByte = panel_->readState();
        const bool aActive = (stateByte & bitA_) == 0;
        const bool bActive = (stateByte & bitB_) == 0;
        const unsigned long now = to_ms_since_boot(get_absolute_time());

        // Centre watchdog: both bits released and stable for debounceDelay_
        // forces centre=1 even if no edge was ever seen.
        if (!aActive && !bActive) {
            if (centerStableSince_ == 0) centerStableSince_ = now;
            if ((now - centerStableSince_) >= debounceDelay_) {
                if (lastState_ != 1) {
                    char buf[3];
                    snprintf(buf, sizeof(buf), "%d", 1);
                    if (tryToSendDcsBiosMessage(msg_, buf)) lastState_ = 1;
                }
                debounceCandidate_ = 1;
                lastDebounceTime_ = now;
                return;
            }
        } else {
            centerStableSince_ = 0;
        }

        const int8_t logical = mapLogical_(aActive, bActive);
        if (logical < 0) return;  // both active: hold last

        if (logical != debounceCandidate_) {
            debounceCandidate_ = logical;
            lastDebounceTime_ = now;
        }
        if ((now - lastDebounceTime_) >= debounceDelay_) {
            if (debounceCandidate_ != lastState_) {
                char buf[3];
                snprintf(buf, sizeof(buf), "%d", debounceCandidate_);
                if (tryToSendDcsBiosMessage(msg_, buf)) lastState_ = debounceCandidate_;
            }
        }
    }

public:
    I2cInputSwitch3Pos2BitT(I2cInputPanel *panel, uint8_t bitA, uint8_t bitB,
                             const char *msg, unsigned long debounceDelay = 50)
        : PollingInput(pollIntervalMs), msg_(msg), panel_(panel), bitA_(bitA),
          bitB_(bitB), lastState_(-1), debounceCandidate_(-1),
          debounceDelay_(debounceDelay) {}

    void SetControl(const char *msg) { msg_ = msg; }
    void resetThisState() override { this->resetState(); }
    int8_t getState() const { return lastState_; }
};

typedef I2cInputSwitch3Pos2BitT<> I2cInputSwitch3Pos2Bit;

// Potentiometer on one of the panel's ADC fields. Mirrors PotentiometerEWMA
// (EWMA smoothing, bounded step per poll, hysteresis, extremes always sent)
// but sources its raw value from the I2C snapshot instead of the local ADC.
template <unsigned long pollIntervalMs = POLL_EVERY_TIME,
          unsigned int hysteresis = 128, unsigned int ewmaDivisor = 5>
class I2cInputPotentiometerT : PollingInput, public ResettableInput {
private:
    const char *msg_;
    I2cInputPanel *panel_;
    uint8_t field_;
    bool reverse_;
    unsigned int input_min_;
    unsigned int input_max_;
    float accumulator;
    unsigned int lastState_;
    bool primed_;

    static unsigned int mapRaw_(unsigned int raw,
                                unsigned int inMin, unsigned int inMax,
                                unsigned int outMin, unsigned int outMax) {
        if (inMax == inMin) return outMin;
        long long span = (long long)outMax - (long long)outMin;
        long long v = (long long)outMin + ((long long)raw - (long long)inMin) * span /
                                            ((long long)inMax - (long long)inMin);
        if (v < 0) v = 0;
        if (v > 65535) v = 65535;
        return (unsigned int)v;
    }

    bool sendValue_(unsigned int value) {
        char buf[6];
        snprintf(buf, sizeof(buf), "%u", value);
        return tryToSendDcsBiosMessage(msg_, buf);
    }

    void resetState() { primed_ = false; }

    void pollInput() {
        if (!panel_->online()) return;

        uint16_t raw = 0;
        if (!panel_->readAdc(field_, &raw)) return;

        unsigned int state = reverse_
            ? mapRaw_(raw, input_min_, input_max_, 65535, 0)
            : mapRaw_(raw, input_min_, input_max_, 0, 65535);

        // First reading after construction or resetState(): adopt it whole.
        // Ramping from an unprimed accumulator would otherwise report a
        // fraction of the real value first.
        if (!primed_) {
            accumulator = (float)state;
            if (sendValue_(state)) {
                lastState_ = state;
                primed_ = true;
            }
            return;
        }

        accumulator += ((float)state - accumulator) / (float)ewmaDivisor;
        state = (unsigned int)accumulator;

        // Limit the maximum step sent per poll to avoid visible hitching.
        const unsigned int maxStep = 2048;

        unsigned int sendState = state;
        if (state > lastState_ + maxStep) {
            sendState = lastState_ + maxStep;
        } else if (state + maxStep < lastState_) {
            sendState = lastState_ - maxStep;
        }

        // Only send if the change exceeds hysteresis (or near the extremes).
        if (((lastState_ > sendState) && (lastState_ - sendState > hysteresis)) ||
            ((sendState > lastState_) && (sendState - lastState_ > hysteresis)) ||
            ((sendState > (65535 - hysteresis)) && (sendState > lastState_)) ||
            ((sendState < hysteresis) && (sendState < lastState_))) {
            if (sendValue_(sendState)) lastState_ = sendState;
        }
    }

public:
    I2cInputPotentiometerT(I2cInputPanel *panel, uint8_t field, const char *msg,
                            bool reverse = false, unsigned int input_min = 0,
                            unsigned int input_max = 1023)
        : PollingInput(pollIntervalMs), msg_(msg), panel_(panel), field_(field),
          reverse_(reverse), input_min_(input_min), input_max_(input_max),
          accumulator(0.0f), lastState_(0), primed_(false) {}

    void SetControl(const char *msg) { msg_ = msg; }
    void resetThisState() override { this->resetState(); }
};

typedef I2cInputPotentiometerT<> I2cInputPotentiometer;

} // namespace DcsBios

#endif
