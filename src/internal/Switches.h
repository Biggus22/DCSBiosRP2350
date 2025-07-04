#ifndef __DCSBIOS_SWITCHES_H
#define __DCSBIOS_SWITCHES_H

#include <math.h>
#include "pico/stdlib.h"
#include "aw9523b.h"
#include "MCP23S17.h" // Include MCP23S17 header
#include "pcf8575.h" // Include PCF8575 header

// Define polling interval constants if they are not already defined
#ifndef POLL_EVERY_TIME
#define POLL_EVERY_TIME 0
#endif

#ifndef POLL_INTERVAL_100_MS
#define POLL_INTERVAL_100_MS 100
#endif

namespace DcsBios {

    template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
    class Switch2PosT : PollingInput, public ResettableInput {
    private:
        const char* msg_;
        char pin_;
        AW9523B* awExpander_;
        MCP23S17* mcpExpander_;
        PCF8575* pcfExpander_;
        bool useAwExpander_;
        bool useMcpExpander_;
        bool usePcfExpander_;
        uint8_t expanderPin_;
        char debounceSteadyState_;
        char lastState_;
        bool reverse_;
        unsigned long debounceDelay_;
        unsigned long lastDebounceTime = 0;

    protected: // This section is for internal helper methods
        char readInput() {
            if (useAwExpander_ && awExpander_) return awExpander_->readPin(expanderPin_);
            if (useMcpExpander_ && mcpExpander_) return mcpExpander_->digitalRead(expanderPin_);
            if (usePcfExpander_ && pcfExpander_) return pcfExpander_->digitalRead(expanderPin_);
            return gpio_get(pin_);
        }

    public: // Changed access specifier for overridden methods
        void resetThisState() override {
            lastState_ = (lastState_ == 0) ? -1 : 0;
        }
        
        // Added override for PollingInput's resetState
        void resetState() override { 
            resetThisState(); // Call the specific reset logic
        }

        void pollInput() override {
            char state = readInput();
            if (reverse_) state = !state;
            unsigned long now = to_ms_since_boot(get_absolute_time());
            if (state != debounceSteadyState_) {
                lastDebounceTime = now;
                debounceSteadyState_ = state;
            }
            if ((now - lastDebounceTime) >= debounceDelay_) {
                if (state != lastState_) {
                    if (tryToSendDcsBiosMessage(msg_, state == 0 ? "0" : "1")) {
                        lastState_ = state;
                    }
                }
            }
        }

    public:
        Switch2PosT(const char* msg, char pin, bool reverse = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pin_(pin), reverse_(reverse), debounceDelay_(debounceDelay),
            awExpander_(nullptr), mcpExpander_(nullptr), pcfExpander_(nullptr), 
            useAwExpander_(false), useMcpExpander_(false), usePcfExpander_(false) {
            gpio_init(pin_);
            gpio_pull_up(pin_);
            gpio_set_dir(pin_, GPIO_IN);
            debounceSteadyState_ = readInput();
            if (reverse_) debounceSteadyState_ = !debounceSteadyState_;
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
        }

        Switch2PosT(const char* msg, AW9523B* expander, uint8_t expanderPin, bool reverse = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), expanderPin_(expanderPin), reverse_(reverse), debounceDelay_(debounceDelay),
            awExpander_(expander), useAwExpander_(true),
            mcpExpander_(nullptr), useMcpExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            debounceSteadyState_ = readInput();
            if (reverse_) debounceSteadyState_ = !debounceSteadyState_;
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
        }

        Switch2PosT(const char* msg, MCP23S17* expander, uint8_t expanderPin, bool reverse = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), expanderPin_(expanderPin), reverse_(reverse), debounceDelay_(debounceDelay),
            mcpExpander_(expander), useMcpExpander_(true),
            awExpander_(nullptr), useAwExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            debounceSteadyState_ = readInput();
            if (reverse_) debounceSteadyState_ = !debounceSteadyState_;
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
        }

        Switch2PosT(const char* msg, PCF8575* expander, uint8_t expanderPin, bool reverse = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), expanderPin_(expanderPin), reverse_(reverse), debounceDelay_(debounceDelay),
            pcfExpander_(expander), usePcfExpander_(true),
            awExpander_(nullptr), useAwExpander_(false),
            mcpExpander_(nullptr), useMcpExpander_(false) {
            debounceSteadyState_ = readInput();
            if (reverse_) debounceSteadyState_ = !debounceSteadyState_;
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
        }
    };

    template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
    class SwitchWithCover2PosT : PollingInput, public ResettableInput {
    private:
        const char* msg_;
        char pin_;
        char coverPin_;
        AW9523B* awExpander_;
        MCP23S17* mcpExpander_;
        PCF8575* pcfExpander_;
        bool useAwExpander_;
        bool useMcpExpander_;
        bool usePcfExpander_;
        uint8_t expPin_;
        uint8_t expCoverPin_;

        char debounceSteadyState_;
        char lastState_;
        char lastCoverState_;
        unsigned long debounceDelay_;
        unsigned long coverDelayMs_;
        unsigned long lastDebounceTime = 0;
        unsigned long lastCoverChangeTime_ = 0;
        bool coverOpenHandled_ = false;

    protected: // This section is for internal helper methods
        char readInput() {
            if (useAwExpander_ && awExpander_) return awExpander_->readPin(expPin_);
            if (useMcpExpander_ && mcpExpander_) return mcpExpander_->digitalRead(expPin_);
            if (usePcfExpander_ && pcfExpander_) return pcfExpander_->digitalRead(expPin_);
            return gpio_get(pin_);
        }

        char readCoverInput() {
            if (useAwExpander_ && awExpander_) return awExpander_->readPin(expCoverPin_);
            if (useMcpExpander_ && mcpExpander_) return mcpExpander_->digitalRead(expCoverPin_);
            if (usePcfExpander_ && pcfExpander_) return pcfExpander_->digitalRead(expCoverPin_);
            return gpio_get(coverPin_);
        }

    public: // Changed access specifier for overridden methods
        void resetThisState() override {
            lastState_ = (lastState_ == 0) ? -1 : 0;
            lastCoverState_ = (lastCoverState_ == 0) ? -1 : 0;
        }

        // Added override for PollingInput's resetState
        void resetState() override { 
            resetThisState(); // Call the specific reset logic
        }

        void pollInput() override {
            char state = readInput();
            char coverState = readCoverInput();
            unsigned long now = to_ms_since_boot(get_absolute_time());

            if (coverState == 0) { // Cover is open (active low)
                if (lastCoverState_ != coverState) {
                    lastCoverChangeTime_ = now;
                    coverOpenHandled_ = false;
                }
                lastCoverState_ = coverState;

                if ((now - lastCoverChangeTime_) >= coverDelayMs_ && !coverOpenHandled_) {
                    if (state != debounceSteadyState_) {
                        lastDebounceTime = now;
                        debounceSteadyState_ = state;
                    }
                    if ((now - lastDebounceTime) >= debounceDelay_) {
                        if (state != lastState_) {
                            if (tryToSendDcsBiosMessage(msg_, state == 0 ? "0" : "1")) {
                                lastState_ = state;
                                coverOpenHandled_ = true;
                            }
                        }
                    }
                }
            } else { // Cover is closed
                lastCoverState_ = coverState;
                debounceSteadyState_ = state;
                lastState_ = state == 0 ? -1 : 0;
                coverOpenHandled_ = false;
            }
        }

    public:
        SwitchWithCover2PosT(const char* msg, char pin, char coverPin, unsigned long debounceDelay = 50, unsigned long coverDelay = 200) :
            PollingInput(pollIntervalMs), msg_(msg), pin_(pin), coverPin_(coverPin), debounceDelay_(debounceDelay), coverDelayMs_(coverDelay),
            awExpander_(nullptr), mcpExpander_(nullptr), pcfExpander_(nullptr), 
            useAwExpander_(false), useMcpExpander_(false), usePcfExpander_(false) {
            gpio_init(pin_); gpio_pull_up(pin_); gpio_set_dir(pin_, GPIO_IN);
            gpio_init(coverPin_); gpio_pull_up(coverPin_); gpio_set_dir(coverPin_, GPIO_IN);
            debounceSteadyState_ = readInput();
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
            lastCoverState_ = readCoverInput();
        }

        SwitchWithCover2PosT(const char* msg, AW9523B* expander, uint8_t expanderPin, uint8_t expanderCoverPin, unsigned long debounceDelay = 50, unsigned long coverDelay = 200) :
            PollingInput(pollIntervalMs), msg_(msg), expPin_(expanderPin), expCoverPin_(expanderCoverPin), debounceDelay_(debounceDelay), coverDelayMs_(coverDelay),
            awExpander_(expander), useAwExpander_(true),
            mcpExpander_(nullptr), useMcpExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            debounceSteadyState_ = readInput();
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
            lastCoverState_ = readCoverInput();
        }

        SwitchWithCover2PosT(const char* msg, MCP23S17* expander, uint8_t expanderPin, uint8_t expanderCoverPin, unsigned long debounceDelay = 50, unsigned long coverDelay = 200) :
            PollingInput(pollIntervalMs), msg_(msg), expPin_(expanderPin), expCoverPin_(expanderCoverPin), debounceDelay_(debounceDelay), coverDelayMs_(coverDelay),
            mcpExpander_(expander), useMcpExpander_(true),
            awExpander_(nullptr), useAwExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            debounceSteadyState_ = readInput();
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
            lastCoverState_ = readCoverInput();
        }

        SwitchWithCover2PosT(const char* msg, PCF8575* expander, uint8_t expanderPin, uint8_t expanderCoverPin, unsigned long debounceDelay = 50, unsigned long coverDelay = 200) :
            PollingInput(pollIntervalMs), msg_(msg), expPin_(expanderPin), expCoverPin_(expanderCoverPin), debounceDelay_(debounceDelay), coverDelayMs_(coverDelay),
            pcfExpander_(expander), usePcfExpander_(true),
            awExpander_(nullptr), useAwExpander_(false),
            mcpExpander_(nullptr), useMcpExpander_(false) {
            debounceSteadyState_ = readInput();
            lastState_ = debounceSteadyState_ == 0 ? -1 : 0;
            lastCoverState_ = readCoverInput();
        }
    };

    // Template for multi-position switch
    // MODIFIED: Reordered template parameters so numPositions is first
    template <int numPositions, unsigned long pollIntervalMs = POLL_EVERY_TIME>
    class SwitchMultiPosT : PollingInput, public ResettableInput {
    private:
        const char* msg_;
        const uint8_t* pins_; // Array of pins
        AW9523B* awExpander_;
        MCP23S17* mcpExpander_;
        PCF8575* pcfExpander_;
        bool useAwExpander_;
        bool useMcpExpander_;
        bool usePcfExpander_;
        unsigned long debounceDelay_;
        unsigned long lastDebounceTime_ = 0;
        char lastState_ = -1;

    protected: // This section is for internal helper methods
        char readInput(uint8_t index) {
            if (useAwExpander_ && awExpander_) return awExpander_->readPin(pins_[index]);
            if (useMcpExpander_ && mcpExpander_) return mcpExpander_->digitalRead(pins_[index]);
            if (usePcfExpander_ && pcfExpander_) return pcfExpander_->digitalRead(pins_[index]);
            return gpio_get(pins_[index]);
        }

    public: // Changed access specifier for overridden methods
        void resetThisState() override {
            lastState_ = -1;
        }

        // Added override for PollingInput's resetState
        void resetState() override { 
            resetThisState(); // Call the specific reset logic
        }

        void pollInput() override {
            unsigned long now = to_ms_since_boot(get_absolute_time());
            for (int i = 0; i < numPositions; ++i) {
                char state = readInput(i);
                if (state == 0) {
                    if (i != lastState_ && (now - lastDebounceTime_) >= debounceDelay_) {
                        char msgBuffer[3];
                        snprintf(msgBuffer, sizeof(msgBuffer), "%d", i);
                        if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
                            lastState_ = i;
                            lastDebounceTime_ = now;
                        }
                    }
                    break;
                }
            }
        }
    
    public:
        // Constructor for direct Pico pins
        SwitchMultiPosT(const char* msg, const uint8_t* pins, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), debounceDelay_(debounceDelay),
            awExpander_(nullptr), mcpExpander_(nullptr), pcfExpander_(nullptr), 
            useAwExpander_(false), useMcpExpander_(false), usePcfExpander_(false) {
            for (int i = 0; i < numPositions; ++i) {
                gpio_init(pins[i]);
                gpio_pull_up(pins[i]);
                gpio_set_dir(pins[i], GPIO_IN);
            }
            resetThisState();
        }
    
        // Constructor for AW9523B expander
        SwitchMultiPosT(const char* msg, AW9523B* expander, const uint8_t* pins, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), debounceDelay_(debounceDelay),
            awExpander_(expander), useAwExpander_(true),
            mcpExpander_(nullptr), useMcpExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            resetThisState();
        }

        // Constructor for MCP23S17 expander
        SwitchMultiPosT(const char* msg, MCP23S17* expander, const uint8_t* pins, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), debounceDelay_(debounceDelay),
            mcpExpander_(expander), useMcpExpander_(true),
            awExpander_(nullptr), useAwExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            resetThisState();
        }

        // Constructor for PCF8575 expander
        SwitchMultiPosT(const char* msg, PCF8575* expander, const uint8_t* pins, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), debounceDelay_(debounceDelay),
            pcfExpander_(expander), usePcfExpander_(true),
            awExpander_(nullptr), useAwExpander_(false),
            mcpExpander_(nullptr), useMcpExpander_(false) {
            resetThisState();
        }
    };

    // Aliases for common use cases (direct pins)
    typedef Switch2PosT<> Switch2Pos;
    typedef Switch2PosT<POLL_INTERVAL_100_MS> Switch2Pos100ms;

    typedef SwitchWithCover2PosT<> SwitchWithCover2Pos;
    typedef SwitchWithCover2PosT<POLL_INTERVAL_100_MS> SwitchWithCover2Pos100ms;

    // MODIFIED: Updated aliases to reflect reordered template parameters
    template <int numPositions> using SwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using SwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

    // Aliases for AW9523B expander use
    typedef Switch2PosT<> AW9523BSwitch2Pos;
    typedef SwitchWithCover2PosT<> AW9523BSwitchWithCover2Pos;
    // MODIFIED: Updated aliases to reflect reordered template parameters
    template <int numPositions> using AW9523BSwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using AW9523BSwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

    // Aliases for MCP23S17 expander use
    typedef Switch2PosT<> MCP23S17Switch2Pos;
    typedef SwitchWithCover2PosT<> MCP23S17SwitchWithCover2Pos;
    // MODIFIED: Updated aliases to reflect reordered template parameters
    template <int numPositions> using MCP23S17SwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using MCP23S17SwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

    // Aliases for PCF8575 expander use
    typedef Switch2PosT<> PCF8575Switch2Pos;
    typedef SwitchWithCover2PosT<> PCF8575SwitchWithCover2Pos;
    // Updated aliases to reflect reordered template parameters
    template <int numPositions> using PCF8575SwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using PCF8575SwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

} // namespace DcsBios

#endif // __DCSBIOS_SWITCHES_H