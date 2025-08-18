// This is a test file for some different switch implementations
// It includes classes for 2-position switches, switches with covers, and multi-position switches.  
#ifndef __DCSBIOS_SWITCHESV2_H
#define __DCSBIOS_SWITCHESV2_H

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
    template <int numPositions, unsigned long pollIntervalMs = POLL_EVERY_TIME>
    class SwitchMultiPosT : PollingInput, public ResettableInput {
    private:
        const char* msg_;
        const uint8_t* pins_; // Array of pins
        bool isMomentary_; // Flag to indicate a momentary switch
        AW9523B* awExpander_;
        MCP23S17* mcpExpander_;
        PCF8575* pcfExpander_;
        bool useAwExpander_;
        bool useMcpExpander_;
        bool usePcfExpander_;
        unsigned long debounceDelay_;
        unsigned long lastDebounceTime_ = 0;
        char lastState_ = -1;
        char currentSteadyState_ = -1;

    protected:
        char readInput(uint8_t index) {
            if (useAwExpander_ && awExpander_) return awExpander_->readPin(pins_[index]);
            if (useMcpExpander_ && mcpExpander_) return mcpExpander_->digitalRead(pins_[index]);
            if (usePcfExpander_ && pcfExpander_) return pcfExpander_->digitalRead(pins_[index]);
            return gpio_get(pins_[index]);
        }

        char determinePosition() {
            if (numPositions == 3) {
                char pin0 = readInput(0);
                char pin1 = readInput(1);
                if (pin0 == 0 && pin1 == 1) return 0;
                if (pin0 == 1 && pin1 == 1) return 1;
                if (pin0 == 1 && pin1 == 0) return 2;
                return -1;
            }
            for (int i = 0; i < numPositions; ++i) {
                if (readInput(i) == 0) {
                    return i;
                }
            }
            return -1;
        }

    public:
        void resetThisState() override {
            lastState_ = -1;
            currentSteadyState_ = -1;
        }

        void resetState() override { 
            resetThisState();
        }

        void pollInput() override {
            unsigned long now = to_ms_since_boot(get_absolute_time());
            char currentPosition = determinePosition();
            
            if (currentPosition != currentSteadyState_) {
                lastDebounceTime_ = now;
                currentSteadyState_ = currentPosition;
            }
            
            if ((now - lastDebounceTime_) >= debounceDelay_ && currentPosition != lastState_) {
                bool shouldSend = false;
                if (!isMomentary_) {
                    if (currentPosition != -1) {
                        shouldSend = true;
                    }
                } else { // isMomentary_ == true
                    // For momentary 3-pos switches, only send for positions 0 and 2.
                    if (numPositions == 3 && (currentPosition == 0 || currentPosition == 2)) {
                        shouldSend = true;
                    }
                    // For other momentary switches, send if any pin is active.
                    else if (numPositions != 3 && currentPosition != -1) {
                        shouldSend = true;
                    }
                }

                if (shouldSend) {
                    char msgBuffer[3];
                    snprintf(msgBuffer, sizeof(msgBuffer), "%d", currentPosition);
                    if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
                        lastState_ = currentPosition;
                    }
                } else {
                    // If not sending (e.g., momentary switch released to middle),
                    // still update lastState_ to detect the next press.
                    lastState_ = currentPosition;
                }
            }
        }
    
    public:
        // Constructor for direct Pico pins
        SwitchMultiPosT(const char* msg, const uint8_t* pins, bool momentary = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), isMomentary_(momentary), debounceDelay_(debounceDelay),
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
        SwitchMultiPosT(const char* msg, AW9523B* expander, const uint8_t* pins, bool momentary = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), isMomentary_(momentary), debounceDelay_(debounceDelay),
            awExpander_(expander), useAwExpander_(true),
            mcpExpander_(nullptr), useMcpExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            resetThisState();
        }

        // Constructor for MCP23S17 expander
        SwitchMultiPosT(const char* msg, MCP23S17* expander, const uint8_t* pins, bool momentary = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), isMomentary_(momentary), debounceDelay_(debounceDelay),
            mcpExpander_(expander), useMcpExpander_(true),
            awExpander_(nullptr), useAwExpander_(false),
            pcfExpander_(nullptr), usePcfExpander_(false) {
            resetThisState();
        }

        // Constructor for PCF8575 expander
        SwitchMultiPosT(const char* msg, PCF8575* expander, const uint8_t* pins, bool momentary = false, unsigned long debounceDelay = 50) :
            PollingInput(pollIntervalMs), msg_(msg), pins_(pins), isMomentary_(momentary), debounceDelay_(debounceDelay),
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

    template <int numPositions> using SwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using SwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

    // Aliases for AW9523B expander use
    typedef Switch2PosT<> AW9523BSwitch2Pos;
    typedef SwitchWithCover2PosT<> AW9523BSwitchWithCover2Pos;
    template <int numPositions> using AW9523BSwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using AW9523BSwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

    // Aliases for MCP23S17 expander use
    typedef Switch2PosT<> MCP23S17Switch2Pos;
    typedef SwitchWithCover2PosT<> MCP23S17SwitchWithCover2Pos;
    template <int numPositions> using MCP23S17SwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using MCP23S17SwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

    // Aliases for PCF8575 expander use
    typedef Switch2PosT<> PCF8575Switch2Pos;
    typedef SwitchWithCover2PosT<> PCF8575SwitchWithCover2Pos;
    template <int numPositions> using PCF8575SwitchMultiPos = SwitchMultiPosT<numPositions, POLL_EVERY_TIME>;
    template <int numPositions> using PCF8575SwitchMultiPos100ms = SwitchMultiPosT<numPositions, POLL_INTERVAL_100_MS>;

} // namespace DcsBios

#endif // __DCSBIOS_SWITCHES_H
