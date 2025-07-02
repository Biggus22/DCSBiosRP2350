// Modified header version with AW9523B support for RotaryEncoderT and RotaryAcceleratedEncoderT only
#ifndef __DCSBIOS_ENCODERS_H
#define __DCSBIOS_ENCODERS_H

#include "pico/stdlib.h"
#include "aw9523b.h"
#include "internal/MCP23S17.h" // Include MCP23S17 header

// Define polling interval constants if they are not already defined
#ifndef POLL_EVERY_TIME
#define POLL_EVERY_TIME 0
#endif

#ifndef POLL_INTERVAL_100_MS
#define POLL_INTERVAL_100_MS 100
#endif

// tryToSendDcsBiosMessage is expected to be part of DCSBIOS.h or a globally visible header.

namespace DcsBios {

    enum StepsPerDetent {
        ONE_STEP_PER_DETENT = 1,
        TWO_STEPS_PER_DETENT = 2,
        FOUR_STEPS_PER_DETENT = 4,
        EIGHT_STEPS_PER_DETENT = 8,
    };

    template <unsigned long pollIntervalMs = POLL_EVERY_TIME, StepsPerDetent stepsPerDetent = ONE_STEP_PER_DETENT>
    class RotaryEncoderT : PollingInput, public ResettableInput {
    private:
        const char* msg_;
        const char* decArg_;
        const char* incArg_;
        char pinA_;
        char pinB_;
        AW9523B* expander_ = nullptr;
        uint8_t expPinA_;
        uint8_t expPinB_;
        bool useExpander_ = false;

        MCP23S17* mcpExpander_ = nullptr; // New: MCP23S17 expander pointer
        uint8_t mcpExpPinA_;
        uint8_t mcpExpPinB_;
        bool useMcpExpander_ = false; // New: Flag for MCP23S17 expander

        char lastState_; // The last *debounced* state
        volatile int delta_;

        // *** DEBOUNCE MEMBERS ***
        unsigned long debounceDelay_; // Configurable debounce delay in milliseconds
        unsigned long lastChangeTime_; // Timestamp of the last raw pin state change
        char lastRawState_;           // The last *raw* state read from the pins
        // ************************

        char readState() {
            if (useExpander_ && expander_) {
                return (char)(expander_->readPin(expPinA_) << 1 | expander_->readPin(expPinB_));
            } else if (useMcpExpander_ && mcpExpander_) {
                return (char)(mcpExpander_->digitalRead(mcpExpPinA_) << 1 | mcpExpander_->digitalRead(mcpExpPinB_));
            } else {
                return (char)(gpio_get(pinA_) << 1 | gpio_get(pinB_));
            }
        }

    public: // pollInput() and reset() are public
        void pollInput() override {
            unsigned long now = to_ms_since_boot(get_absolute_time());
            char currentRawState = readState(); // Get current raw state

            // Detect if the raw state has changed since last check
            if (currentRawState != lastRawState_) {
                lastChangeTime_ = now;       // Record the time of this raw change
                lastRawState_ = currentRawState; // Update the last raw state
            }

            // Only proceed if the input has been stable for at least debounceDelay_
            // and the stable state is different from the last *processed* state.
            if ((now - lastChangeTime_) >= debounceDelay_) {
                if (currentRawState != lastState_) { // currentRawState is now considered stable
                    // Process the encoder logic based on the stable state change
                    // This is your existing encoder state machine logic
                    if (lastState_ == 0b00) {
                        if (currentRawState == 0b01) delta_++;
                        else if (currentRawState == 0b10) delta_--;
                    } else if (lastState_ == 0b01) {
                        if (currentRawState == 0b11) delta_++;
                        else if (currentRawState == 0b00) delta_--;
                    } else if (lastState_ == 0b11) {
                        if (currentRawState == 0b10) delta_++;
                        else if (currentRawState == 0b01) delta_--;
                    } else if (lastState_ == 0b10) {
                        if (currentRawState == 0b00) delta_++;
                        else if (currentRawState == 0b11) delta_--;
                    }
                    
                    lastState_ = currentRawState; // Update the last *processed/stable* state
                }
            }
            
            // Use tryToSendDcsBiosMessage
            if (delta_ >= stepsPerDetent) {
                if (tryToSendDcsBiosMessage(msg_, incArg_)) delta_ -= stepsPerDetent;
            }
            if (delta_ <= -stepsPerDetent) {
                if (tryToSendDcsBiosMessage(msg_, decArg_)) delta_ += stepsPerDetent;
            }
        }

        // Constructor for native Pico pins
        // Added debounceDelay parameter with a default value
        RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, char pinA, char pinB, unsigned long debounceDelay = 5)
            : PollingInput(pollIntervalMs), msg_(msg), decArg_(decArg), incArg_(incArg), pinA_(pinA), pinB_(pinB),
              useExpander_(false), useMcpExpander_(false), delta_(0),
              debounceDelay_(debounceDelay), lastChangeTime_(0) // Initialize new members
        {
            gpio_init(pinA_); gpio_pull_up(pinA_); gpio_set_dir(pinA_, GPIO_IN);
            gpio_init(pinB_); gpio_pull_up(pinB_); gpio_set_dir(pinB_, GPIO_IN);
            lastRawState_ = readState(); // Initialize last raw state
            lastState_ = lastRawState_;  // Initialize debounced state
        }

        // Constructor for AW9523B expander
        // Added debounceDelay parameter with a default value
        RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, AW9523B* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 5)
            : PollingInput(pollIntervalMs), expander_(expander), expPinA_(pinA), expPinB_(pinB), useExpander_(true),
              mcpExpander_(nullptr), useMcpExpander_(false), delta_(0),
              debounceDelay_(debounceDelay), lastChangeTime_(0) // Initialize new members
        {
            msg_ = msg;
            decArg_ = decArg;
            incArg_ = incArg;
            lastRawState_ = readState(); // Initialize last raw state
            lastState_ = lastRawState_;  // Initialize debounced state
        }

        // New constructor for MCP23S17 expander
        // Added debounceDelay parameter with a default value
        RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, MCP23S17* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 5)
            : PollingInput(pollIntervalMs), useExpander_(false), // Not using AW9523B expander
              mcpExpander_(expander), mcpExpPinA_(pinA), mcpExpPinB_(pinB), useMcpExpander_(true), delta_(0),
              debounceDelay_(debounceDelay), lastChangeTime_(0) // Initialize new members
        {
            msg_ = msg;
            decArg_ = decArg;
            incArg_ = incArg;
            lastRawState_ = readState(); // Initialize last raw state
            lastState_ = lastRawState_;  // Initialize debounced state
        }
        
        // *** Implement pure virtual functions from base classes ***
        void resetState() override { // From PollingInput
            resetThisState();
        }

        void resetThisState() override { // From ResettableInput
            delta_ = 0;
            lastRawState_ = readState();
            lastState_ = lastRawState_;
            lastChangeTime_ = to_ms_since_boot(get_absolute_time());
        }
        // *********************************************************

        // Convenience reset method, calls the actual state reset
        void reset() {
            resetThisState();
        }
    };
    // ... (RotaryAcceleratedEncoderT and aliases would follow here, modified similarly if needed)
    
    // Aliases for MCP23S17 expander use
    typedef RotaryEncoderT<POLL_EVERY_TIME, ONE_STEP_PER_DETENT> MCP23S17RotaryEncoderOneStep;
    typedef RotaryEncoderT<POLL_EVERY_TIME, TWO_STEPS_PER_DETENT> MCP23S17RotaryEncoderTwoSteps;
    typedef RotaryEncoderT<POLL_EVERY_TIME, FOUR_STEPS_PER_DETENT> MCP23S17RotaryEncoderFourSteps;
    typedef RotaryEncoderT<POLL_EVERY_TIME, EIGHT_STEPS_PER_DETENT> MCP23S17RotaryEncoderEightSteps;

} // namespace DcsBios

#endif // __DCSBIOS_ENCODERS_H