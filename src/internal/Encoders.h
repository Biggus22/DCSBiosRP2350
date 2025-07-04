// Modified header version with AW9523B, MCP23S17, and PCF8575 support for RotaryEncoderT and RotaryAcceleratedEncoderT only
#ifndef __DCSBIOS_ENCODERS_H
#define __DCSBIOS_ENCODERS_H

#include "pico/stdlib.h"
#include "aw9523b.h"
#include "internal/MCP23S17.h" // Include MCP23S17 header
#include "pcf8575.h" // Include PCF8575 header

// Define polling interval constants if they are not already defined
#ifndef POLL_EVERY_TIME
#define POLL_EVERY_TIME 0
#endif

#ifndef POLL_INTERVAL_100_MS
#define POLL_INTERVAL_100_MS 100
#endif

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

        MCP23S17* mcpExpander_ = nullptr; // MCP23S17 expander pointer
        uint8_t mcpExpPinA_;
        uint8_t mcpExpPinB_;
        bool useMcpExpander_ = false; // Flag for MCP23S17 expander

        PCF8575* pcfExpander_ = nullptr; // PCF8575 expander pointer
        uint8_t pcfExpPinA_;
        uint8_t pcfExpPinB_;
        bool usePcfExpander_ = false; // Flag for PCF8575 expander

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
            } else if (usePcfExpander_ && pcfExpander_) {
                return (char)(pcfExpander_->digitalRead(pcfExpPinA_) << 1 | pcfExpander_->digitalRead(pcfExpPinB_));
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
              useExpander_(false), useMcpExpander_(false), usePcfExpander_(false), delta_(0),
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
              mcpExpander_(nullptr), useMcpExpander_(false), pcfExpander_(nullptr), usePcfExpander_(false), delta_(0),
              debounceDelay_(debounceDelay), lastChangeTime_(0) // Initialize new members
        {
            msg_ = msg;
            decArg_ = decArg;
            incArg_ = incArg;
            lastRawState_ = readState(); // Initialize last raw state
            lastState_ = lastRawState_;  // Initialize debounced state
        }

        // Constructor for MCP23S17 expander
        // Added debounceDelay parameter with a default value
        RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, MCP23S17* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 5)
            : PollingInput(pollIntervalMs), useExpander_(false), // Not using AW9523B expander
              mcpExpander_(expander), mcpExpPinA_(pinA), mcpExpPinB_(pinB), useMcpExpander_(true),
              pcfExpander_(nullptr), usePcfExpander_(false), delta_(0),
              debounceDelay_(debounceDelay), lastChangeTime_(0) // Initialize new members
        {
            msg_ = msg;
            decArg_ = decArg;
            incArg_ = incArg;
            lastRawState_ = readState(); // Initialize last raw state
            lastState_ = lastRawState_;  // Initialize debounced state
        }

        // Constructor for PCF8575 expander
        // Added debounceDelay parameter with a default value
        RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, PCF8575* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 5)
            : PollingInput(pollIntervalMs), useExpander_(false), // Not using AW9523B expander
              mcpExpander_(nullptr), useMcpExpander_(false),
              pcfExpander_(expander), pcfExpPinA_(pinA), pcfExpPinB_(pinB), usePcfExpander_(true), delta_(0),
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

    // NEW: Emulated Concentric Encoder Template Class
    template <unsigned long pollIntervalMs = POLL_EVERY_TIME, StepsPerDetent normalStepsPerDetent = TWO_STEPS_PER_DETENT, StepsPerDetent altStepsPerDetent = ONE_STEP_PER_DETENT>
    class EmulatedConcentricEncoderT : PollingInput, public ResettableInput {
    private:
        // Hardware configuration
        char buttonPin_;
        char encoderPinA_;
        char encoderPinB_;
        AW9523B* expander_ = nullptr;
        uint8_t expButtonPin_;
        uint8_t expEncoderPinA_;
        uint8_t expEncoderPinB_;
        bool useExpander_ = false;

        MCP23S17* mcpExpander_ = nullptr;
        uint8_t mcpExpButtonPin_;
        uint8_t mcpExpEncoderPinA_;
        uint8_t mcpExpEncoderPinB_;
        bool useMcpExpander_ = false;

        PCF8575* pcfExpander_ = nullptr;
        uint8_t pcfExpButtonPin_;
        uint8_t pcfExpEncoderPinA_;
        uint8_t pcfExpEncoderPinB_;
        bool usePcfExpander_ = false;

        // DCS-BIOS messages for normal operation (button not pressed)
        const char* normalMsg_;
        const char* normalDecArg_;
        const char* normalIncArg_;
        
        // DCS-BIOS messages for alternate operation (button pressed)
        const char* altMsg_;
        const char* altDecArg_;
        const char* altIncArg_;
        
        // Encoder state variables
        char lastEncoderState_;
        volatile int encoderDelta_;
        
        // Encoder debounce variables
        unsigned long encoderDebounceDelay_;
        unsigned long lastEncoderChangeTime_;
        char lastEncoderRawState_;
        
        // Button state variables
        bool buttonPressed_;
        unsigned long buttonDebounceDelay_;
        unsigned long lastButtonChangeTime_;
        bool lastButtonRawState_;

        char readEncoderState() {
            if (useExpander_ && expander_) {
                return (char)(expander_->readPin(expEncoderPinA_) << 1 | expander_->readPin(expEncoderPinB_));
            } else if (useMcpExpander_ && mcpExpander_) {
                return (char)(mcpExpander_->digitalRead(mcpExpEncoderPinA_) << 1 | mcpExpander_->digitalRead(mcpExpEncoderPinB_));
            } else if (usePcfExpander_ && pcfExpander_) {
                return (char)(pcfExpander_->digitalRead(pcfExpEncoderPinA_) << 1 | pcfExpander_->digitalRead(pcfExpEncoderPinB_));
            } else {
                return (char)(gpio_get(encoderPinA_) << 1 | gpio_get(encoderPinB_));
            }
        }
        
        bool readButtonState() {
            if (useExpander_ && expander_) {
                return !expander_->readPin(expButtonPin_); // Assuming active low with pull-up
            } else if (useMcpExpander_ && mcpExpander_) {
                return !mcpExpander_->digitalRead(mcpExpButtonPin_); // Assuming active low with pull-up
            } else if (usePcfExpander_ && pcfExpander_) {
                return !pcfExpander_->digitalRead(pcfExpButtonPin_); // Assuming active low with pull-up
            } else {
                return !gpio_get(buttonPin_); // Assuming active low with pull-up
            }
        }

    public:
        void pollInput() override {
            unsigned long now = to_ms_since_boot(get_absolute_time());
            
            // Handle button debouncing
            bool currentButtonRawState = readButtonState();
            if (currentButtonRawState != lastButtonRawState_) {
                lastButtonChangeTime_ = now;
                lastButtonRawState_ = currentButtonRawState;
            }
            
            if ((now - lastButtonChangeTime_) >= buttonDebounceDelay_) {
                if (currentButtonRawState != buttonPressed_) {
                    buttonPressed_ = currentButtonRawState;
                    // Reset encoder delta when button state changes to prevent unwanted commands
                    encoderDelta_ = 0;
                }
            }
            
            // Handle encoder debouncing
            char currentEncoderRawState = readEncoderState();
            if (currentEncoderRawState != lastEncoderRawState_) {
                lastEncoderChangeTime_ = now;
                lastEncoderRawState_ = currentEncoderRawState;
            }
            
            if ((now - lastEncoderChangeTime_) >= encoderDebounceDelay_) {
                if (currentEncoderRawState != lastEncoderState_) {
                    // Process encoder state machine
                    if (lastEncoderState_ == 0b00) {
                        if (currentEncoderRawState == 0b01) encoderDelta_++;
                        else if (currentEncoderRawState == 0b10) encoderDelta_--;
                    } else if (lastEncoderState_ == 0b01) {
                        if (currentEncoderRawState == 0b11) encoderDelta_++;
                        else if (currentEncoderRawState == 0b00) encoderDelta_--;
                    } else if (lastEncoderState_ == 0b11) {
                        if (currentEncoderRawState == 0b10) encoderDelta_++;
                        else if (currentEncoderRawState == 0b01) encoderDelta_--;
                    } else if (lastEncoderState_ == 0b10) {
                        if (currentEncoderRawState == 0b00) encoderDelta_++;
                        else if (currentEncoderRawState == 0b11) encoderDelta_--;
                    }
                    
                    lastEncoderState_ = currentEncoderRawState;
                }
            }
            
            // Send DCS-BIOS messages based on button state
            const char* msg;
            const char* incArg;
            const char* decArg;
            int stepsPerDetent;
            
            if (buttonPressed_) { // Button held down - use alternate function
                msg = altMsg_;
                incArg = altIncArg_;
                decArg = altDecArg_;
                stepsPerDetent = altStepsPerDetent;
            } else { // Normal operation
                msg = normalMsg_;
                incArg = normalIncArg_;
                decArg = normalDecArg_;
                stepsPerDetent = normalStepsPerDetent;
            }
            
            // Process encoder delta and send messages
            if (encoderDelta_ >= stepsPerDetent) {
                if (tryToSendDcsBiosMessage(msg, incArg)) {
                    encoderDelta_ -= stepsPerDetent;
                }
            }
            if (encoderDelta_ <= -stepsPerDetent) {
                if (tryToSendDcsBiosMessage(msg, decArg)) {
                    encoderDelta_ += stepsPerDetent;
                }
            }
        }

        // Constructor for native Pico pins
        EmulatedConcentricEncoderT(const char* normalMsg, const char* normalDecArg, const char* normalIncArg,
                                  const char* altMsg, const char* altDecArg, const char* altIncArg,
                                  char buttonPin, char encoderPinA, char encoderPinB,
                                  unsigned long encoderDebounceDelay = 5, unsigned long buttonDebounceDelay = 50)
            : PollingInput(pollIntervalMs), 
              normalMsg_(normalMsg), normalDecArg_(normalDecArg), normalIncArg_(normalIncArg),
              altMsg_(altMsg), altDecArg_(altDecArg), altIncArg_(altIncArg),
              buttonPin_(buttonPin), encoderPinA_(encoderPinA), encoderPinB_(encoderPinB),
              useExpander_(false), useMcpExpander_(false), usePcfExpander_(false), encoderDelta_(0),
              encoderDebounceDelay_(encoderDebounceDelay), lastEncoderChangeTime_(0),
              buttonPressed_(false), buttonDebounceDelay_(buttonDebounceDelay), lastButtonChangeTime_(0)
        {
            gpio_init(buttonPin_); gpio_pull_up(buttonPin_); gpio_set_dir(buttonPin_, GPIO_IN);
            gpio_init(encoderPinA_); gpio_pull_up(encoderPinA_); gpio_set_dir(encoderPinA_, GPIO_IN);
            gpio_init(encoderPinB_); gpio_pull_up(encoderPinB_); gpio_set_dir(encoderPinB_, GPIO_IN);
            
            lastEncoderRawState_ = readEncoderState();
            lastEncoderState_ = lastEncoderRawState_;
            lastButtonRawState_ = readButtonState();
            buttonPressed_ = lastButtonRawState_;
        }

        // Constructor for AW9523B expander
        EmulatedConcentricEncoderT(const char* normalMsg, const char* normalDecArg, const char* normalIncArg,
                                  const char* altMsg, const char* altDecArg, const char* altIncArg,
                                  AW9523B* expander, uint8_t buttonPin, uint8_t encoderPinA, uint8_t encoderPinB,
                                  unsigned long encoderDebounceDelay = 5, unsigned long buttonDebounceDelay = 50)
            : PollingInput(pollIntervalMs),
              normalMsg_(normalMsg), normalDecArg_(normalDecArg), normalIncArg_(normalIncArg),
              altMsg_(altMsg), altDecArg_(altDecArg), altIncArg_(altIncArg),
              expander_(expander), expButtonPin_(buttonPin), expEncoderPinA_(encoderPinA), expEncoderPinB_(encoderPinB),
              useExpander_(true), useMcpExpander_(false), usePcfExpander_(false), encoderDelta_(0),
              encoderDebounceDelay_(encoderDebounceDelay), lastEncoderChangeTime_(0),
              buttonPressed_(false), buttonDebounceDelay_(buttonDebounceDelay), lastButtonChangeTime_(0)
        {
            lastEncoderRawState_ = readEncoderState();
            lastEncoderState_ = lastEncoderRawState_;
            lastButtonRawState_ = readButtonState();
            buttonPressed_ = lastButtonRawState_;
        }

        // Constructor for MCP23S17 expander
        EmulatedConcentricEncoderT(const char* normalMsg, const char* normalDecArg, const char* normalIncArg,
                                  const char* altMsg, const char* altDecArg, const char* altIncArg,
                                  MCP23S17* expander, uint8_t buttonPin, uint8_t encoderPinA, uint8_t encoderPinB,
                                  unsigned long encoderDebounceDelay = 5, unsigned long buttonDebounceDelay = 50)
            : PollingInput(pollIntervalMs),
              normalMsg_(normalMsg), normalDecArg_(normalDecArg), normalIncArg_(normalIncArg),
              altMsg_(altMsg), altDecArg_(altDecArg), altIncArg_(altIncArg),
              mcpExpander_(expander), mcpExpButtonPin_(buttonPin), mcpExpEncoderPinA_(encoderPinA), mcpExpEncoderPinB_(encoderPinB),
              useExpander_(false), useMcpExpander_(true), usePcfExpander_(false), encoderDelta_(0),
              encoderDebounceDelay_(encoderDebounceDelay), lastEncoderChangeTime_(0),
              buttonPressed_(false), buttonDebounceDelay_(buttonDebounceDelay), lastButtonChangeTime_(0)
        {
            lastEncoderRawState_ = readEncoderState();
            lastEncoderState_ = lastEncoderRawState_;
            lastButtonRawState_ = readButtonState();
            buttonPressed_ = lastButtonRawState_;
        }

        // Constructor for PCF8575 expander
        EmulatedConcentricEncoderT(const char* normalMsg, const char* normalDecArg, const char* normalIncArg,
                                  const char* altMsg, const char* altDecArg, const char* altIncArg,
                                  PCF8575* expander, uint8_t buttonPin, uint8_t encoderPinA, uint8_t encoderPinB,
                                  unsigned long encoderDebounceDelay = 5, unsigned long buttonDebounceDelay = 50)
            : PollingInput(pollIntervalMs),
              normalMsg_(normalMsg), normalDecArg_(normalDecArg), normalIncArg_(normalIncArg),
              altMsg_(altMsg), altDecArg_(altDecArg), altIncArg_(altIncArg),
              pcfExpander_(expander), pcfExpButtonPin_(buttonPin), pcfExpEncoderPinA_(encoderPinA), pcfExpEncoderPinB_(encoderPinB),
              useExpander_(false), useMcpExpander_(false), usePcfExpander_(true), encoderDelta_(0),
              encoderDebounceDelay_(encoderDebounceDelay), lastEncoderChangeTime_(0),
              buttonPressed_(false), buttonDebounceDelay_(buttonDebounceDelay), lastButtonChangeTime_(0)
        {
            lastEncoderRawState_ = readEncoderState();
            lastEncoderState_ = lastEncoderRawState_;
            lastButtonRawState_ = readButtonState();
            buttonPressed_ = lastButtonRawState_;
        }

        // *** Implement pure virtual functions from base classes ***
        void resetState() override { // From PollingInput
            resetThisState();
        }

        void resetThisState() override { // From ResettableInput
            encoderDelta_ = 0;
            lastEncoderRawState_ = readEncoderState();
            lastEncoderState_ = lastEncoderRawState_;
            lastButtonRawState_ = readButtonState();
            buttonPressed_ = lastButtonRawState_;
            unsigned long now = to_ms_since_boot(get_absolute_time());
            lastEncoderChangeTime_ = now;
            lastButtonChangeTime_ = now;
        }
        // *********************************************************

        // Convenience reset method
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

    // Aliases for PCF8575 expander use
    typedef RotaryEncoderT<POLL_EVERY_TIME, ONE_STEP_PER_DETENT> PCF8575RotaryEncoderOneStep;
    typedef RotaryEncoderT<POLL_EVERY_TIME, TWO_STEPS_PER_DETENT> PCF8575RotaryEncoderTwoSteps;
    typedef RotaryEncoderT<POLL_EVERY_TIME, FOUR_STEPS_PER_DETENT> PCF8575RotaryEncoderFourSteps;
    typedef RotaryEncoderT<POLL_EVERY_TIME, EIGHT_STEPS_PER_DETENT> PCF8575RotaryEncoderEightSteps;

    // Aliases for Emulated Concentric Encoder
    typedef EmulatedConcentricEncoderT<POLL_EVERY_TIME, TWO_STEPS_PER_DETENT, ONE_STEP_PER_DETENT> EmulatedConcentricEncoder;
    typedef EmulatedConcentricEncoderT<POLL_EVERY_TIME, ONE_STEP_PER_DETENT, ONE_STEP_PER_DETENT> EmulatedConcentricEncoderOneStep;
    typedef EmulatedConcentricEncoderT<POLL_EVERY_TIME, TWO_STEPS_PER_DETENT, TWO_STEPS_PER_DETENT> EmulatedConcentricEncoderTwoSteps;
    typedef EmulatedConcentricEncoderT<POLL_EVERY_TIME, FOUR_STEPS_PER_DETENT, ONE_STEP_PER_DETENT> EmulatedConcentricEncoderFourStepsNormal;

} // namespace DcsBios

#endif // __DCSBIOS_ENCODERS_H