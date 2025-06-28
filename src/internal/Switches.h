#ifndef __DCSBIOS_SWITCHES_H
#define __DCSBIOS_SWITCHES_H

#include <math.h>
#include "pico/stdlib.h"
#include "aw9523b.h" // Include for AW9523B support
#include "MCP23S17.h" // Include for MCP23S17 support

namespace DcsBios {

	// Assuming POLL_EVERY_TIME is defined as 0 in DcsBios.h or FoxConfig.h
	template <unsigned long pollIntervalMs = 0> // Changed default to 0 for "POLL_EVERY_TIME"
	class Switch2PosT : PollingInput, public ResettableInput {
	private:
		const char* msg_;
		char pin_;
		bool useAwExpander_; // Flag for AW9523B
		AW9523B* awExpander_; // Pointer for AW9523B
		bool useMcpExpander_; // Flag for MCP23S17
		MCP23S17* mcpExpander_; // Pointer for MCP23S17
		uint8_t expanderPin_;
		char debounceSteadyState_;
		char lastState_;
		bool reverse_;
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime = 0;

		char readInput() {
			if (useAwExpander_ && awExpander_) return awExpander_->readPin(expanderPin_);
			if (useMcpExpander_ && mcpExpander_) return mcpExpander_->digitalRead(expanderPin_);
			return gpio_get(pin_);
		}

		// Overrides PollingInput::resetState()
		void resetState() override {
			lastState_ = (lastState_ == 0) ? -1 : 0;
			// Further reset logic if needed
		}

		// Overrides ResettableInput::resetThisState()
		void resetThisState() override {
			// This function will call the actual reset logic implemented in resetState()
			this->resetState();
		}

		void pollInput() {
			char state = readInput();
			if (reverse_) state = !state;
			unsigned long now = to_ms_since_boot(get_absolute_time());
			if (state != debounceSteadyState_) {
				lastDebounceTime = now;
				debounceSteadyState_ = state;
			}
			if ((now - lastDebounceTime) >= debounceDelay_) {
				if (debounceSteadyState_ != lastState_) {
					if (tryToSendDcsBiosMessage(msg_, state == 1 ? "0" : "1")) {
						lastState_ = debounceSteadyState_;
					}
				}
			}
		}

	public:
		// Constructor for direct Pico GPIO pins
		Switch2PosT(const char* msg, char pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), pin_(pin), useAwExpander_(false), awExpander_(nullptr),
			useMcpExpander_(false), mcpExpander_(nullptr), expanderPin_(0), reverse_(reverse), debounceDelay_(debounceDelay) {
			gpio_init(pin_);
			gpio_pull_up(pin_); // Assume active-low switch requiring pull-up
			gpio_set_dir(pin_, GPIO_IN);
			debounceSteadyState_ = readInput();
			lastState_ = debounceSteadyState_;
			lastDebounceTime = to_ms_since_boot(get_absolute_time());
		}

		// Constructor for AW9523B expander
		Switch2PosT(const char* msg, AW9523B* expander, uint8_t expanderPin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), pin_(0), useAwExpander_(true), awExpander_(expander),
			useMcpExpander_(false), mcpExpander_(nullptr), expanderPin_(expanderPin), reverse_(reverse), debounceDelay_(debounceDelay) {
			// Pins should be configured as INPUT_PULLUP by the expander's setup
            // No pinMode call here, as AW9523B doesn't have it as per previous error.
			debounceSteadyState_ = readInput();
			lastState_ = debounceSteadyState_;
			lastDebounceTime = to_ms_since_boot(get_absolute_time());
		}

		// Constructor for MCP23S17 expander
		Switch2PosT(const char* msg, MCP23S17* expander, uint8_t expanderPin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), pin_(0), useAwExpander_(false), awExpander_(nullptr),
			useMcpExpander_(true), mcpExpander_(expander), expanderPin_(expanderPin), reverse_(reverse), debounceDelay_(debounceDelay) {
			if (mcpExpander_) {
				mcpExpander_->pinMode(expanderPin_, INPUT_PULLUP);
			}
			debounceSteadyState_ = readInput();
			lastState_ = debounceSteadyState_;
			lastDebounceTime = to_ms_since_boot(get_absolute_time());
		}
	};


	template <unsigned long pollIntervalMs = 0, unsigned int numPositions = 3> // Changed default to 0 for "POLL_EVERY_TIME"
	class SwitchMultiPosT : PollingInput, public ResettableInput { // Added public ResettableInput explicitly for clarity, though it might be inherited via PollingInput
	private:
		const char* msg_;
		const uint8_t* pins_; // Array of Pico GPIO pins
		AW9523B* awExpander_; // Pointer for AW9523B
		MCP23S17* mcpExpander_; // Pointer for MCP23S17
		const uint8_t* expPins_; // Array of expander pins
		bool useAwExpander_; // Flag for AW9523B
		bool useMcpExpander_; // Flag for MCP23S17
		char lastState_; // Stores the index of the currently active pin (0 to numPositions-1)
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime_;

		char readInput(int index) {
			if (useAwExpander_ && awExpander_) return awExpander_->readPin(expPins_[index]);
			if (useMcpExpander_ && mcpExpander_) return mcpExpander_->digitalRead(expPins_[index]);
			return gpio_get(pins_[index]);
		}

		// Overrides PollingInput::resetState()
		void resetState() override {
			lastState_ = -1; // No position active initially
			for (int i = 0; i < numPositions; ++i) {
				if (readInput(i) == 0) { // Assuming active-low
					lastState_ = i;
					break;
				}
			}
			lastDebounceTime_ = to_ms_since_boot(get_absolute_time());
		}

		// Overrides ResettableInput::resetThisState()
		void resetThisState() override {
			// This function will call the actual reset logic implemented in resetState()
			this->resetState();
		}

		void pollInput() {
			unsigned long now = to_ms_since_boot(get_absolute_time());
			for (int i = 0; i < numPositions; ++i) {
				char state = readInput(i);
				if (state == 0) {  // active-low
					if (i != lastState_ && (now - lastDebounceTime_) >= debounceDelay_) {
						char msgBuffer[3];
						snprintf(msgBuffer, sizeof(msgBuffer), "%d", i);
						if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
							lastState_ = i;
							lastDebounceTime_ = now;
						}
					}
					break; // Found the active switch, exit loop
				}
			}
		}
	
	public:
		// Constructor for direct Pico GPIO pins
		SwitchMultiPosT(const char* msg, const uint8_t* pins, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), pins_(pins), awExpander_(nullptr), mcpExpander_(nullptr), expPins_(nullptr),
			useAwExpander_(false), useMcpExpander_(false), debounceDelay_(debounceDelay) {
			for (int i = 0; i < numPositions; ++i) {
				gpio_init(pins_[i]);
				gpio_pull_up(pins_[i]); // Assume active-low requiring pull-up
				gpio_set_dir(pins_[i], GPIO_IN);
			}
			resetState(); // Call the actual reset function
		}
	
		// Constructor for AW9523B expander
		SwitchMultiPosT(const char* msg, AW9523B* expander, const uint8_t* expPins, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), pins_(nullptr), awExpander_(expander), mcpExpander_(nullptr), expPins_(expPins),
			useAwExpander_(true), useMcpExpander_(false), debounceDelay_(debounceDelay) {
			// Pins should be configured as INPUT_PULLUP by the expander's setup
            // No pinMode call here for AW9523B.
			resetState(); // Call the actual reset function
		}

		// Constructor for MCP23S17 expander
		SwitchMultiPosT(const char* msg, MCP23S17* expander, const uint8_t* expPins, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), pins_(nullptr), awExpander_(nullptr), mcpExpander_(expander), expPins_(expPins),
			useAwExpander_(false), useMcpExpander_(true), debounceDelay_(debounceDelay) {
			if (mcpExpander_) {
				for (int i = 0; i < numPositions; ++i) {
					mcpExpander_->pinMode(expPins_[i], INPUT_PULLUP);
				}
			}
			resetState(); // Call the actual reset function
		}
	};

} // namespace DcsBios

#endif // __DCSBIOS_SWITCHES_H