// Modified header version with AW9523B support for RotaryEncoderT and RotaryAcceleratedEncoderT only
#ifndef __DCSBIOS_ENCODERS_H
#define __DCSBIOS_ENCODERS_H

#include "pico/stdlib.h"
#include "aw9523b.h" // Assuming aw9523b.h provides the AW9523B class definition
#include "MCP23S17.h" // Include for MCP23S17 support

namespace DcsBios {

	enum StepsPerDetent {
		ONE_STEP_PER_DETENT = 1,
		TWO_STEPS_PER_DETENT = 2,
		FOUR_STEPS_PER_DETENT = 4,
		EIGHT_STEPS_PER_DETENT = 8,
	};

	// Assuming POLL_EVERY_TIME is defined as 0 in DcsBios.h or FoxConfig.h
	template <unsigned long pollIntervalMs = 0, StepsPerDetent stepsPerDetent = ONE_STEP_PER_DETENT> // Changed default to 0
	class RotaryEncoderT : PollingInput, public ResettableInput {
	private:
		const char* msg_;
		const char* decArg_;
		const char* incArg_;
		char pinA_;
		char pinB_;
		AW9523B* awExpander_ = nullptr; // Renamed to differentiate from MCP23S17
		MCP23S17* mcpExpander_ = nullptr; // New member for MCP23S17
		uint8_t expPinA_;
		uint8_t expPinB_;
		bool useAwExpander_ = false; // Flag for AW9523B
		bool useMcpExpander_ = false; // Flag for MCP23S17
		char lastState_;
		signed char delta_;
		uint32_t lastUpdate_ = 0;

		char readState() {
			if (useAwExpander_ && awExpander_) {
				return (awExpander_->readPin(expPinA_) << 1) | awExpander_->readPin(expPinB_);
			} else if (useMcpExpander_ && mcpExpander_) { // Read from MCP23S17 if configured
                return (mcpExpander_->digitalRead(expPinA_) << 1) | mcpExpander_->digitalRead(expPinB_);
            }
			return (gpio_get(pinA_) << 1) | gpio_get(pinB_);
		}

		// Overrides PollingInput::resetState()
		void resetState() override {
			lastState_ = readState();
			delta_ = 0;
		}

		// Overrides ResettableInput::resetThisState()
		void resetThisState() override {
			// This function will call the actual reset logic implemented in resetState()
			this->resetState();
		}

		void pollInput() {
			unsigned long now = to_ms_since_boot(get_absolute_time());
			if (now - lastUpdate_ < pollIntervalMs) {
				return; // Not time to poll yet
			}
			lastUpdate_ = now;

			char state = readState();
			if (state == lastState_) return;

			// Encoder logic (Gray code)
			switch (lastState_) {
				case 0: if (state == 1) delta_--; if (state == 2) delta_++; break;
				case 1: if (state == 3) delta_--; if (state == 0) delta_++; break;
				case 2: if (state == 3) delta_--; if (state == 0) delta_++; break;
				case 3: if (state == 1) delta_--; if (state == 2) delta_++; break;
			}
			lastState_ = state;
		
			if (delta_ >= stepsPerDetent) {
				if (tryToSendDcsBiosMessage(msg_, incArg_)) delta_ -= stepsPerDetent;
			}
			if (delta_ <= -stepsPerDetent) {
				if (tryToSendDcsBiosMessage(msg_, decArg_)) delta_ += stepsPerDetent;
			}
		}
		

	public:
		// Constructor for direct Pico GPIO pins
		RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, char pinA, char pinB)
			: PollingInput(pollIntervalMs), msg_(msg), decArg_(decArg), incArg_(incArg),
			  pinA_(pinA), pinB_(pinB), useAwExpander_(false), useMcpExpander_(false) {
			gpio_init(pinA_); gpio_pull_up(pinA_); gpio_set_dir(pinA_, GPIO_IN);
			gpio_init(pinB_); gpio_pull_up(pinB_); gpio_set_dir(pinB_, GPIO_IN);
			delta_ = 0;
			lastState_ = readState();
		}

		// Constructor for AW9523B expander
		RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, AW9523B* expander, uint8_t expPinA, uint8_t expPinB)
			: PollingInput(pollIntervalMs), msg_(msg), decArg_(decArg), incArg_(incArg),
			  awExpander_(expander), expPinA_(expPinA), expPinB_(expPinB), useAwExpander_(true), useMcpExpander_(false) {
			// AW9523B pins are configured by the expander class itself or manually.
            // Removed awExpander_->pinMode() calls as AW9523B class doesn't have this member.
			delta_ = 0;
			lastState_ = readState();
		}

        // Constructor for MCP23S17 expander
		RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, MCP23S17* expander, uint8_t expPinA, uint8_t expPinB)
			: PollingInput(pollIntervalMs), msg_(msg), decArg_(decArg), incArg_(incArg),
			  mcpExpander_(expander), expPinA_(expPinA), expPinB_(expPinB), useAwExpander_(false), useMcpExpander_(true) {
			if (mcpExpander_) {
				mcpExpander_->pinMode(expPinA_, INPUT_PULLUP);
				mcpExpander_->pinMode(expPinB_, INPUT_PULLUP);
			}
			delta_ = 0;
			lastState_ = readState();
		}
	};


	template <unsigned long pollIntervalMs = 0, StepsPerDetent stepsPerDetent = ONE_STEP_PER_DETENT> // Changed default to 0
	class RotaryAcceleratedEncoderT : public RotaryEncoderT<pollIntervalMs, stepsPerDetent> {
	private:
		unsigned long lastActivityTime_ = 0;
		unsigned long accelerationThresholdMs_ = 150; // Time in ms below which acceleration kicks in
		unsigned long accelerationFactor_ = 2; // How much to multiply delta by

	protected:
		// RotaryAcceleratedEncoderT inherits from RotaryEncoderT, which now implements resetState()
		// and resetThisState(). No need to implement them here unless different behavior is needed.

		void pollInput() override {
			unsigned long now = to_ms_since_boot(get_absolute_time());
			if (now - this->lastUpdate_ < pollIntervalMs) {
				return; // Not time to poll yet
			}
			this->lastUpdate_ = now;

			char state = this->readState();
			if (state == this->lastState_) return;

			// Store activity time for acceleration logic
			lastActivityTime_ = now;

			// Encoder logic (Gray code)
			switch (this->lastState_) {
				case 0: if (state == 1) this->delta_--; if (state == 2) this->delta_++; break;
				case 1: if (state == 3) this->delta_--; if (state == 0) this->delta_++; break;
				case 2: if (state == 3) this->delta_--; if (state == 0) this->delta_++; break;
				case 3: if (state == 1) this->delta_--; if (state == 2) this->delta_++; break;
			}
			this->lastState_ = state;

			// Apply acceleration
			signed char effectiveDelta = this->delta_;
			if (now - lastActivityTime_ < accelerationThresholdMs_) {
				effectiveDelta *= accelerationFactor_;
			}
		
			if (effectiveDelta >= stepsPerDetent) {
				if (this->tryToSendDcsBiosMessage(this->msg_, this->incArg_)) this->delta_ -= stepsPerDetent;
			}
			if (effectiveDelta <= -stepsPerDetent) {
				if (this->tryToSendDcsBiosMessage(this->msg_, this->decArg_)) this->delta_ += stepsPerDetent;
			}
		}

	public:
		// Constructors for direct Pico GPIO pins
		RotaryAcceleratedEncoderT(const char* msg, const char* decArg, const char* incArg, char pinA, char pinB,
								  unsigned long accelerationThresholdMs = 150, unsigned long accelerationFactor = 2)
			: RotaryEncoderT<pollIntervalMs, stepsPerDetent>(msg, decArg, incArg, pinA, pinB),
			  accelerationThresholdMs_(accelerationThresholdMs), accelerationFactor_(accelerationFactor) {}

		// Constructors for AW9523B expander
		RotaryAcceleratedEncoderT(const char* msg, const char* decArg, const char* incArg, AW9523B* expander, uint8_t expPinA, uint8_t expPinB,
								  unsigned long accelerationThresholdMs = 150, unsigned long accelerationFactor = 2)
			: RotaryEncoderT<pollIntervalMs, stepsPerDetent>(msg, decArg, incArg, expander, expPinA, expPinB),
			  accelerationThresholdMs_(accelerationThresholdMs), accelerationFactor_(accelerationFactor) {}

        // Constructors for MCP23S17 expander
		RotaryAcceleratedEncoderT(const char* msg, const char* decArg, const char* incArg, MCP23S17* expander, uint8_t expPinA, uint8_t expPinB,
								  unsigned long accelerationThresholdMs = 150, unsigned long accelerationFactor = 2)
			: RotaryEncoderT<pollIntervalMs, stepsPerDetent>(msg, decArg, incArg, expander, expPinA, expPinB),
			  accelerationThresholdMs_(accelerationThresholdMs), accelerationFactor_(accelerationFactor) {}
	};

} // namespace DcsBios

#endif // __DCSBIOS_ENCODERS_H