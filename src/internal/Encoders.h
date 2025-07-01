// Modified header version with AW9523B support for RotaryEncoderT and RotaryAcceleratedEncoderT only
#ifndef __DCSBIOS_ENCODERS_H
#define __DCSBIOS_ENCODERS_H

#include "pico/stdlib.h"
#include "aw9523b.h"
#include "MCP23S17.h" // Include MCP23S17 header

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

		MCP23S17* mcpExpander_ = nullptr; // New: MCP23S17 expander
		uint8_t mcpPinA_;             // New: MCP23S17 pin A
		uint8_t mcpPinB_;             // New: MCP23S17 pin B
		bool useMcpExpander_ = false; // New: Flag for MCP23S17

		char lastState_;
		signed char delta_;
		uint32_t lastUpdate_ = 0;

		char readState() {
			if (useExpander_ && expander_) {
				return (expander_->readPin(expPinA_) << 1) | expander_->readPin(expPinB_);
			} else if (useMcpExpander_ && mcpExpander_) { // New: Read from MCP23S17
				return (mcpExpander_->digitalRead(mcpPinA_) << 1) | mcpExpander_->digitalRead(mcpPinB_);
			}
			return (gpio_get(pinA_) << 1) | gpio_get(pinB_);
		}

		void resetState() { lastState_ = (lastState_ == 0) ? -1 : 0; }

	public: // Moved pollInput to public
		void pollInput() {
			uint32_t now = to_ms_since_boot(get_absolute_time());
			if (now - lastUpdate_ < 12) return;  // Throttle to one event every 10ms
			lastUpdate_ = now;
		
			char state = readState();
			switch (lastState_) {
				case 0: if (state == 2) delta_--; if (state == 1) delta_++; break;
				case 1: if (state == 0) delta_--; if (state == 3) delta_++; break;
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
		RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, char pinA, char pinB)
			: PollingInput(pollIntervalMs), useExpander_(false), useMcpExpander_(false) { // Modified: Initialize new flag
			msg_ = msg;
			decArg_ = decArg;
			incArg_ = incArg;
			pinA_ = pinA;
			pinB_ = pinB;
			gpio_init(pinA_); gpio_pull_up(pinA_); gpio_set_dir(pinA_, GPIO_IN);
			gpio_init(pinB_); gpio_pull_up(pinB_); gpio_set_dir(pinB_, GPIO_IN);
			delta_ = 0;
			lastState_ = readState();
		}

		RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, AW9523B* expander, uint8_t pinA, uint8_t pinB)
			: PollingInput(pollIntervalMs), expander_(expander), expPinA_(pinA), expPinB_(pinB), useExpander_(true), useMcpExpander_(false) { // Modified: Initialize new flag
			msg_ = msg;
			decArg_ = decArg;
			incArg_ = incArg;
			delta_ = 0;
			lastState_ = readState();
		}

		// New: Constructor for MCP23S17
		RotaryEncoderT(const char* msg, const char* decArg, const char* incArg, MCP23S17* expander, uint8_t pinA, uint8_t pinB)
			: PollingInput(pollIntervalMs), mcpExpander_(expander), mcpPinA_(pinA), mcpPinB_(pinB), useExpander_(false), useMcpExpander_(true) {
			msg_ = msg;
			decArg_ = decArg;
			incArg_ = incArg;
			delta_ = 0;
			// MCP23S17 pins need to be configured as input with pull-up externally.
			// The begin() method of MCP23S17 sets all pins as inputs by default.
			// Pull-ups are also handled by MCP23S17::pinMode with INPUT_PULLUP.
			lastState_ = readState();
		}

		void SetControl(const char* msg) { msg_ = msg; }
		void resetThisState() { resetState(); }
	};
	typedef RotaryEncoderT<> RotaryEncoder;
	typedef RotaryEncoderT<> MCP23S17RotaryEncoder; // New typedef for clarity

	// You can apply the same pattern to RotaryAcceleratedEncoderT when needed.

} // namespace DcsBios

#endif