// Modified header version with AW9523B support for RotaryEncoderT and RotaryAcceleratedEncoderT only
#ifndef __DCSBIOS_ENCODERS_H
#define __DCSBIOS_ENCODERS_H

#include "pico/stdlib.h"
#include "aw9523b.h"

namespace DcsBios {

	enum StepsPerDetent {
		ONE_STEP_PER_DETENT = 1,
		TWO_STEPS_PER_DETENT = 2,
		FOUR_STEPS_PER_DETENT = 4,
		EIGHT_STEPS_PER_DETENT = 8,
	};

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, StepsPerDetent stepsPerDetent = FOUR_STEPS_PER_DETENT>
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
		char lastState_;
		signed char delta_;
		uint32_t lastUpdate_ = 0;

		char readState() {
			if (useExpander_ && expander_) {
				return (expander_->readPin(expPinA_) << 1) | expander_->readPin(expPinB_);
			}
			return (gpio_get(pinA_) << 1) | gpio_get(pinB_);
		}

		void resetState() { lastState_ = (lastState_ == 0) ? -1 : 0; }

		void pollInput() {
			uint32_t now = to_ms_since_boot(get_absolute_time());
			if (now - lastUpdate_ < 20) return;  // Throttle to one event every 20ms
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
			: PollingInput(pollIntervalMs), useExpander_(false) {
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
			: PollingInput(pollIntervalMs), expander_(expander), expPinA_(pinA), expPinB_(pinB), useExpander_(true) {
			msg_ = msg;
			decArg_ = decArg;
			incArg_ = incArg;
			delta_ = 0;
			lastState_ = readState();
		}

		void SetControl(const char* msg) { msg_ = msg; }
		void resetThisState() { resetState(); }
	};
	typedef RotaryEncoderT<> RotaryEncoder;
	typedef RotaryEncoderT<DcsBios::ONE_STEP_PER_DETENT> RotaryEncoder1Step;
	typedef RotaryEncoderT<DcsBios::TWO_STEPS_PER_DETENT> RotaryEncoder2Step;
	typedef RotaryEncoderT<DcsBios::FOUR_STEPS_PER_DETENT> RotaryEncoder4Step;
	typedef RotaryEncoderT<DcsBios::EIGHT_STEPS_PER_DETENT> RotaryEncoder8Step;

	// You can apply the same pattern to RotaryAcceleratedEncoderT when needed.

	//To emulate dual concentric rotary encoders/resolvers/potentiometers using a rotary encoder with a push button.
	//Secondary message is used when push button or switch is enabled.
	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, StepsPerDetent stepsPerDetent = FOUR_STEPS_PER_DETENT>
	class EmulatedConcentricRotaryEncoderT : PollingInput, public ResettableInput {
	private:
		const char* msg1_;	//Function 1 (default)
		const char* decArg1_;
		const char* incArg1_;
		const char* msg2_;	//Function 2
		const char* decArg2_;
		const char* incArg2_;
		char pinA_;
		char pinB_;
		char pinToggle_;	//Integrated button pin
		AW9523B* expander_ = nullptr;
		uint8_t expPinA_;
		uint8_t expPinB_;
		uint8_t expPinToggle_;
		bool useExpander_ = false;
		bool msg1Mode_;
		char prevMode_;
		char lastState_;
		signed char delta_;
		uint32_t lastUpdate_ = 0;
		
		char readState() {
			if (useExpander_ && expander_) {
				char currentMode;
				msg1Mode_ = ((currentMode = expander_->readPin(expPinToggle_)) != prevMode_)?!msg1Mode_:msg1Mode_;
				prevMode_ = currentMode;
				return (expander_->readPin(expPinA_) << 1) | expander_->readPin(expPinB_);
			} else {
				char currentMode;
				msg1Mode_ = ((currentMode = gpio_get(pinToggle_)) != prevMode_)?!msg1Mode_:msg1Mode_;
				prevMode_ = currentMode;
				return (gpio_get(pinA_) << 1) | gpio_get(pinB_);
			}
		}
		
		void resetState() {
			msg1Mode_ = !msg1Mode_;
			lastState_ = (lastState_==0)?-1:0;
		}
		
		void pollInput() {
			uint32_t now = to_ms_since_boot(get_absolute_time());
			if (now - lastUpdate_ < 20) return;  // Throttle to one event every 20ms
			lastUpdate_ = now;
			
			char state = readState();
			switch(lastState_) {
				case 0:
					if (state == 2) delta_--;
					if (state == 1) delta_++;
					break;
				case 1:
					if (state == 0) delta_--;
					if (state == 3) delta_++;
					break;
				case 2:
					if (state == 3) delta_--;
					if (state == 0) delta_++;
					break;
				case 3:
					if (state == 1) delta_--;
					if (state == 2) delta_++;
					break;
			}
			lastState_ = state;
			
			if (delta_ >= stepsPerDetent) {
				if (tryToSendDcsBiosMessage(msg1Mode_?msg1_:msg2_, msg1Mode_?incArg1_:incArg2_))
					delta_ -= stepsPerDetent;
			}
			if (delta_ <= -stepsPerDetent) {
				if (tryToSendDcsBiosMessage(msg1Mode_?msg1_:msg2_, msg1Mode_?decArg1_:decArg2_))
					delta_ += stepsPerDetent;
			}
		}
	public:
		// Constructor for direct GPIO pins
		EmulatedConcentricRotaryEncoderT(const char* msg1, const char* decArg1, const char* incArg1, 
			const char* msg2, const char* decArg2, const char* incArg2, 
			char pinA, char pinB, char pinC) :
			PollingInput(pollIntervalMs), useExpander_(false)
		{
			msg1_ = msg1;
			decArg1_ = decArg1;
			incArg1_ = incArg1;
			msg2_ = msg2;
			decArg2_ = decArg2;
			incArg2_ = incArg2;
			
			pinA_ = pinA;
			pinB_ = pinB;
			pinToggle_ = pinC;
			msg1Mode_ = true;
			
			gpio_init(pinA_); gpio_pull_up(pinA_); gpio_set_dir(pinA_, GPIO_IN);
			gpio_init(pinB_); gpio_pull_up(pinB_); gpio_set_dir(pinB_, GPIO_IN);
			gpio_init(pinToggle_); gpio_pull_up(pinToggle_); gpio_set_dir(pinToggle_, GPIO_IN);
			prevMode_ = gpio_get(pinToggle_);	//Prevents defaulting to secondary action on initialization
			
			delta_ = 0;
			lastState_ = readState();
		}

		// Constructor for AW9523B expander pins
		EmulatedConcentricRotaryEncoderT(const char* msg1, const char* decArg1, const char* incArg1, 
			const char* msg2, const char* decArg2, const char* incArg2,
			AW9523B* expander, uint8_t pinA, uint8_t pinB, uint8_t pinC) :
			PollingInput(pollIntervalMs), expander_(expander), expPinA_(pinA), expPinB_(pinB), 
			expPinToggle_(pinC), useExpander_(true)
		{
			msg1_ = msg1;
			decArg1_ = decArg1;
			incArg1_ = incArg1;
			msg2_ = msg2;
			decArg2_ = decArg2;
			incArg2_ = incArg2;
			
			msg1Mode_ = true;
			
			delta_ = 0;
			lastState_ = readState();
		}

		void SetControl(const char* msg) {	
			msg1_ = msg;	// Note this won't work to remap the second message
		}
		
		void resetThisState() {
			this->resetState();
		}
	};
	typedef EmulatedConcentricRotaryEncoderT<> EmulatedConcentricRotaryEncoder;
	typedef EmulatedConcentricRotaryEncoderT<DcsBios::ONE_STEP_PER_DETENT> EmulatedConcentricRotaryEncoder1Step;
	typedef EmulatedConcentricRotaryEncoderT<DcsBios::TWO_STEPS_PER_DETENT> EmulatedConcentricRotaryEncoder2Step;
	typedef EmulatedConcentricRotaryEncoderT<DcsBios::FOUR_STEPS_PER_DETENT> EmulatedConcentricRotaryEncoder4Step;
	typedef EmulatedConcentricRotaryEncoderT<DcsBios::EIGHT_STEPS_PER_DETENT> EmulatedConcentricRotaryEncoder8Step;

} // namespace DcsBios

#endif
