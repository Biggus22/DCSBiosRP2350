#ifndef __DCSBIOS_SWITCHES_H
#define __DCSBIOS_SWITCHES_H

#include <math.h>
#include <cstdio>
#include "pico/stdlib.h"
#include "aw9523b.h"
#include "PCF8575.h"
#include "MCP23S17.h"

namespace DcsBios {

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
	class Switch2PosT : PollingInput, public ResettableInput {
	private:
		const char* msg_;
		char pin_;
		bool useExpander_;
		enum ExpanderType { NONE, AW9523B_TYPE, PCF8575_TYPE, MCP23S17_TYPE };
		ExpanderType expanderType_;
		
		// Expander pointers
		AW9523B* aw9523b_;
		PCF8575* pcf8575_;
		MCP23S17* mcp23s17_;
		
		uint8_t expanderPin_;
		char debounceSteadyState_;
		char lastState_;
		bool reverse_;
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime = 0;

		char readInput() {
			if (useExpander_) {
				switch (expanderType_) {
					case AW9523B_TYPE:
						if (aw9523b_) return aw9523b_->readPin(expanderPin_);
						break;
					case PCF8575_TYPE:
						if (pcf8575_) return pcf8575_->digitalRead(expanderPin_);
						break;
					case MCP23S17_TYPE:
						if (mcp23s17_) return mcp23s17_->digitalRead(expanderPin_);
						break;
					default:
						break;
				}
				return 0; // Default if expander not found
			}
			return gpio_get(pin_);
		}

		void resetState() {
			lastState_ = (lastState_ == 0) ? -1 : 0;
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
		// GPIO constructor
		Switch2PosT(const char* msg, char pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(false), expanderType_(NONE)
		{
			msg_ = msg;
			pin_ = pin;
			gpio_init(pin_);
			gpio_pull_up(pin_);
			gpio_set_dir(pin_, GPIO_IN);
			debounceDelay_ = debounceDelay;
			reverse_ = reverse;
			lastState_ = gpio_get(pin_);
			if (reverse_) lastState_ = !lastState_;
		}

		// AW9523B constructor
		Switch2PosT(const char* msg, AW9523B* expander, uint8_t pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(true), expanderType_(AW9523B_TYPE), 
			aw9523b_(expander), expanderPin_(pin)
		{
			msg_ = msg;
			debounceDelay_ = debounceDelay;
			reverse_ = reverse;
			aw9523b_->setPinInput(expanderPin_);
			lastState_ = readInput();
			if (reverse_) lastState_ = !lastState_;
		}

		// PCF8575 constructor
		Switch2PosT(const char* msg, PCF8575* expander, uint8_t pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(true), expanderType_(PCF8575_TYPE), 
			pcf8575_(expander), expanderPin_(pin)
		{
			msg_ = msg;
			debounceDelay_ = debounceDelay;
			reverse_ = reverse;
			pcf8575_->pinMode(expanderPin_, GPIO_IN);
			pcf8575_->pullUp(expanderPin_, true); // Enable pull-up
			lastState_ = readInput();
			if (reverse_) lastState_ = !lastState_;
		}

		// MCP23S17 constructor
		Switch2PosT(const char* msg, MCP23S17* expander, uint8_t pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(true), expanderType_(MCP23S17_TYPE), 
			mcp23s17_(expander), expanderPin_(pin)
		{
			msg_ = msg;
			debounceDelay_ = debounceDelay;
			reverse_ = reverse;
			mcp23s17_->pinMode(expanderPin_, _INPUT_PULLUP);
			lastState_ = readInput();
			if (reverse_) lastState_ = !lastState_;
		}

		void SetControl(const char* msg) { msg_ = msg; }
		void resetThisState() { this->resetState(); }

		bool getState() const {
			return lastState_ != 0;
		}
	};
	typedef Switch2PosT<> Switch2Pos;

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, unsigned long coverDelayMs = 200>
	class SwitchWithCover2PosT : PollingInput, public ResettableInput {
	private:
		const char* switchMsg_;
		const char* coverMsg_;
		char pin_;
		bool useExpander_;
		enum ExpanderType { NONE, AW9523B_TYPE, PCF8575_TYPE, MCP23S17_TYPE };
		ExpanderType expanderType_;
		
		// Expander pointers
		AW9523B* aw9523b_;
		PCF8575* pcf8575_;
		MCP23S17* mcp23s17_;
		
		uint8_t expanderPin_;
		char lastState_;
		char switchState_;
		bool reverse_;
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime = 0;

		enum switchCoverStateEnum { stOFF_CLOSED = 0, stOFF_OPEN = 1, stON_OPEN = 2 };
		switchCoverStateEnum switchCoverState_;
		unsigned long lastSwitchStateTime;

		char readInput() {
			if (useExpander_) {
				switch (expanderType_) {
					case AW9523B_TYPE:
						if (aw9523b_) return aw9523b_->readPin(expanderPin_);
						break;
					case PCF8575_TYPE:
						if (pcf8575_) return pcf8575_->digitalRead(expanderPin_);
						break;
					case MCP23S17_TYPE:
						if (mcp23s17_) return mcp23s17_->digitalRead(expanderPin_);
						break;
					default:
						break;
				}
				return 0;
			}
			return gpio_get(pin_);
		}

		void resetState() {
			lastState_ = (lastState_ == 0) ? -1 : 0;
			if (switchState_ && !reverse_) switchCoverState_ = stOFF_CLOSED;
			else switchCoverState_ = stON_OPEN;
			lastSwitchStateTime = to_ms_since_boot(get_absolute_time());
		}

		void pollInput() {
			char state = readInput();
			if (reverse_) state = !state;
			if (state != lastState_) lastDebounceTime = to_ms_since_boot(get_absolute_time());
			if (state != switchState_ &&
				(to_ms_since_boot(get_absolute_time()) - lastDebounceTime) > debounceDelay_) {
				if (to_ms_since_boot(get_absolute_time()) - lastSwitchStateTime > coverDelayMs) {
					if (state) {
						if (switchCoverState_ == stON_OPEN && tryToSendDcsBiosMessage(switchMsg_, "0")) {
							switchCoverState_ = stOFF_OPEN;
							lastSwitchStateTime = to_ms_since_boot(get_absolute_time());
						} else if (switchCoverState_ == stOFF_OPEN && tryToSendDcsBiosMessage(coverMsg_, "0")) {
							switchCoverState_ = stOFF_CLOSED;
							lastSwitchStateTime = to_ms_since_boot(get_absolute_time());
							switchState_ = state;
						}
					} else {
						if (switchCoverState_ == stOFF_CLOSED && tryToSendDcsBiosMessage(coverMsg_, "1")) {
							switchCoverState_ = stOFF_OPEN;
							lastSwitchStateTime = to_ms_since_boot(get_absolute_time());
						} else if (switchCoverState_ == stOFF_OPEN && tryToSendDcsBiosMessage(switchMsg_, "1")) {
							switchCoverState_ = stON_OPEN;
							lastSwitchStateTime = to_ms_since_boot(get_absolute_time());
							switchState_ = state;
						}
					}
				}
			}
			lastState_ = state;
		}

	public:
		// GPIO constructor
		SwitchWithCover2PosT(const char* switchMsg, const char* coverMsg, char pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(false), expanderType_(NONE)
		{
			switchMsg_ = switchMsg;
			coverMsg_ = coverMsg;
			pin_ = pin;
			gpio_init(pin_);
			gpio_pull_up(pin_);
			gpio_set_dir(pin_, GPIO_IN);
			switchState_ = gpio_get(pin_);
			lastState_ = switchState_;
			reverse_ = reverse;
			debounceDelay_ = debounceDelay;
			resetState();
		}

		// AW9523B constructor
		SwitchWithCover2PosT(const char* switchMsg, const char* coverMsg, AW9523B* expander, uint8_t pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(true), expanderType_(AW9523B_TYPE), aw9523b_(expander), expanderPin_(pin)
		{
			switchMsg_ = switchMsg;
			coverMsg_ = coverMsg;
			reverse_ = reverse;
			aw9523b_->setPinInput(expanderPin_);
			debounceDelay_ = debounceDelay;
			switchState_ = readInput();
			lastState_ = switchState_;
			resetState();
		}

		// PCF8575 constructor
		SwitchWithCover2PosT(const char* switchMsg, const char* coverMsg, PCF8575* expander, uint8_t pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(true), expanderType_(PCF8575_TYPE), pcf8575_(expander), expanderPin_(pin)
		{
			switchMsg_ = switchMsg;
			coverMsg_ = coverMsg;
			reverse_ = reverse;
			pcf8575_->pinMode(expanderPin_, GPIO_IN);
			pcf8575_->pullUp(expanderPin_, true);
			debounceDelay_ = debounceDelay;
			switchState_ = readInput();
			lastState_ = switchState_;
			resetState();
		}

		// MCP23S17 constructor
		SwitchWithCover2PosT(const char* switchMsg, const char* coverMsg, MCP23S17* expander, uint8_t pin, bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), useExpander_(true), expanderType_(MCP23S17_TYPE), mcp23s17_(expander), expanderPin_(pin)
		{
			switchMsg_ = switchMsg;
			coverMsg_ = coverMsg;
			reverse_ = reverse;
			mcp23s17_->pinMode(expanderPin_, _INPUT_PULLUP);
			debounceDelay_ = debounceDelay;
			switchState_ = readInput();
			lastState_ = switchState_;
			resetState();
		}

		void resetThisState() { this->resetState(); }
	};
	typedef SwitchWithCover2PosT<> SwitchWithCover2Pos;

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, int numPositions = 3>
	class SwitchMultiPosT : PollingInput, public ResettableInput {
	private:
		const char* msg_;
		const uint8_t* pins_;
		bool useExpander_;
		enum ExpanderType { NONE, AW9523B_TYPE, PCF8575_TYPE, MCP23S17_TYPE };
		ExpanderType expanderType_;
		
		// Expander pointers
		AW9523B* aw9523b_;
		PCF8575* pcf8575_;
		MCP23S17* mcp23s17_;
		
		uint8_t expanderPins_[5];
		char lastState_;
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime_ = 0;
	
		char readInput(uint8_t index) {
			if (useExpander_) {
				switch (expanderType_) {
					case AW9523B_TYPE:
						if (aw9523b_) return aw9523b_->readPin(expanderPins_[index]);
						break;
					case PCF8575_TYPE:
						if (pcf8575_) return pcf8575_->digitalRead(expanderPins_[index]);
						break;
					case MCP23S17_TYPE:
						if (mcp23s17_) return mcp23s17_->digitalRead(expanderPins_[index]);
						break;
					default:
						break;
				}
				return 0;
			}
			return gpio_get(pins_[index]);
		}
	
		void resetState() {
			lastState_ = -1;
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
					break;
				}
			}
		}
	
	public:
		// GPIO constructor
		SwitchMultiPosT(const char* msg, const uint8_t* pins, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), pins_(pins), useExpander_(false), 
			expanderType_(NONE), debounceDelay_(debounceDelay) {
			for (int i = 0; i < numPositions; ++i) {
				gpio_init(pins[i]);
				gpio_pull_up(pins[i]);
				gpio_set_dir(pins[i], GPIO_IN);
			}
			resetState();
		}
	
		// AW9523B constructor
		SwitchMultiPosT(const char* msg, AW9523B* expander, const uint8_t* pins, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), useExpander_(true), expanderType_(AW9523B_TYPE), 
			aw9523b_(expander), debounceDelay_(debounceDelay) {
			for (int i = 0; i < numPositions; ++i) {
				expanderPins_[i] = pins[i];
				aw9523b_->setPinInput(pins[i]);
			}
			resetState();
		}

		// PCF8575 constructor
		SwitchMultiPosT(const char* msg, PCF8575* expander, const uint8_t* pins, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), useExpander_(true), expanderType_(PCF8575_TYPE), 
			pcf8575_(expander), debounceDelay_(debounceDelay) {
			for (int i = 0; i < numPositions; ++i) {
				expanderPins_[i] = pins[i];
				pcf8575_->pinMode(pins[i], GPIO_IN);
				pcf8575_->pullUp(pins[i], true);
			}
			resetState();
		}

		// MCP23S17 constructor
		SwitchMultiPosT(const char* msg, MCP23S17* expander, const uint8_t* pins, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), msg_(msg), useExpander_(true), expanderType_(MCP23S17_TYPE), 
			mcp23s17_(expander), debounceDelay_(debounceDelay) {
			for (int i = 0; i < numPositions; ++i) {
				expanderPins_[i] = pins[i];
				mcp23s17_->pinMode(pins[i], _INPUT_PULLUP);
			}
			resetState();
		}
	
		void SetControl(const char* msg) { msg_ = msg; }
		void resetThisState() { resetState(); }
	};
	
	typedef SwitchMultiPosT<POLL_EVERY_TIME, 3> Switch3Pos;
	typedef SwitchMultiPosT<POLL_EVERY_TIME, 4> Switch4Pos;
	typedef SwitchMultiPosT<POLL_EVERY_TIME, 5> Switch5Pos;

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
	class Switch3Pos2PinT : PollingInput, public ResettableInput {
	private:
		const char* msg_;
		bool useExpander_;
		enum ExpanderType { NONE, AW9523B_TYPE, PCF8575_TYPE, MCP23S17_TYPE };
		ExpanderType expanderType_;
		
		// Expander pointers
		AW9523B* aw9523b_;
		PCF8575* pcf8575_;
		MCP23S17* mcp23s17_;
		
		uint8_t pins_[2];                 // pins_[0] = A, pins_[1] = B

		int8_t lastState_;                // -1 unknown, else 0/1/2
		int8_t debounceCandidate_;        // candidate logical state awaiting debounce
		unsigned long debounceDelay_;     // like other classes
		unsigned long lastDebounceTime_;  // when candidate was (re)observed

		// Center watchdog (forces center after stable HIGH/HIGH for debounceDelay_)
		unsigned long centerStableSince_;

		// Active-LOW read (external pull-ups)
		inline char readInput(uint8_t idx) {
			if (useExpander_) {
				switch (expanderType_) {
					case AW9523B_TYPE:
						if (aw9523b_) return aw9523b_->readPin(pins_[idx]);
						break;
					case PCF8575_TYPE:
						if (pcf8575_) return pcf8575_->digitalRead(pins_[idx]);
						break;
					case MCP23S17_TYPE:
						if (mcp23s17_) return mcp23s17_->digitalRead(pins_[idx]);
						break;
					default:
						break;
				}
				return 0;
			}
			return gpio_get(pins_[idx]);
		}

		// Map raw A/B to logical position; return -1 if invalid (both active)
		inline int8_t mapLogical(bool aActive, bool bActive) {
			if (aActive && bActive)   return -1; // transient during travel: ignore, hold last
			if (aActive && !bActive)  return 0;  // A thrown
			if (!aActive && !bActive) return 1;  // center
			/* !aActive && bActive */ return 2;  // B thrown
		}

		void resetState() {
			lastState_ = -1;
			debounceCandidate_ = -1;
			lastDebounceTime_ = 0;
			centerStableSince_ = 0;
		}

		void pollInput() {
			const unsigned long now = to_ms_since_boot(get_absolute_time());

			// Read raw pins (active when LOW)
			const bool aActive = (readInput(0) == 0);
			const bool bActive = (readInput(1) == 0);

			// Center watchdog: if both HIGH and stable for debounceDelay_, force center=1
			const bool isCenterPhysical = (!aActive && !bActive);
			if (isCenterPhysical) {
				if (centerStableSince_ == 0) centerStableSince_ = now;
				if ((now - centerStableSince_) >= debounceDelay_) {
					if (lastState_ != 1) {
						char msgBuffer[3];
						snprintf(msgBuffer, sizeof(msgBuffer), "%d", 1);
						if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
							lastState_ = 1;
						}
					}
					// Align logical debounce state with enforced center and exit
					debounceCandidate_ = 1;
					lastDebounceTime_  = now;
					return;
				}
			} else {
				centerStableSince_ = 0; // reset when not physically centered
			}

			// Determine current logical candidate
			const int8_t logical = mapLogical(aActive, bActive);

			// If invalid combo (both active), do not change candidate; just hold last.
			if (logical < 0) return;

			// Debounce the logical state (like your other classes)
			if (logical != debounceCandidate_) {
				debounceCandidate_ = logical;
				lastDebounceTime_  = now;
			}

			if ((now - lastDebounceTime_) >= debounceDelay_) {
				if (debounceCandidate_ != lastState_) {
					char msgBuffer[3];
					snprintf(msgBuffer, sizeof(msgBuffer), "%d", debounceCandidate_);
					if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
						lastState_ = debounceCandidate_;
					}
				}
			}
		}

	public:
		// GPIO constructor (two discrete pins)
		Switch3Pos2PinT(const char* msg, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(false),
			expanderType_(NONE),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			centerStableSince_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			gpio_init(pins_[0]); gpio_pull_up(pins_[0]); gpio_set_dir(pins_[0], GPIO_IN);
			gpio_init(pins_[1]); gpio_pull_up(pins_[1]); gpio_set_dir(pins_[1], GPIO_IN);
			resetState();
		}

		// AW9523B constructor (two pins on expander)
		Switch3Pos2PinT(const char* msg, AW9523B* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(true),
			expanderType_(AW9523B_TYPE),
			aw9523b_(expander),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			centerStableSince_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			aw9523b_->setPinInput(pins_[0]); // no internal pulls; external pull-ups present
			aw9523b_->setPinInput(pins_[1]);
			resetState();
		}

		// PCF8575 constructor (two pins on expander)
		Switch3Pos2PinT(const char* msg, PCF8575* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(true),
			expanderType_(PCF8575_TYPE),
			pcf8575_(expander),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			centerStableSince_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			pcf8575_->pinMode(pins_[0], GPIO_IN);
			pcf8575_->pullUp(pins_[0], true);
			pcf8575_->pinMode(pins_[1], GPIO_IN);
			pcf8575_->pullUp(pins_[1], true);
			resetState();
		}

		// MCP23S17 constructor (two pins on expander)
		Switch3Pos2PinT(const char* msg, MCP23S17* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(true),
			expanderType_(MCP23S17_TYPE),
			mcp23s17_(expander),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			centerStableSince_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			mcp23s17_->pinMode(pins_[0], _INPUT_PULLUP);
			mcp23s17_->pinMode(pins_[1], _INPUT_PULLUP);
			resetState();
		}

		void SetControl(const char* msg) { msg_ = msg; }
		void resetThisState() { resetState(); }
	};
	typedef Switch3Pos2PinT<> Switch3Pos2Pin;

	// Switch3PosLatchedMomentary - A 3-position momentary switch that only sends states 0 and 2
	// Enhanced version with state reset capability
	template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
	class Switch3PosLatchedMomentaryT : PollingInput, public ResettableInput {
	private:
		const char* msg_;
		bool useExpander_;
		enum ExpanderType { NONE, AW9523B_TYPE, PCF8575_TYPE, MCP23S17_TYPE };
		ExpanderType expanderType_;
		
		// Expander pointers
		AW9523B* aw9523b_;
		PCF8575* pcf8575_;
		MCP23S17* mcp23s17_;
		
		uint8_t pins_[2];                 // pins_[0] = A, pins_[1] = B

		int8_t lastState_;                // -1 unknown, else 0/1/2 (logical state)
		int8_t debounceCandidate_;        // candidate logical state awaiting debounce
		unsigned long debounceDelay_;     // like other classes
		unsigned long lastDebounceTime_;  // when candidate was (re)observed
		
		// New: Track when we fail to send a message (indicating game state mismatch)
		bool messageFailure_;
		unsigned long lastFailureTime_;
		unsigned long lastCenterTime_;
		static const unsigned long FAILURE_RESET_DELAY = 100; // ms

		// Active-LOW read (external pull-ups)
		inline char readInput(uint8_t idx) {
			if (useExpander_) {
				switch (expanderType_) {
					case AW9523B_TYPE:
						if (aw9523b_) return aw9523b_->readPin(pins_[idx]);
						break;
					case PCF8575_TYPE:
						if (pcf8575_) return pcf8575_->digitalRead(pins_[idx]);
						break;
					case MCP23S17_TYPE:
						if (mcp23s17_) return mcp23s17_->digitalRead(pins_[idx]);
						break;
					default:
						break;
				}
				return 0;
			}
			return gpio_get(pins_[idx]);
		}

		// Map raw A/B to logical position; return -1 if invalid (both active)
		inline int8_t mapLogical(bool aActive, bool bActive) {
			if (aActive && bActive)   return -1; // transient during travel: ignore, hold last
			if (aActive && !bActive)  return 0;  // A thrown
			if (!aActive && !bActive) return 1;  // center
			/* !aActive && bActive */ return 2;  // B thrown
		}

		void resetState() {
			lastState_ = -1;
			debounceCandidate_ = -1;
			lastDebounceTime_ = 0;
			messageFailure_ = false;
			lastFailureTime_ = 0;
			lastCenterTime_ = 0;
		}

		void pollInput() {
			const unsigned long now = to_ms_since_boot(get_absolute_time());

			// Read raw pins (active when LOW)
			const bool aActive = (readInput(0) == 0);
			const bool bActive = (readInput(1) == 0);

			// Determine current logical candidate
			const int8_t logical = mapLogical(aActive, bActive);

			// If invalid combo (both active), do not change candidate; just hold last.
			if (logical < 0) return;

			// Handle message failure recovery
			if (messageFailure_ && (now - lastFailureTime_) > FAILURE_RESET_DELAY) {
				// If we're back at center position after a failure, reset our state tracking
				if (logical == 1) {
					lastState_ = -1; // Reset so next valid position will be sent
					messageFailure_ = false;
				}
			}

			// Only send messages for states 0 (up) and 2 (down)
			// Ignore state 1 (center) as it's just the resting position
			if (logical == 0 || logical == 2) {
				// Debounce the logical state (like your other classes)
				if (logical != debounceCandidate_) {
					debounceCandidate_ = logical;
					lastDebounceTime_  = now;
				}

				if ((now - lastDebounceTime_) >= debounceDelay_) {
					// Map physical positions to game values: 0->1, 2->0
					int8_t gameValue = (debounceCandidate_ == 0) ? 1 : 0;
					if (gameValue != lastState_) {
						char msgBuffer[3];
						snprintf(msgBuffer, sizeof(msgBuffer), "%d", gameValue);
						if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
							lastState_ = gameValue;  // Store the game value we sent, not physical position
							messageFailure_ = false; // Clear any previous failure
						} else {
							// Message failed to send - likely because game is already in this state
							messageFailure_ = true;
							lastFailureTime_ = now;
						}
					}
				}
			} else if (logical == 1) {
				// We're at center position - reset state tracking after a timeout
				// This allows the switch to be reactivated even if game state changed
				if (now - lastCenterTime_ > 50) { // 50ms timeout
					lastState_ = -1; // Reset state tracking to allow reactivation
				}
				lastCenterTime_ = now;
			} else if (logical == 1 && messageFailure_) {
				// We're at center after a failure - this is a good time to reset
				// But wait a bit to ensure we're stable at center
				if ((now - lastFailureTime_) > FAILURE_RESET_DELAY) {
					lastState_ = -1; // Reset so next valid position will be sent
					messageFailure_ = false;
				}
			}
		}

	public:
		// GPIO constructor (two discrete pins)
		Switch3PosLatchedMomentaryT(const char* msg, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(false),
			expanderType_(NONE),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			messageFailure_(false),
			lastFailureTime_(0),
			lastCenterTime_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			gpio_init(pins_[0]); gpio_pull_up(pins_[0]); gpio_set_dir(pins_[0], GPIO_IN);
			gpio_init(pins_[1]); gpio_pull_up(pins_[1]); gpio_set_dir(pins_[1], GPIO_IN);
			resetState();
		}

		// AW9523B constructor (two pins on expander)
		Switch3PosLatchedMomentaryT(const char* msg, AW9523B* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(true),
			expanderType_(AW9523B_TYPE),
			aw9523b_(expander),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			messageFailure_(false),
			lastFailureTime_(0),
			lastCenterTime_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			aw9523b_->setPinInput(pins_[0]); // no internal pulls; external pull-ups present
			aw9523b_->setPinInput(pins_[1]);
			resetState();
		}

		// PCF8575 constructor (two pins on expander)
		Switch3PosLatchedMomentaryT(const char* msg, PCF8575* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(true),
			expanderType_(PCF8575_TYPE),
			pcf8575_(expander),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			messageFailure_(false),
			lastFailureTime_(0),
			lastCenterTime_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			pcf8575_->pinMode(pins_[0], GPIO_IN); // no internal pulls; external pull-ups present
			pcf8575_->pullUp(pins_[0], true);
			pcf8575_->pinMode(pins_[1], GPIO_IN);
			pcf8575_->pullUp(pins_[1], true);
			resetState();
		}

		// MCP23S17 constructor (two pins on expander)
		Switch3PosLatchedMomentaryT(const char* msg, MCP23S17* expander, uint8_t pinA, uint8_t pinB, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			msg_(msg),
			useExpander_(true),
			expanderType_(MCP23S17_TYPE),
			mcp23s17_(expander),
			lastState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			messageFailure_(false),
			lastFailureTime_(0),
			lastCenterTime_(0)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			mcp23s17_->pinMode(pins_[0], _INPUT_PULLUP);
			mcp23s17_->pinMode(pins_[1], _INPUT_PULLUP);
			resetState();
		}

		void SetControl(const char* msg) { msg_ = msg; }
		void resetThisState() { resetState(); }
		
		// New: Manual state reset method for external use
		void forceStateReset() {
			lastState_ = -1;
			messageFailure_ = false;
		}
	};

	// Switch3PosLatchedMomentaryStateAware - A 3-position momentary switch with game state feedback
	// This version knows the current game state and only sends commands when needed
	template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
	class Switch3PosLatchedMomentaryStateAwareT : PollingInput, public ResettableInput, Int16Buffer {
	private:
		const char* msg_;
		bool useExpander_;
		enum ExpanderType { NONE, AW9523B_TYPE, PCF8575_TYPE, MCP23S17_TYPE };
		ExpanderType expanderType_;
		
		// Expander pointers
		AW9523B* aw9523b_;
		PCF8575* pcf8575_;
		MCP23S17* mcp23s17_;
		
		uint8_t pins_[2];                 // pins_[0] = A, pins_[1] = B

		int8_t lastSentState_;            // -1 unknown, else 0/1/2 (last state we sent to DCS)
		int8_t currentGameState_;         // -1 unknown, else 0/1/2 (current game state from DCS-BIOS)
		int8_t physicalState_;            // -1 unknown, else 0/1/2 (current physical switch position)
		int8_t debounceCandidate_;        // candidate logical state awaiting debounce
		unsigned long debounceDelay_;     // debounce time
		unsigned long lastDebounceTime_;  // when candidate was observed

		unsigned int mask_;               // DCS-BIOS mask for extracting data
		unsigned char shift_;             // DCS-BIOS shift for extracting data

		// Active-LOW read (external pull-ups)
		inline char readInput(uint8_t idx) {
			if (useExpander_) {
				switch (expanderType_) {
					case AW9523B_TYPE:
						if (aw9523b_) return aw9523b_->readPin(pins_[idx]);
						break;
					case PCF8575_TYPE:
						if (pcf8575_) return pcf8575_->digitalRead(pins_[idx]);
						break;
					case MCP23S17_TYPE:
						if (mcp23s17_) return mcp23s17_->digitalRead(pins_[idx]);
						break;
					default:
						break;
				}
				return 0;
			}
			return gpio_get(pins_[idx]);
		}

		// Map raw A/B to logical position; return -1 if invalid (both active)
		inline int8_t mapLogical(bool aActive, bool bActive) {
			if (aActive && bActive)   return -1; // transient during travel: ignore, hold last
			if (aActive && !bActive)  return 0;  // A thrown
			if (!aActive && !bActive) return 1;  // center
			/* !aActive && bActive */ return 2;  // B thrown
		}

		void resetState() {
			lastSentState_ = -1;
			currentGameState_ = -1;
			physicalState_ = -1;
			debounceCandidate_ = -1;
			lastDebounceTime_ = 0;
		}

		void pollInput() {
			const unsigned long now = to_ms_since_boot(get_absolute_time());

			// Read raw pins (active when LOW)
			const bool aActive = (readInput(0) == 0);
			const bool bActive = (readInput(1) == 0);

			// Determine current physical state
			const int8_t logical = mapLogical(aActive, bActive);

			// If invalid combo (both active), do not change candidate; just hold last.
			if (logical < 0) return;
			
			// Update physical state
			physicalState_ = logical;

			// Only send messages for states 0 (up) and 2 (down)
			// Ignore state 1 (center) as it's just the resting position
			if (logical == 0 || logical == 2) {
				// Check if we need to send this command
				// Send if: physical position differs from game state, OR we haven't sent anything yet
				// Need to map physical states to game states: 0->1, 2->0
				int8_t gameValue = (logical == 0) ? 1 : 0;  // Map physical 0 to game 1, physical 2 to game 0
				bool shouldSend = (currentGameState_ != gameValue) || (lastSentState_ == -1);
				
				if (shouldSend) {
					// Debounce the logical state
					if (logical != debounceCandidate_) {
						debounceCandidate_ = logical;
						lastDebounceTime_ = now;
					}

					if ((now - lastDebounceTime_) >= debounceDelay_) {
						if (debounceCandidate_ != lastSentState_) {
							char msgBuffer[3];
							int8_t mappedValue = (debounceCandidate_ == 0) ? 1 : 0;  // Map physical to game value
								std::snprintf(msgBuffer, sizeof(msgBuffer), "%d", mappedValue);
							if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
								lastSentState_ = mappedValue;  // Store the game value we sent, not the physical position
							}
						}
					}
				}
			}
			// For center position (1), we don't send anything - game state remains unchanged
		}

	public:
		// GPIO constructor (two discrete pins)
		Switch3PosLatchedMomentaryStateAwareT(const char* msg, uint8_t pinA, uint8_t pinB, 
											 unsigned int address, unsigned int mask, unsigned char shift,
											 unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			Int16Buffer(address),
			msg_(msg),
			useExpander_(false),
			expanderType_(NONE),
			lastSentState_(-1),
			currentGameState_(-1),
			physicalState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			mask_(mask),
			shift_(shift)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			gpio_init(pins_[0]); gpio_pull_up(pins_[0]); gpio_set_dir(pins_[0], GPIO_IN);
			gpio_init(pins_[1]); gpio_pull_up(pins_[1]); gpio_set_dir(pins_[1], GPIO_IN);
			resetState();
		}

		// AW9523B constructor (two pins on expander)
		Switch3PosLatchedMomentaryStateAwareT(const char* msg, AW9523B* expander, uint8_t pinA, uint8_t pinB,
											 unsigned int address, unsigned int mask, unsigned char shift,
											 unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			Int16Buffer(address),
			msg_(msg),
			useExpander_(true),
			expanderType_(AW9523B_TYPE),
			aw9523b_(expander),
			lastSentState_(-1),
			currentGameState_(-1),
			physicalState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			mask_(mask),
			shift_(shift)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			aw9523b_->setPinInput(pins_[0]);
			aw9523b_->setPinInput(pins_[1]);
			resetState();
		}

		// PCF8575 constructor (two pins on expander)
		Switch3PosLatchedMomentaryStateAwareT(const char* msg, PCF8575* expander, uint8_t pinA, uint8_t pinB,
											 unsigned int address, unsigned int mask, unsigned char shift,
											 unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			Int16Buffer(address),
			msg_(msg),
			useExpander_(true),
			expanderType_(PCF8575_TYPE),
			pcf8575_(expander),
			lastSentState_(-1),
			currentGameState_(-1),
			physicalState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			mask_(mask),
			shift_(shift)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			pcf8575_->pinMode(pins_[0], GPIO_IN);
			pcf8575_->pullUp(pins_[0], true);
			pcf8575_->pinMode(pins_[1], GPIO_IN);
			pcf8575_->pullUp(pins_[1], true);
			resetState();
		}

		// MCP23S17 constructor (two pins on expander)
		Switch3PosLatchedMomentaryStateAwareT(const char* msg, MCP23S17* expander, uint8_t pinA, uint8_t pinB,
											 unsigned int address, unsigned int mask, unsigned char shift,
											 unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs),
			Int16Buffer(address),
			msg_(msg),
			useExpander_(true),
			expanderType_(MCP23S17_TYPE),
			mcp23s17_(expander),
			lastSentState_(-1),
			currentGameState_(-1),
			physicalState_(-1),
			debounceCandidate_(-1),
			debounceDelay_(debounceDelay),
			lastDebounceTime_(0),
			mask_(mask),
			shift_(shift)
		{
			pins_[0] = pinA; pins_[1] = pinB;
			mcp23s17_->pinMode(pins_[0], _INPUT_PULLUP);
			mcp23s17_->pinMode(pins_[1], _INPUT_PULLUP);
			resetState();
		}

		// Override the loop method to handle DCS-BIOS data updates
		virtual void loop() override {
			// Call parent loop to handle DCS-BIOS data reception
			Int16Buffer::loop();
			
			// Check if we have updated data from DCS-BIOS
			if (hasUpdatedData()) {
				// Extract the current game state using mask and shift
				unsigned int rawData = getData();
				currentGameState_ = (int8_t)((rawData & mask_) >> shift_);
			}
		}

		void SetControl(const char* msg) { msg_ = msg; }
		void resetThisState() { resetState(); }
		
		// Method to get current game state (useful for debugging)
		int8_t getCurrentGameState() const { return currentGameState_; }
		int8_t getPhysicalState() const { return physicalState_; }
		int8_t getLastSentState() const { return lastSentState_; }
	};

	typedef Switch3PosLatchedMomentaryStateAwareT<> Switch3PosLatchedMomentaryStateAware;
	typedef Switch3PosLatchedMomentaryT<> Switch3PosLatchedMomentary;
}
#endif