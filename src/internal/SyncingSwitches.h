#ifndef __DCSBIOS_SYNCING_SWITCHES_H
#define __DCSBIOS_SYNCING_SWITCHES_H

#include <math.h>
#include <cstdio>
#include "pico/stdlib.h"

namespace DcsBios {

static inline unsigned int normalizeExportedValueToIndex(unsigned int fullVal, unsigned int maskedVal, unsigned int numPositions) {
	// If mask-derived value already fits the expected index range, use it.
	if (numPositions <= 1) return 0;
	if (maskedVal < (unsigned int)numPositions) return maskedVal;
	// If the full exported value already looks like an index, use it.
	if (fullVal < (unsigned int)numPositions) return fullVal;

	// Try several common scales (0-100, 0-255, 0-65535).
	if (fullVal <= 100) {
		unsigned int step = 100 / (numPositions - 1);
		return (fullVal + step/2) / (step ? step : 1);
	}
	if (fullVal <= 255) {
		unsigned int step = 255 / (numPositions - 1);
		return (fullVal + step/2) / (step ? step : 1);
	}
	unsigned int step = 65535 / (numPositions - 1);
	return (fullVal + step/2) / (step ? step : 1);
}

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
	class SyncingSwitch2PosT : PollingInput, public ResettableInput, Int16Buffer {
	private:
		const char* msg_;
		char pin_;
		bool reverse_;
		char lastState_;
		char steadyState_;
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime = 0;
		unsigned int mask;
		unsigned char shift;

		char readState() {
			char state = gpio_get(pin_);
			if (reverse_) state = !state;
			return (state == 1) ? 0 : 1;
		}

		void resetState() {
			lastState_ = (lastState_ == 0) ? -1 : 0;
			steadyState_ = lastState_;
		}

		void pollInput() {
			char state = readState();
			if (state != lastState_) {
				lastDebounceTime = to_ms_since_boot(get_absolute_time());
			}

			if ((to_ms_since_boot(get_absolute_time()) - lastDebounceTime) > debounceDelay_) {
				if (state != steadyState_) {
					if (tryToSendDcsBiosMessage(msg_, state == 0 ? "0" : "1")) {
						steadyState_ = state;
					}
				}
			}

			lastState_ = state;
		}

	public:
		SyncingSwitch2PosT(const char* msg, char pin,
			unsigned int syncToAddress, unsigned int syncToMask, unsigned char syncToShift,
			bool reverse = false, unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), Int16Buffer(syncToAddress)
		{
			msg_ = msg;
			pin_ = pin;
			reverse_ = reverse;
			gpio_init(pin_);
			gpio_pull_up(pin_);
			gpio_set_dir(pin_, GPIO_IN);
			lastState_ = readState();
			steadyState_ = lastState_;
			debounceDelay_ = debounceDelay;
			this->mask = syncToMask;
			this->shift = syncToShift;
		}

		void SetControl(const char* msg) { msg_ = msg; }

		void resetThisState() { this->resetState(); }

		unsigned int getData() {
			unsigned int full = this->Int16Buffer::getData();
			unsigned int maskedVal = ((full) & mask) >> shift;
			return normalizeExportedValueToIndex(full, maskedVal, 2);
		}

		virtual void loop() {
			if (hasUpdatedData()) {
				unsigned int dcsData = getData();
				lastState_ = dcsData;
				steadyState_ = dcsData;
			}
		}
	};

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
	class SyncingSwitch3PosT : PollingInput, public ResettableInput, Int16Buffer {
	private:
		const char* msg_;
		char pinA_;
		char pinB_;
		char lastState_;
		char steadyState_;
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime = 0;

		unsigned int mask;
		unsigned char shift;

		char readState() {
			if (gpio_get(pinA_) == 0) return 0;
			if (gpio_get(pinB_) == 0) return 2;
			return 1;
		}

		void resetState() {
			lastState_ = (lastState_==0)?-1:0;
			steadyState_ = lastState_;
		}

		void pollInput() {
			char state = readState();
			if (state != lastState_) {
				lastDebounceTime = to_ms_since_boot(get_absolute_time());
			}

			if ((to_ms_since_boot(get_absolute_time()) - lastDebounceTime) > debounceDelay_) {
				if (state != steadyState_) {
					if (state == 0) {
						if (tryToSendDcsBiosMessage(msg_, "0"))
							steadyState_ = state;
					}
					else if (state == 1) {
						if (tryToSendDcsBiosMessage(msg_, "1"))
							steadyState_ = state;
					}
					else if (state == 2) {
						if (tryToSendDcsBiosMessage(msg_, "2"))
							steadyState_ = state;
					}
				}
			}

			lastState_ = state;
		}

	public:
		SyncingSwitch3PosT(const char* msg, char pinA, char pinB,
			unsigned int syncToAddress, unsigned int syncToMask, unsigned char syncToShift,
			unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), Int16Buffer(syncToAddress)
		{
			msg_ = msg;
			pinA_ = pinA;
			pinB_ = pinB;
			gpio_init(pinA_);
			gpio_pull_up(pinA_);
			gpio_set_dir(pinA_, GPIO_IN);
			gpio_init(pinB_);
			gpio_pull_up(pinB_);
			gpio_set_dir(pinB_, GPIO_IN);
			lastState_ = readState();
			steadyState_ = lastState_;
			debounceDelay_ = debounceDelay;

			this->mask = syncToMask;
			this->shift = syncToShift;
		}

		void SetControl(const char* msg) {
			msg_ = msg;
		}

		void resetThisState() {
			this->resetState();
		}

		unsigned int getData() {
			unsigned int full = this->Int16Buffer::getData();
			unsigned int maskedVal = ((full) & mask) >> shift;
			return normalizeExportedValueToIndex(full, maskedVal, 3);
		}

		virtual void loop() {
			if (hasUpdatedData()) {
				unsigned int dcsData = getData();
				lastState_ = dcsData;
			}
		}
	};

	typedef SyncingSwitch3PosT<> SyncingSwitch3Pos;
// Center-aware variant: treats "no pin active" as the center position for odd-numbered switches
// and reports the center index (numPositions/2). Use this when a 3-pos hardware switch leaves
// no pin active in the center position (make-before-break or open-center wiring).
template <unsigned long pollIntervalMs = POLL_EVERY_TIME, int numPositions = 3>
class SyncingSwitchMultiPosCenterT : PollingInput, public ResettableInput, Int16Buffer {
private:
	const char* msg_;
	const uint8_t* pins_;
	char lastState_;
	char steadyState_;
	unsigned long debounceDelay_;
	unsigned long lastDebounceTime_ = 0;
	unsigned int mask;
	unsigned char shift;

	char readState() {
		for (int i = 0; i < numPositions; ++i) {
			if (gpio_get(pins_[i]) == 0) return i; // active-low
		}
		// No pin active -> center for odd counts
		if (numPositions % 2 == 1) return numPositions / 2;
		return 0;
	}

	void resetState() {
		lastState_ = (lastState_==0)?-1:0;
		steadyState_ = lastState_;
	}

	void pollInput() {
		unsigned long now = to_ms_since_boot(get_absolute_time());
		int activeIndex = -1;
		for (int i = 0; i < numPositions; ++i) {
			if (gpio_get(pins_[i]) == 0) { activeIndex = i; break; }
		}
		if (activeIndex < 0) {
			if (numPositions % 2 == 1) activeIndex = numPositions / 2;
			else activeIndex = 0;
		}
		char state = (char)activeIndex;


		if (state != lastState_) {
			lastDebounceTime_ = now;
		}

		if ((now - lastDebounceTime_) > debounceDelay_) {
			if (state != steadyState_) {
					char msgBuffer[4];
				std::snprintf(msgBuffer, sizeof(msgBuffer), "%d", state);

				if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
					steadyState_ = state;
				}
			}
		}

		lastState_ = state;
	}

public:
	SyncingSwitchMultiPosCenterT(const char* msg, const uint8_t* pins,
		unsigned int syncToAddress, unsigned int syncToMask, unsigned char syncToShift,
		unsigned long debounceDelay = 50) :
		PollingInput(pollIntervalMs), Int16Buffer(syncToAddress)
	{
		msg_ = msg;
		pins_ = pins;
		for (int i = 0; i < numPositions; ++i) {
			gpio_init(pins_[i]);
			gpio_pull_up(pins_[i]);
			gpio_set_dir(pins_[i], GPIO_IN);
		}
		lastState_ = readState();
		steadyState_ = lastState_;
		debounceDelay_ = debounceDelay;
		mask = syncToMask;
		shift = syncToShift;
	}

	void SetControl(const char* msg) { msg_ = msg; }

	void resetThisState() { this->resetState(); }

	unsigned int getData() {
		unsigned int full = this->Int16Buffer::getData();
		unsigned int maskedVal = ((full) & mask) >> shift;
		return normalizeExportedValueToIndex(full, maskedVal, numPositions);
	}

	virtual void loop() {
		if (hasUpdatedData()) {
			unsigned int dcsData = getData();
			lastState_ = dcsData;
			steadyState_ = dcsData;
		}
	}
};
	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, int numPositions = 3>
	class SyncingSwitchMultiPosT : PollingInput, public ResettableInput, Int16Buffer {
	private:
		const char* msg_;
		const uint8_t* pins_;
		char lastState_;
		char steadyState_;
		unsigned long debounceDelay_;
		unsigned long lastDebounceTime_ = 0;
		unsigned int mask;
		unsigned char shift;

		char readState() {
			for (int i = 0; i < numPositions; ++i) {
				if (gpio_get(pins_[i]) == 0) return i; // active-low
			}
			return 0;
		}

		void resetState() {
			lastState_ = (lastState_==0)?-1:0;
			steadyState_ = lastState_;
		}

		void pollInput() {
			unsigned long now = to_ms_since_boot(get_absolute_time());
			for (int i = 0; i < numPositions; ++i) {
				char state = gpio_get(pins_[i]) == 0 ? i : -1;
				if (state < 0) continue;
				if (state != lastState_) {
					lastDebounceTime_ = now;
				}
				if ((now - lastDebounceTime_) > debounceDelay_) {
					if (state != steadyState_) {
						char msgBuffer[3];
						std::snprintf(msgBuffer, sizeof(msgBuffer), "%d", state);

						if (tryToSendDcsBiosMessage(msg_, msgBuffer)) {
							steadyState_ = state;
						}
					}
				}
				lastState_ = state;
				break;
			}
		}

	public:
		SyncingSwitchMultiPosT(const char* msg, const uint8_t* pins,
			unsigned int syncToAddress, unsigned int syncToMask, unsigned char syncToShift,
			unsigned long debounceDelay = 50) :
			PollingInput(pollIntervalMs), Int16Buffer(syncToAddress)
		{
			msg_ = msg;
			pins_ = pins;
			for (int i = 0; i < numPositions; ++i) {
				gpio_init(pins_[i]);
				gpio_pull_up(pins_[i]);
				gpio_set_dir(pins_[i], GPIO_IN);
			}
			lastState_ = readState();
			steadyState_ = lastState_;
			debounceDelay_ = debounceDelay;
			mask = syncToMask;
			shift = syncToShift;
		}

		void SetControl(const char* msg) { msg_ = msg; }

		void resetThisState() { this->resetState(); }

		unsigned int getData() {
			return ((this->Int16Buffer::getData()) & mask) >> shift;
		}

		virtual void loop() {
			if (hasUpdatedData()) {
				unsigned int dcsData = getData();
				lastState_ = dcsData;
				steadyState_ = dcsData;
			}
		}
	};
}

#endif
