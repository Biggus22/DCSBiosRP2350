#ifndef __DCSBIOS_ANALOGMULTIPOS_H
#define __DCSBIOS_ANALOGMULTIPOS_H

#include <stdint.h>
#include <cstdio>
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "PollingInput.h"
#include "ExportStreamListener.h"

namespace DcsBios {

template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
class AnalogMultiPosT : public PollingInput, public ResettableInput {
private:
	const char* msg_;
	uint adc_pin_;
	uint adc_channel_;
	unsigned char numOfSteps_;
	unsigned char lastState_;
	uint32_t period_us_ = 750000;
	uint64_t last_time_us_ = 0;

	unsigned char readState() {
		uint16_t raw = adc_read();  // Assumes correct channel already selected
		return (raw * numOfSteps_) / 4096;  // Map 12-bit to step count
	}

	void resetState() {
		lastState_ = (lastState_ == 0) ? -1 : 0;
	}

	void pollInput() {
		uint64_t now = time_us_64();
		if (now - last_time_us_ > period_us_) {
			last_time_us_ = now;
			adc_select_input(adc_channel_);
			unsigned char state = readState();

			if (state != lastState_) {
				char cstr[5];
				std::snprintf(cstr, sizeof(cstr), "%d", state);
				if (tryToSendDcsBiosMessage(msg_, cstr)) {
					lastState_ = state;
				}
			}
		}
	}

public:
	AnalogMultiPosT(const char* msg, uint gpio_adc_pin, unsigned char numOfSteps) :
		PollingInput(pollIntervalMs), msg_(msg), numOfSteps_(numOfSteps)
	{
		adc_pin_ = gpio_adc_pin;
		if (gpio_adc_pin >= 26 && gpio_adc_pin <= 28) {
            adc_channel_ = gpio_adc_pin - 26;
        } else {
            // Fail-safe: default to channel 0 if invalid pin
            adc_channel_ = 0;
        }
		adc_gpio_init(adc_pin_);
		lastState_ = 255;  // Force update on first poll
		last_time_us_ = time_us_64();
	}

	void SetControl(const char* msg) {
		msg_ = msg;
	}

	void resetThisState() {
		resetState();
	}
};

typedef AnalogMultiPosT<> AnalogMultiPos;

} // namespace DcsBios

#endif
