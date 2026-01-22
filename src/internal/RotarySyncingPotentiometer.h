#ifndef __DCSBIOS_ROTARYPOTS_H
#define __DCSBIOS_ROTARYPOTS_H

#include <math.h>
#include <stdlib.h>
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/adc.h"

namespace DcsBios {

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, bool invert = false>
	class RotarySyncingPotentiometerEWMA : PollingInput, Int16Buffer, public ResettableInput {
	private:
		void resetState() {
			lastState_ = (lastState_==0)?-1:0;
		}

		void pollInput() {
			lastState_ = readState();
		}

		inline unsigned int readState() {
			adc_select_input(adc_channel_);
			uint raw = adc_read();
			return mapInt(raw, invert ? 4095 : 0, invert ? 0 : 4095, 0, 65535);
		}

		const char* msg_;
		char gpio_pin_;
		uint8_t adc_channel_;
		unsigned int lastState_;
		unsigned int mask;
		unsigned char shift;
		unsigned long lastSendTime;
		int (*mapperCallback)(unsigned int, unsigned int);

	public:
		RotarySyncingPotentiometerEWMA(const char* msg, char gpio_pin,
			unsigned int syncToAddress, unsigned int syncToMask, unsigned char syncToShift,
			int (*mapperCallback)(unsigned int, unsigned int)) :
			PollingInput(pollIntervalMs), Int16Buffer(syncToAddress)
		{
			msg_ = msg;
			gpio_pin_ = gpio_pin;

			adc_init();
			switch (gpio_pin_) {
				case 26: adc_channel_ = 0; break;
				case 27: adc_channel_ = 1; break;
				case 28: adc_channel_ = 2; break;
				default: adc_channel_ = 0; break; // fallback
			}
			adc_gpio_init(gpio_pin_);

			lastState_ = readState();

			mask = syncToMask;
			shift = syncToShift;
			lastSendTime = to_ms_since_boot(get_absolute_time());
			this->mapperCallback = mapperCallback;
		}

		void SetControl(const char* msg) {
			msg_ = msg;
		}

		void resetThisState() {
			this->resetState();
		}

		unsigned int getData() {
			return ((this->Int16Buffer::getData()) & mask) >> shift;
		}

		virtual void loop() {
			unsigned int dcsData = getData();
			int requiredAdjustment = mapperCallback(lastState_, dcsData);

			if (requiredAdjustment != 0) {
				if (to_ms_since_boot(get_absolute_time()) - lastSendTime > 100) {
					char buff[7];
					std::snprintf(buff, sizeof(buff), "%+d", requiredAdjustment);
					tryToSendDcsBiosMessage(msg_, buff);
					lastSendTime = to_ms_since_boot(get_absolute_time());
				}
			}
		}
	};

	typedef RotarySyncingPotentiometerEWMA<> RotarySyncingPotentiometer;
	typedef RotarySyncingPotentiometerEWMA<POLL_EVERY_TIME, true> InvertedRotarySyncingPotentiometer;

}

#endif
