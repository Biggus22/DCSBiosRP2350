#ifndef __DCSBIOS_POTS_H
#define __DCSBIOS_POTS_H

#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

namespace DcsBios {

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, unsigned int hysteresis = 128, unsigned int ewma_divisor = 5>
	class PotentiometerEWMA : PollingInput, public ResettableInput {
	private:
		// This function resets the last state.
void resetState() {
			lastState_ = (lastState_==0)?-1:0;
		}

		// This function reads the ADC input. It maps the raw value to an unsigned integer state. It updates the accumulator using the EWMA divisor. It calculates a target state. It then checks if the state change exceeds the hysteresis threshold. If the change is significant, it sends the state message.
void pollInput() {
			uint rawValue;

			adc_select_input(adc_channel_);
			rawValue = adc_read();

			unsigned int state;
			if (reverse_)
				state = mapInt(rawValue, input_min_, input_max_, 65535, 0);
			else
				state = mapInt(rawValue, input_min_, input_max_, 0, 65535);

			accumulator += ((float)state - accumulator) / (float)ewma_divisor;
			state = (unsigned int)accumulator;

			// Limit the maximum step sent per poll to avoid visible hitching.
			const unsigned int maxStep = 2048;

			unsigned int target = state;
			unsigned int sendState = target;

			if (target > lastState_ + maxStep) {
				sendState = lastState_ + maxStep;
			} else if (target + maxStep < lastState_) {
				sendState = lastState_ - maxStep;
			}

			// Only send if change exceeds hysteresis (or near extremes)
			if (((lastState_ > sendState && (lastState_ - sendState > hysteresis)))
				|| ((sendState > lastState_) && (sendState - lastState_ > hysteresis))
				|| ((sendState > (65535 - hysteresis) && sendState > lastState_))
				|| ((sendState < hysteresis && sendState < lastState_))
			) {
				char buf[6];
				utoa(sendState, buf, 10);
				if (tryToSendDcsBiosMessage(msg_, buf))
					lastState_ = sendState;
			}
		}

		const char* msg_;
		char gpio_pin_;
		uint8_t adc_channel_;
		unsigned int lastState_;
		float accumulator;
		bool reverse_;
		unsigned int input_min_;
		unsigned int input_max_;

	public:
		PotentiometerEWMA(const char* msg, char gpio_pin, bool reverse = false, unsigned int input_min = 0, unsigned int input_max = 4095)
			: PollingInput(pollIntervalMs) {
			msg_ = msg;
			gpio_pin_ = gpio_pin;
			reverse_ = reverse;
			input_min_ = input_min;
			input_max_ = input_max;

			adc_init();

			switch (gpio_pin) {
				case 26: adc_channel_ = 0; break;
				case 27: adc_channel_ = 1; break;
				case 28: adc_channel_ = 2; break;
				case 29: adc_channel_ = 3; break;
				default: adc_channel_ = 0; break; // fallback
			}

			adc_gpio_init(gpio_pin_);

			adc_select_input(adc_channel_);
			uint raw = adc_read();

			if (reverse_)
				lastState_ = mapInt(raw, input_min_, input_max_, 65535, 0);
			else
				lastState_ = mapInt(raw, input_min_, input_max_, 0, 65535);

			accumulator = lastState_;
		}

		// This function sets the message string.
void SetControl(const char* msg) {
			msg_ = msg;
		}

		void resetThisState() {
			this->resetState();
		}
	};

	// Low-resolution potentiometer: reduce ADC resolution by dropping low bits
	// to reduce sensitivity and message churn with minimal CPU cost.
	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, unsigned int hysteresis = 128, unsigned int dropBits = 2>
	class LowResPotentiometer : PollingInput, public ResettableInput {
	private:
		void resetState() {
			lastState_ = (lastState_==0)?-1:0;
		}

		void pollInput() {
			adc_select_input(adc_channel_);
			uint rawValue = adc_read();

			// Quantize by dropping `dropBits` LSBs (simple shift with rounding)
			const uint quant = (rawValue + (1u << (dropBits ? (dropBits - 1) : 0))) >> dropBits;
			const unsigned int maxRaw = 4095u >> dropBits;

			unsigned int state;
			if (reverse_)
				state = mapInt(quant, 0, maxRaw, 65535, 0);
			else
				state = mapInt(quant, 0, maxRaw, 0, 65535);

			// Only send if change exceeds hysteresis (or at extremes)
			if (((lastState_ > state && (lastState_ - state > hysteresis)))
				|| ((state > lastState_) && (state - lastState_ > hysteresis))
				|| ((state > (65535 - hysteresis) && state > lastState_))
				|| ((state < hysteresis && state < lastState_))
			) {
				char buf[6];
				utoa(state, buf, 10);
				if (tryToSendDcsBiosMessage(msg_, buf))
					lastState_ = state;
			}
		}

		const char* msg_;
		char gpio_pin_;
		uint8_t adc_channel_;
		unsigned int lastState_;
		bool reverse_;
		unsigned int input_min_;
		unsigned int input_max_;

	public:
		LowResPotentiometer(const char* msg, char gpio_pin, bool reverse = false, unsigned int input_min = 0, unsigned int input_max = 4095)
			: PollingInput(pollIntervalMs) {
			msg_ = msg;
			gpio_pin_ = gpio_pin;
			reverse_ = reverse;
			input_min_ = input_min;
			input_max_ = input_max;

			adc_init();
			switch (gpio_pin) {
				case 26: adc_channel_ = 0; break;
				case 27: adc_channel_ = 1; break;
				case 28: adc_channel_ = 2; break;
				case 29: adc_channel_ = 3; break;
				default: adc_channel_ = 0; break;
			}
			adc_gpio_init(gpio_pin_);
			adc_select_input(adc_channel_);
			uint raw = adc_read();
			const uint quant = (raw + (1u << (dropBits ? (dropBits - 1) : 0))) >> dropBits;
			const unsigned int maxRaw = 4095u >> dropBits;
			if (reverse_)
				lastState_ = mapInt(quant, 0, maxRaw, 65535, 0);
			else
				lastState_ = mapInt(quant, 0, maxRaw, 0, 65535);
		}

		void SetControl(const char* msg) {
			msg_ = msg;
		}

		void resetThisState() {
			this->resetState();
		}
	};

	// Low-res EWMA potentiometer: combine low-resolution quantization with light EWMA
	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, unsigned int hysteresis = 128, unsigned int dropBits = 2, unsigned int ewma_divisor = 5>
	class LowResPotentiometerEWMA : PollingInput, public ResettableInput {
	private:
		void resetState() {
			lastState_ = (lastState_==0)?-1:0;
		}

		void pollInput() {
			adc_select_input(adc_channel_);
			uint rawValue = adc_read();

			const uint quant = (rawValue + (1u << (dropBits ? (dropBits - 1) : 0))) >> dropBits;
			const unsigned int maxRaw = 4095u >> dropBits;

			unsigned int state;
			if (reverse_)
				state = mapInt(quant, 0, maxRaw, 65535, 0);
			else
				state = mapInt(quant, 0, maxRaw, 0, 65535);

			accumulator += ((float)state - accumulator) / (float)ewma_divisor;
			state = (unsigned int)accumulator;

			// Only send if change exceeds hysteresis (or at extremes)
			if (((lastState_ > state && (lastState_ - state > hysteresis)))
				|| ((state > lastState_) && (state - lastState_ > hysteresis))
				|| ((state > (65535 - hysteresis) && state > lastState_))
				|| ((state < hysteresis && state < lastState_))
			) {
				char buf[6];
				utoa(state, buf, 10);
				if (tryToSendDcsBiosMessage(msg_, buf))
					lastState_ = state;
			}
		}

		const char* msg_;
		char gpio_pin_;
		uint8_t adc_channel_;
		unsigned int lastState_;
		float accumulator;
		bool reverse_;
		unsigned int input_min_;
		unsigned int input_max_;

	public:
		LowResPotentiometerEWMA(const char* msg, char gpio_pin, bool reverse = false, unsigned int input_min = 0, unsigned int input_max = 4095)
			: PollingInput(pollIntervalMs) {
			msg_ = msg;
			gpio_pin_ = gpio_pin;
			reverse_ = reverse;
			input_min_ = input_min;
			input_max_ = input_max;

			adc_init();
			switch (gpio_pin) {
				case 26: adc_channel_ = 0; break;
				case 27: adc_channel_ = 1; break;
				case 28: adc_channel_ = 2; break;
				case 29: adc_channel_ = 3; break;
				default: adc_channel_ = 0; break;
			}
			adc_gpio_init(gpio_pin_);
			adc_select_input(adc_channel_);
			uint raw = adc_read();
			const uint quant = (raw + (1u << (dropBits ? (dropBits - 1) : 0))) >> dropBits;
			const unsigned int maxRaw = 4095u >> dropBits;
			if (reverse_)
				lastState_ = mapInt(quant, 0, maxRaw, 65535, 0);
			else
				lastState_ = mapInt(quant, 0, maxRaw, 0, 65535);

			accumulator = lastState_;
		}

		void SetControl(const char* msg) {
			msg_ = msg;
		}

		void resetThisState() {
			this->resetState();
		}
	};

	typedef PotentiometerEWMA<> Potentiometer;
}

#endif
