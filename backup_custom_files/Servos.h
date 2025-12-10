#ifndef __DCSBIOS_SERVOS_H
#define __DCSBIOS_SERVOS_H

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "ExportStreamListener.h"

namespace DcsBios {

class ServoOutput : public Int16Buffer {
private:
	uint pin_;
	uint slice_;
	int minPulseWidth_;
	int maxPulseWidth_;
	unsigned int (*map_function_)(unsigned int newValue);
	bool initialized_ = false;

	void init_pwm() {
		gpio_set_function(pin_, GPIO_FUNC_PWM);
		slice_ = pwm_gpio_to_slice_num(pin_);

		// Set 50Hz = 20ms period for servo control
		// System clock is 125MHz, to get 50Hz: 125,000,000 / 50 = 2,500,000 cycles per servo cycle
		// With clkdiv=64: 125,000,000 / 64 = 1,953,125 Hz, so cycles per period: 1,953,125 / 50 = 39,062.5
		pwm_config config = pwm_get_default_config();
		pwm_config_set_clkdiv(&config, 64.f);               // slower clock for more resolution
		pwm_config_set_wrap(&config, 39062);                // 50Hz period (20ms)
		pwm_init(slice_, &config, true);
		initialized_ = true;
	}

	uint16_t pulseWidthToPwm(uint32_t us) {
		// Convert microseconds to PWM level based on 20ms (20000us) period
		// 20000us full period → wrap value is 39062 → 1us ≈ 1.95 PWM ticks
		uint32_t level = (us * 39062) / 20000;
		if (level > 39062) level = 39062;
		return level;
	}

public:
	ServoOutput(unsigned int address, uint pin, int minPulseWidth = 544, int maxPulseWidth = 2400)
		: Int16Buffer(address), pin_(pin), minPulseWidth_(minPulseWidth), maxPulseWidth_(maxPulseWidth), map_function_(nullptr) {}

	ServoOutput(unsigned int address, uint pin, int minPulseWidth, int maxPulseWidth, unsigned int (*map_function)(unsigned int newValue))
		: Int16Buffer(address), pin_(pin), minPulseWidth_(minPulseWidth), maxPulseWidth_(maxPulseWidth), map_function_(map_function) {}

	void loop() override {
		if (!initialized_) init_pwm();

		if (hasUpdatedData()) {
			uint16_t us = mapValue(getData());
			pwm_set_gpio_level(pin_, pulseWidthToPwm(us));
		}
	}

	unsigned int mapValue(unsigned int value) {
		if (map_function_) {
			return map_function_(value);
		} else {
			return (value * (maxPulseWidth_ - minPulseWidth_) / 65535) + minPulseWidth_;
		}
	}
};

} // namespace DcsBios

#endif
