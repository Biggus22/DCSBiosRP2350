#ifndef PICO_BOARD
#define PICO_BOARD
#endif

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// Rotary switch on GP29 via resistor ladder -> single ADC pin.
// GP29 = ADC channel 3 on RP2040/RP2350.
#define ROTARY_ADC_PIN     29
#define ROTARY_ADC_CHANNEL 3

int main() {
    stdio_init_all();
    sleep_ms(2000); // let USB CDC enumerate

    adc_init();
    adc_gpio_init(ROTARY_ADC_PIN);
    adc_select_input(ROTARY_ADC_CHANNEL);

    printf("Rotary switch ADC test - GP%d (ADC%d)\n", ROTARY_ADC_PIN, ROTARY_ADC_CHANNEL);
    fflush(stdout);

    for (;;) {
        uint16_t raw = adc_read();
        printf("raw=%u\n", raw);
        fflush(stdout);
        sleep_ms(100);
    }
}
