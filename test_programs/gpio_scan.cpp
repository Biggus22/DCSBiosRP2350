#ifndef PICO_BOARD
#define PICO_BOARD
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// ==== CONFIG ======================================================
// Scan this GPIO range. RP2350 has GP0..GP45; most breakout boards expose
// the common 0..28. Adjust PIN_START/PIN_END to your board.
#define PIN_START      0
#define PIN_END        28

// Active-low switches: pull up and treat LOW as the activated state.
#define PULL_ACTIVE    1     // 1 = pull-up (active-low to GND), 0 = pull-down

// Debounce window; reject level changes shorter than this (us).
#define DEBOUNCE_US    10000 // 10 ms

// Poll interval between pin sweeps (us).
#define SCAN_US        1000   // re-poll pins every 1 ms
// ================================================================

static uint32_t debounceStart[PIN_END + 1];
static bool debouncePending[PIN_END + 1];

static int theGpCount(void)     { return PIN_END - PIN_START + 1; }

static void print_activated(const bool* activated) {
    printf("ACTIVE: ");
    bool any = false;
    for (int pin = PIN_START; pin <= PIN_END; pin++) {
        if (activated[pin]) {
            printf("GP%d ", pin);
            any = true;
        }
    }
    printf("%s\n", any ? "" : "(none)");
}

static void fingerprint(const bool* activated) {
    printf("\n-------- Startup fingerprint (raw pin levels) --------\n");
    for (int pin = PIN_START; pin <= PIN_END; pin++) {
        printf("  GP%-2d = %s\n", pin, gpio_get(pin) ? "HIGH" : "LOW");
    }
    printf("-------- (of %d pins; LOW = active with pull-up) --------\n", theGpCount());
    print_activated(activated);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);            // allow USB CDC to enumerate

    printf("\nGPIO Switch Scanner  (active-low, pull-up)\n");

    // Configure range as inputs with the chosen pull direction.
    for (int pin = PIN_START; pin <= PIN_END; pin++) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_set_function(pin, GPIO_FUNC_SIO);
        if (PULL_ACTIVE)
            gpio_pull_up(pin);
        else
            gpio_pull_down(pin);
    }

    bool prevLvl[PIN_END + 1];
    bool activated[PIN_END + 1];
    for (int pin = PIN_START; pin <= PIN_END; pin++) {
        prevLvl[pin] = gpio_get(pin);
        activated[pin] = PULL_ACTIVE ? !prevLvl[pin] : prevLvl[pin];
        debounceStart[pin] = 0;
        debouncePending[pin] = false;
    }

    fingerprint(activated);

    for (;;) {
        uint32_t now = time_us_32();

        for (int pin = PIN_START; pin <= PIN_END; pin++) {
            bool lvl = gpio_get(pin);
            if (lvl != prevLvl[pin]) {
                // A change was detected.
                if (!debouncePending[pin]) {
                    debouncePending[pin] = true;
                    debounceStart[pin] = now;
                } else if (now - debounceStart[pin] >= DEBOUNCE_US) {
                    // Stable new level after debounce window -> report it.
                    prevLvl[pin] = lvl;
                    debouncePending[pin] = false;
                    bool active = PULL_ACTIVE ? !lvl : lvl;  // activation = LOW w/ pull-up
                    activated[pin] = active;
                    printf("GPIO%d -> %s  [%s]\n",
                           pin,
                           lvl ? "HIGH" : "LOW",
                           active ? "ACTIVATED" : "released");
                    fflush(stdout);
                }
            } else if (debouncePending[pin]) {
                // Level flipped back before debounce window -> discard.
                debouncePending[pin] = false;
            }
        }

        sleep_us(SCAN_US);
    }
}