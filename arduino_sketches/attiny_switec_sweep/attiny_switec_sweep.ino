/*
 * ATtiny1614 SwitecX25 Sweep Demo
 *
 * Continuously sweeps the motor back and forth between the two scale
 * ends. No I2C, no DCS-BIOS — just a standalone movement demo to verify
 * the coil wiring and mechanism.
 *
 * Uses SwitecX25 library: https://github.com/clearwater/SwitecX25
 *
 * ATtiny1614 wiring (megaTinyCore):
 *   X27 coils: coil1=PA7, coil2=PA6, coil3=PA4, coil4=PA5
 *   Backlight (red PWM LED): PB1
 *   Motor supply per driver board; common GND with the ATtiny.
 *
 * Swapping any two coil pins reverses motor direction.
 */

#include <SwitecX25.h>

#define STEPS (315*3)  // 945 steps, 315° at 1/3 resolution
#define BACKLIGHT_PIN PIN_PB1
#define BACKLIGHT_BRIGHT 255
#define BACKLIGHT_PERIOD_MS 10000UL

SwitecX25 motor1(STEPS, PIN_PA7, PIN_PA6, PIN_PA4, PIN_PA5);

static bool forward = true;
static bool backlightOn = true;
static unsigned long lastToggle = 0;

void setup() {
    pinMode(BACKLIGHT_PIN, OUTPUT);
    analogWrite(BACKLIGHT_PIN, BACKLIGHT_BRIGHT);
    motor1.zero();               // sweep to lower stop and reset counter
    motor1.setPosition(STEPS);   // start moving toward the upper stop
    lastToggle = millis();
}

void loop() {
    if (millis() - lastToggle >= BACKLIGHT_PERIOD_MS) {
        backlightOn = !backlightOn;
        analogWrite(BACKLIGHT_PIN, backlightOn ? BACKLIGHT_BRIGHT : 0);
        lastToggle = millis();
    }

    if (motor1.stopped) {
        // reached a stop — reverse direction
        forward = !forward;
        motor1.setPosition(forward ? STEPS : 0);
    }
    motor1.update();
}
