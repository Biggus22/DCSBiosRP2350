#ifndef RP2040_BOARD
#define RP2040_BOARD
#endif
#include "pico/time.h"
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "internal/ws2812.h" // Include the WS2812 header

#define afcsCoilPin 10       // Output to hold coil - AP Hold Coil
#define altHoldCoilPin 7     // Output to hold coil - AFCS Coil
#define FULL_POWER 255       // Full 12V power (PWM 255)
#define HOLD_POWER_AFCS 50   // Reduced power for AP Hold
#define HOLD_POWER_ALTHLD 60 // Reduced power for AFCS
#define HOLD_TIME 3000       // Time in milliseconds to hold full power (3 sec)
#define NUM_LEDS 18          // Total number of SK6812 LEDs

// Test input pins (using same pins as your switches for testing)
#define TEST_AFCS_PIN 12     // Same as pltAfcsAutopilotPin
#define TEST_ALTHOLD_PIN 9   // Same as pltAfcsAltHoldPin

WS2812 externalLeds(pio0, 0, 14, false); // Global WS2812 object for external NeoPixels on pin 14

// PWM slice and channel variables
uint slice_num_afcs, slice_num_althold;
uint channel_afcs, channel_althold;

// State tracking
bool afcs_active = false;
bool afcs_in_full_power = false;  // Track whether AFCS coil is in full power mode
bool althold_active = false;
bool test_afcs_last_state = false;
bool test_althold_last_state = false;

// Additional state tracking for AFCS power timing
absolute_time_t afcs_full_power_end_time;

void setup_pwm(uint gpio_pin)
{
    // Set the GPIO function to PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    // Get PWM slice and channel for this GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint channel = pwm_gpio_to_channel(gpio_pin);

    // Store slice and channel info for each pin
    if (gpio_pin == afcsCoilPin)
    {
        slice_num_afcs = slice_num;
        channel_afcs = channel;
    }
    else if (gpio_pin == altHoldCoilPin)
    {
        slice_num_althold = slice_num;
        channel_althold = channel;
    }

    // Set PWM frequency (optional - adjust as needed)
    pwm_set_clkdiv(slice_num, 4.0f);
    pwm_set_wrap(slice_num, 255); // 8-bit resolution (0-255)

    // Enable PWM
    pwm_set_enabled(slice_num, true);
}

void analog_write(uint gpio_pin, uint16_t value)
{
    // Set PWM duty cycle (0-255) for the correct pin
    if (gpio_pin == afcsCoilPin)
    {
        pwm_set_chan_level(slice_num_afcs, channel_afcs, value);
    }
    else if (gpio_pin == altHoldCoilPin)
    {
        pwm_set_chan_level(slice_num_althold, channel_althold, value);
    }
}

void activate_afcs_coil()
{
    printf("Activating AFCS Coil - Full power for %dms, then hold power\n", HOLD_TIME);
    analog_write(afcsCoilPin, FULL_POWER);
    afcs_in_full_power = true;  // Set flag when in full power mode
    afcs_active = true;
    afcs_full_power_end_time = make_timeout_time_ms(HOLD_TIME);  // Set end time for full power
    printf("AFCS Coil now in full power mode (PWM: %d)\n", FULL_POWER);
}

void deactivate_afcs_coil()
{
    printf("Deactivating AFCS Coil\n");
    analog_write(afcsCoilPin, 0);
    afcs_in_full_power = false;  // Reset flag when deactivating
    afcs_active = false;
}

void activate_althold_coil()
{
    printf("Activating Alt Hold Coil - Full power for %dms, then hold power\n", HOLD_TIME);
    analog_write(altHoldCoilPin, FULL_POWER);
    sleep_ms(HOLD_TIME);
    analog_write(altHoldCoilPin, HOLD_POWER_ALTHLD);
    althold_active = true;
    printf("Alt Hold Coil now in hold mode (PWM: %d)\n", HOLD_POWER_ALTHLD);
}

void deactivate_althold_coil()
{
    printf("Deactivating Alt Hold Coil\n");
    analog_write(altHoldCoilPin, 0);
    althold_active = false;
}

void update_led_status()
{
    // Clear all LEDs first
    externalLeds.clear();
    
    // Set LEDs based on coil status
    if (afcs_active || althold_active)
    {
        externalLeds.setBrightness(128); // Medium brightness
        
        for (int i = 0; i < NUM_LEDS; i++)
        {
            if (afcs_active && althold_active)
            {
                // Both active - yellow
                externalLeds.setPixel(i, externalLeds.rgbw(255, 255, 0, 0));
            }
            else if (afcs_active)
            {
                // AFCS only - red
                externalLeds.setPixel(i, externalLeds.rgbw(255, 0, 0, 0));
            }
            else if (althold_active)
            {
                // Alt Hold only - blue
                externalLeds.setPixel(i, externalLeds.rgbw(0, 0, 255, 0));
            }
        }
    }
    else
    {
        // Both inactive - dim green
        externalLeds.setBrightness(32);
        for (int i = 0; i < NUM_LEDS; i++)
        {
            externalLeds.setPixel(i, externalLeds.rgbw(0, 64, 0, 0));
        }
    }
    
    externalLeds.show();
}

void print_status()
{
    printf("\n=== PWM Test Status ===\n");
    printf("AFCS Coil (Pin %d): %s", afcsCoilPin, afcs_active ? "ACTIVE" : "INACTIVE");
    if (afcs_active) {
        printf(" [%s]", afcs_in_full_power ? "FULL POWER" : "HOLD POWER");
    }
    printf("\n");
    printf("Alt Hold Coil (Pin %d): %s\n", altHoldCoilPin, althold_active ? "ACTIVE" : "INACTIVE");
    printf("Test Switches - AFCS: %s, Alt Hold: %s\n", 
           gpio_get(TEST_AFCS_PIN) ? "HIGH" : "LOW",
           gpio_get(TEST_ALTHOLD_PIN) ? "HIGH" : "LOW");
    printf("========================\n\n");
}

void print_help()
{
    printf("\n=== AFCS/Alt Hold PWM Test Program ===\n");
    printf("This program tests your AFCS and Alt Hold coil controls\n");
    printf("\nTest Methods:\n");
    printf("1. Toggle switches connected to pins %d (AFCS) and %d (Alt Hold)\n", TEST_AFCS_PIN, TEST_ALTHOLD_PIN);
    printf("2. Serial commands (type and press Enter):\n");
    printf("   'a' or 'A' - Toggle AFCS coil\n");
    printf("   'h' or 'H' - Toggle Alt Hold coil\n");
    printf("   's' or 'S' - Show status\n");
    printf("   'r' or 'R' - Reset (deactivate both coils)\n");
    printf("   '?' or 'help' - Show this help\n");
    printf("\nOperation:\n");
    printf("   Switch HIGH = Coil ON at full power for %dms, then reduced hold power\n", HOLD_TIME);
    printf("   Switch LOW = Coil OFF (0 PWM)\n");
    printf("=====================================\n\n");
}

int main()
{
    stdio_init_all(); // Initialize USB CDC
    sleep_ms(2000);   // Wait for USB CDC to be ready

    printf("\n=== AFCS/Alt Hold PWM Test Program Starting ===\n");
    
    // Setup coil pins
    setup_pwm(afcsCoilPin);
    setup_pwm(altHoldCoilPin);
    printf("PWM outputs initialized on pins %d and %d\n", afcsCoilPin, altHoldCoilPin);

    // Setup NeoPixel strip
        externalLeds.begin(NUM_LEDS);
           externalLeds.clear();
               externalLeds.show();
                   printf("NeoPixel strip initialized on pin 14\\n\");

    // Setup test input pins with pullups
    gpio_init(TEST_AFCS_PIN);
    gpio_set_dir(TEST_AFCS_PIN, GPIO_IN);
    gpio_pull_up(TEST_AFCS_PIN);
    
    gpio_init(TEST_ALTHOLD_PIN);
    gpio_set_dir(TEST_ALTHOLD_PIN, GPIO_IN);
    gpio_pull_up(TEST_ALTHOLD_PIN);
    
    printf("Test input pins %d and %d initialized with pullups\n", TEST_AFCS_PIN, TEST_ALTHOLD_PIN);

    print_help();
    print_status();

    printf("Test program ready! Use switches or serial commands...\n\n");

    while (true)
    {
        // Check for serial input
        int c = getchar_timeout_us(1000); // Non-blocking read with 1ms timeout
        if (c != PICO_ERROR_TIMEOUT)
        {
            switch (c)
            {
                case 'a':
                case 'A':
                    if (afcs_active) {
                        deactivate_afcs_coil();
                    } else {
                        activate_afcs_coil();
                    }
                    update_led_status();
                    break;
                    
                case 'h':
                case 'H':
                    if (althold_active) {
                        deactivate_althold_coil();
                    } else {
                        activate_althold_coil();
                    }
                    update_led_status();
                    break;
                    
                case 's':
                case 'S':
                    print_status();
                    break;
                    
                case 'r':
                case 'R':
                    printf("Resetting all coils...\n");
                    deactivate_afcs_coil();
                    deactivate_althold_coil();
                    update_led_status();
                    break;
                    
                case '?':
                    print_help();
                    break;
                    
                default:
                    if (c >= 32 && c < 127) { // Printable characters
                        printf("Unknown command: '%c'. Type '?' for help.\n", c);
                    }
                    break;
            }
        }

        // Check hardware toggle switches (inverted logic due to pullups)
        bool afcs_pin_state = !gpio_get(TEST_AFCS_PIN);  // Invert because of pullup
        bool althold_pin_state = !gpio_get(TEST_ALTHOLD_PIN);  // Invert because of pullup
        
        // Update coil states based on actual switch positions
        // If switch is in ON position and coil is not active, activate it
        if (afcs_pin_state && !afcs_active)
        {
            printf("Hardware: AFCS switch ON - Activating coil\n");
            activate_afcs_coil();
            update_led_status();
        }
        // If switch is in OFF position and coil is active, deactivate it
        else if (!afcs_pin_state && afcs_active)
        {
            printf("Hardware: AFCS switch OFF - Deactivating coil\n");
            deactivate_afcs_coil();
            update_led_status();
        }
        
        // Same logic for Alt Hold
        if (althold_pin_state && !althold_active)
        {
            printf("Hardware: Alt Hold switch ON - Activating coil\n");
            activate_althold_coil();
            update_led_status();
        }
        else if (!althold_pin_state && althold_active)
        {
            printf("Hardware: Alt Hold switch OFF - Deactivating coil\n");
            deactivate_althold_coil();
            update_led_status();
        }
        
        // Store previous states for reference (not used for edge detection anymore)
        test_afcs_last_state = afcs_pin_state;
        test_althold_last_state = althold_pin_state;
        
        // Check if it's time to switch AFCS from full power to hold power
        if (afcs_active && afcs_in_full_power && absolute_time_diff_us(afcs_full_power_end_time, get_absolute_time()) <= 0)
        {
            // Time to switch to hold power
            analog_write(afcsCoilPin, HOLD_POWER_AFCS);
            afcs_in_full_power = false;
            printf("AFCS Coil now in hold mode (PWM: %d)\n", HOLD_POWER_AFCS);
            update_led_status();
        }
        
        sleep_ms(10); // Small delay to prevent excessive polling
    }
}