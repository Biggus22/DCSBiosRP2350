#ifndef __DCSBIOS_H
#define __DCSBIOS_H

#ifndef NULL
#define NULL 0
#endif

#include <stdint.h>


#include "internal/ExportStreamListener.h"
#include "internal/PollingInput.h"
#include "internal/Protocol.h"
#include "internal/Addresses.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "internal/rs485.h"
#include "internal/DeviceAddress.h"
#include "internal/BoardMode.h"
#include "internal/core1_tasks.h"
#include "internal/FoxConfig.h"
#include "pico/mutex.h"

namespace DcsBios {
	const unsigned char PIN_NC = 0xFF;
}

// Custom map function equivalent to Arduino's map()
static inline int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_min == in_max) return out_min; // Prevent division by zero
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
The following is an ugly hack to work with the Arduino IDE's build system.
The DCS-BIOS Arduino Library is configured with #defines such as DCSBIOS_RS485_MASTER or DCSBIOS_RS485_SLAVE <address>.
To make sure these defines are visible when compiling the code, we can't put it into a separate translation unit.

Normally, those #defines would go in a separate "config.h" or you would use compiler flags. But since Arduino libraries
do not come with their own build system, we are just putting everything into the header file.
*/


namespace DcsBios {
    extern BoardMode currentBoardMode;  
	extern void core1_task();
    ProtocolParser parser;

    void setup() {
        printf("DCS-BIOS setup started...\n");
    }

    void loop() {
        while (multicore_fifo_rvalid()) {
            char ch = (char)multicore_fifo_pop_blocking(); // Get character from Core 1
            parser.processChar(ch);
        }
        PollingInput::pollInputs();
        ExportStreamListener::loopAll();
    }

    static mutex_t g_fifo_mutex;
    static bool g_fifo_mutex_inited = false;

    static inline void fifo_lock() {
        if (!g_fifo_mutex_inited) {
            mutex_init(&g_fifo_mutex);
            g_fifo_mutex_inited = true;
        }
        mutex_enter_blocking(&g_fifo_mutex);
    }
    static inline void fifo_unlock() { mutex_exit(&g_fifo_mutex); }

    static inline void fifo_push_byte_blocking(uint8_t b) {
        // Multicore FIFO is 32-bit; use low 8 bits for ASCII byte
        multicore_fifo_push_blocking((uint32_t)b);
    }

    // Build "TAG VALUE\n" once, then push all bytes inside a single critical section.
    bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
        // 1) Format entire line
        char line[96];
        int n = snprintf(line, sizeof(line), "%s %s\n", (msg ? msg : ""), (arg ? arg : ""));
        if (n <= 0 || n >= (int)sizeof(line)) {
            return false; // formatting failed or truncated
        }

        // 2) Atomic section to prevent cross-core interleaving and mid-line truncation
        fifo_lock();
        for (int i = 0; i < n; ++i) {
            fifo_push_byte_blocking((uint8_t)line[i]);
        }
        fifo_unlock();

        DcsBios::PollingInput::setMessageSentOrQueued();
        return true;
    }

    void resetAllStates() {
        PollingInput::resetAllStates();
    }

    // Core 1: Handles USB tasks
    void core1_task() {
        if (watchdog_caused_reboot()) {
            sleep_ms(100); // Wait before USB resume
        }

        watchdog_enable(WATCHDOG_TIMEOUT, true);

        switch (DcsBios::currentBoardMode.mode) {
            case DcsBios::BoardModeType::HOST:
                DcsBios::core1_host_task(); // Defined in core1_tasks.cpp
                break;
    
            case DcsBios::BoardModeType::SLAVE:
                DcsBios::core1_slave_task();
                break;
    
            case DcsBios::BoardModeType::USB_ONLY:
                DcsBios::core1_usb_only_task();
                break;

            case DcsBios::BoardModeType::RS485_TERMINAL:
                DcsBios::core1_rs485_terminal_task();
                break;

            default:
                while (true) {
                    sleep_ms(500);
                }
                break;
        }
    }


    inline int mapInt(int x, int in_min, int in_max, int out_min, int out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
    
}


#include "internal/Buttons.h"
#include "internal/Switches.h"
#include "internal/SyncingSwitches.h" 
#include "internal/Encoders.h"
#include "internal/Potentiometers.h"
#include "internal/RotarySyncingPotentiometer.h"
#include "internal/Leds.h"
#include "internal/Display.h"
#ifndef DCSBIOS_DISABLE_SERVO
#include "internal/Servos.h"
#endif
#include "internal/Dimmer.h"
#include "internal/BcdWheels.h"
#include "internal/AnalogMultiPos.h"
#include "internal/RotarySwitch.h"
#if defined(USE_MATRIX_SWITCHES) || defined(DCSBIOS_USE_MATRIX_SWITCHES)
#include "internal/MatrixSwitches.h"
#endif
// #include "internal/DualModeButton.h"

namespace DcsBios {
	template<unsigned int first, unsigned int second>
	unsigned int piecewiseMap(unsigned int newValue) {
		return 0;
	}

	template<unsigned int from1, unsigned int to1, unsigned int from2, unsigned int to2, unsigned int... rest>
	unsigned int piecewiseMap(unsigned int newValue) {
		if (newValue < from2) {
			return map(newValue, from1, from2, to1, to2);
		} else {
			return piecewiseMap<from2, to2, rest...>(newValue);
		}
	}
}

#ifndef DCSBIOS_RS485_MASTER
namespace DcsBios {	
	inline bool sendDcsBiosMessage(const char* msg, const char* arg) {
		while(!tryToSendDcsBiosMessage(msg, arg));
		return true;
	}
}

// for backwards compatibility, can be removed when we have a proper place to document this interface:
inline bool sendDcsBiosMessage(const char* msg, const char* arg) {
	while(!DcsBios::tryToSendDcsBiosMessage(msg, arg));
	return true;
}
#endif

#endif // include guard
