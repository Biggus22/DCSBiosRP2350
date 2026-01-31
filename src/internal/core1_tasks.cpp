#include "core1_tasks.h"
#include "rs485.h"
#include "rs485_arduino.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "BoardMode.h"
#include <stdio.h>
#include <cstring>



namespace DcsBios {

    #define FIFO_TIMEOUT 100

#ifdef DCSBIOS_RS485_ARDUINO
    // ==========================================================================
    // Arduino RS485 Frame Transmission Helper
    // ==========================================================================
    //
    // Constructs and transmits an Arduino DCS-BIOS RS485 frame with proper timing.
    //
    // Frame wire format: [address][msgType][length][data...][checksum]
    //
    // PARAMETERS:
    //   address:  Slave address to poll (1-126) or 0 for broadcast
    //   msgType:  Message type (currently only 0 is used)
    //   data:     Optional payload (DCS-BIOS data from PC)
    //   length:   Number of payload bytes
    //
    // TIMING:
    //   - Enforces inter-frame gap before transmission (600µs by default)
    //   - This quiet period allows Arduino slaves to reset their frame receivers
    //   - Without this gap, slaves may experience byte misalignment errors
    //
    // CHECKSUM:
    //   - Currently hardcoded to 0 (reserved for future use)
    //   - Arduino library expects checksum byte but doesn't validate it
    //
    static inline void rs485_send_frame(uint8_t address, uint8_t msgType, const uint8_t* data, uint8_t length) {
        // Wait for inter-frame gap (bus quiet time)
        // This ensures the Arduino slave's frame receiver has reset to initial state
        sleep_us(DCSBIOS_RS485_ARDUINO_INTERFRAME_GAP_US);

        // Transmit frame header: [address][msgType][length]
        uint8_t header[3] = { address, msgType, length };
        rs485_send_bytes(header, sizeof(header), false);

        // Transmit payload data (if any)
        if (length > 0 && data) {
            rs485_send_bytes(data, length, false);
        }

        // Transmit checksum (currently always 0)
        uint8_t checksum = 0;
        rs485_send_bytes(&checksum, 1, true); // true = flush UART TX FIFO
    }
#endif

    void core1_host_task() {
#ifdef DCSBIOS_RS485_ARDUINO
        using namespace DcsBios::ArduinoRs485;

        // Active-only polling state (Arduino RS485 compatibility mode)
        static ActiveSlaveSet activeSlaves;
        static bool configSet = false;
        if (!configSet) {
            PollConfig cfg;
            cfg.activeOnly = (DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY != 0);
            cfg.maxMisses = DCSBIOS_RS485_ARDUINO_MAX_MISSES;
            activeSlaves.setConfig(cfg);
            configSet = true;
        }

        static FrameReceiver frameRx;
        static Frame frame{};

        static uint8_t txBuffer[DCSBIOS_RS485_ARDUINO_FRAME_MAX];
        static uint8_t txLen = 0;

        static absolute_time_t lastUsbRx = get_absolute_time();
        static absolute_time_t lastPoll = get_absolute_time();
        static absolute_time_t responseStart = get_absolute_time();
        static bool waitingResponse = false;
        static uint8_t waitingAddress = 0;

        while (true) {
            watchdog_update();

            // --- USB CDC Receive -> DCS-BIOS Parser and RS-485 Broadcast (Arduino framing) ---
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                uint8_t byte = static_cast<uint8_t>(ch);
                multicore_fifo_push_timeout_us((uint32_t)byte, FIFO_TIMEOUT);
                lastUsbRx = get_absolute_time();

                if (txLen < DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                    txBuffer[txLen++] = byte;
                }

                if (txLen >= DCSBIOS_RS485_ARDUINO_FRAME_MAX) {
                    rs485_send_frame(0, 0, txBuffer, txLen);
                    txLen = 0;
                }
            }

            if (txLen > 0) {
                int64_t idleUs = absolute_time_diff_us(lastUsbRx, get_absolute_time());
                if (idleUs > 200) {
                    rs485_send_frame(0, 0, txBuffer, txLen);
                    txLen = 0;
                }
            }

            // --- RS-485 Receive (Arduino frame handling) ---
            while (DcsBios::rs485_receive_available()) {
                uint8_t byte = static_cast<uint8_t>(DcsBios::rs485_receive_char());
                if (frameRx.pushByte(byte, frame)) {
                    if (waitingResponse && frame.address == waitingAddress && frame.msgType == 0) {
                        activeSlaves.recordResponse(frame.address);
                        waitingResponse = false;

                        for (uint8_t i = 0; i < frame.length; ++i) {
                            putchar((char)frame.data[i]);
                        }
                    }
                }
            }

            // --- Poll loop (active-only when enabled; fallback to full scan) ---
            if (waitingResponse) {
                int64_t elapsedUs = absolute_time_diff_us(responseStart, get_absolute_time());
                if (elapsedUs > DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US) {
                    activeSlaves.recordTimeout(waitingAddress);
                    waitingResponse = false;
                }
            } else {
                int64_t sincePollUs = absolute_time_diff_us(lastPoll, get_absolute_time());
                if (sincePollUs > DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US) {
                    uint8_t addr = activeSlaves.nextAddress();
                    rs485_send_frame(addr, 0, nullptr, 0);
                    waitingResponse = true;
                    waitingAddress = addr;
                    lastPoll = get_absolute_time();
                    responseStart = lastPoll;
                }
            }

            // --- Core0 Output -> USB CDC ---
            if (multicore_fifo_rvalid()) {
                putchar((char)multicore_fifo_pop_blocking());
                sleep_us(10);
            }
        }
#else
        static char slaveBuffer[128];
        static uint8_t slaveBufferPos = 0;
        static bool receivingSlaveMessage = false;
    
        while (true) {
            watchdog_update();
    
            // --- USB CDC Receive -> RS-485 Broadcast ---
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                multicore_fifo_push_timeout_us((uint32_t)ch, FIFO_TIMEOUT);
                DcsBios::rs485_send_char((char)ch);
            }
    
            // --- RS-485 Receive (full message handling) ---
            if (DcsBios::rs485_receive_available()) {
                char c = DcsBios::rs485_receive_char();
    
                if (!receivingSlaveMessage) {
                    if ((uint8_t)c <= 0x0F) {
                        // Slave board address detected (0x00 - 0x0F)
                        receivingSlaveMessage = true;
                        slaveBufferPos = 0; // Reset buffer
                    } else {
                        // Normal DCS-BIOS broadcast, forward immediately
                        multicore_fifo_push_timeout_us((uint32_t)c, FIFO_TIMEOUT);
                    }
                } else {
                    if (c == '\n' || slaveBufferPos >= (sizeof(slaveBuffer) - 2)) {
                        // Message complete
                        slaveBuffer[slaveBufferPos] = '\n';     // Force newline
                        slaveBuffer[slaveBufferPos + 1] = '\0'; // Null-terminate for safety
    
                        // Forward the entire event string (without board address)
                        for (uint8_t i = 0; i < slaveBufferPos; ++i) {
                            multicore_fifo_push_timeout_us((uint32_t)slaveBuffer[i], FIFO_TIMEOUT);
                        }
                        multicore_fifo_push_timeout_us((uint32_t)'\n', FIFO_TIMEOUT);
    
                        // Debug Print
                        printf("%s", slaveBuffer);
    
                        // Reset state
                        receivingSlaveMessage = false;
                        slaveBufferPos = 0;
                    } else {
                        // Store received character into buffer
                        slaveBuffer[slaveBufferPos++] = c;
                    }
                }
            }
    
            // --- Core0 Output -> USB CDC ---
            if (multicore_fifo_rvalid()) {
                putchar((char)multicore_fifo_pop_blocking());
                sleep_us(10);
            }
        }
#endif
    }
    
    

    void core1_slave_task() {
        static bool sendingNewMessage = true;
        static char outgoingBuffer[128];
        static int outgoingIndex = 0;
    
        while (true) {
            watchdog_update();
    
            // --- RS-485 Receive from Host ---
            if (DcsBios::rs485_receive_available()) {
                char c = DcsBios::rs485_receive_char();
                multicore_fifo_push_timeout_us((uint32_t)c, FIFO_TIMEOUT);
            }
    
            // --- Core0 Event -> RS-485 Output ---
            if (multicore_fifo_rvalid()) {
                if (sendingNewMessage) {
                    // Start of a new outgoing message: send board address first
                    DcsBios::rs485_send_char((char)DcsBios::currentBoardMode.address);
                    outgoingBuffer[outgoingIndex++] = (char)DcsBios::currentBoardMode.address;
                    sendingNewMessage = false;
                }
    
                char c = (char)multicore_fifo_pop_blocking();
                DcsBios::rs485_send_char(c);
                outgoingBuffer[outgoingIndex++] = c;
    
                if (c == '\n') {
                    // End of message
                    outgoingBuffer[outgoingIndex] = '\0'; // Null-terminate
                    printf("[RS485 SEND]: %s", outgoingBuffer); // Debug printf
    
                    outgoingIndex = 0;
                    sendingNewMessage = true;
                }
    
                // Buffer overflow protection
                if (outgoingIndex >= sizeof(outgoingBuffer) - 1) {
                    outgoingBuffer[outgoingIndex] = '\0'; // Null-terminate
                    printf("[RS485 ERROR]: Buffer overflow: %s\n", outgoingBuffer);
                    outgoingIndex = 0;
                    sendingNewMessage = true;
                }
            }
        }
    }
    

    void core1_usb_only_task() {
        while (true) {
            watchdog_update();

            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                multicore_fifo_push_timeout_us((uint32_t)ch, FIFO_TIMEOUT);
            }

            if (multicore_fifo_rvalid()) {
                putchar((char)multicore_fifo_pop_blocking());
                sleep_us(10);
            }
        }
    }

    
    void core1_rs485_terminal_task() {
        static char usbInputBuffer[128];
        static uint8_t usbInputPos = 0;
    
        static char rs485InputBuffer[128];
        static uint8_t rs485InputPos = 0;
    
        printf("RS485 Terminal Mode Started\n");
        printf("Type text and press Enter to send over RS485.\n");
    
        while (true) {
            watchdog_update();
    
            // === 1. Read from USB COM and send over RS-485 ===
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                char c = (char)ch;
                printf("%c", c);  // Local echo
    
                if (c == '\r' || c == '\n') {
                    usbInputBuffer[usbInputPos] = '\0';
    
                    if (usbInputPos > 0) {
                        printf("\nSending over RS485: %s\n", usbInputBuffer);
    
                        rs485_send_string(usbInputBuffer);
                        rs485_send_char('\n'); // Mark end of message
                        rs485_flush();
                    }
    
                    usbInputPos = 0;  // Reset after sending
                }
                else if (usbInputPos < sizeof(usbInputBuffer) - 1) {
                    usbInputBuffer[usbInputPos++] = c;
                }
            }
    
            // === 2. Read from RS-485 and print to USB ===
            if (rs485_receive_available()) {
                char r = rs485_receive_char();
    
                if (r == '\n' || rs485InputPos >= sizeof(rs485InputBuffer) - 1) {
                    rs485InputBuffer[rs485InputPos] = '\0';
    
                    if (rs485InputPos > 0) {
                        printf("\nReceived from RS485: %s\n", rs485InputBuffer);
                    }
    
                    rs485InputPos = 0;  // Reset after complete message
                }
                else {
                    rs485InputBuffer[rs485InputPos++] = r;
                }
            }
    
            sleep_us(10);
        }
    }
    
    

}
