#include "core1_tasks.h"
#include "rs485.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "BoardMode.h"
#include <stdio.h>
#include <cstring>



namespace DcsBios {

    #define FIFO_TIMEOUT 100

    void core1_host_task() {
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
