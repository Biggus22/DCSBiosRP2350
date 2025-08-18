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
        while (true) {
            watchdog_update();
            static char usbInputBuffer[128];
            static uint8_t usbInputPos = 0;
            static char rs485InputBuffer[128];
            static uint8_t rs485InputPos = 0;
    
            int ch = getchar_timeout_us(0);
            if (ch != PICO_ERROR_TIMEOUT) {
                char c = (char)ch;
                printf("%c", c);
    
                if (c == '\r' || c == '\n') {
                    usbInputBuffer[usbInputPos] = '\0';
    
                    if (usbInputPos > 0) {
                        printf("\nSending over RS485: %s\n", usbInputBuffer);
                        rs485_send_string(usbInputBuffer);
                        rs485_send_char('\n');
                        rs485_flush();
                    }
    
                    usbInputPos = 0;
                }
                else if (usbInputPos < sizeof(usbInputBuffer) - 1) {
                    usbInputBuffer[usbInputPos++] = c;
                }
            }
    
            if (rs485_receive_available()) {
                char r = rs485_receive_char();
    
                if (r == '\n' || rs485InputPos >= sizeof(rs485InputBuffer) - 1) {
                    rs485InputBuffer[rs485InputPos] = '\0';
    
                    if (rs485InputPos > 0) {
                        printf("\nReceived from RS485: %s\n", rs485InputBuffer);
                    }
    
                    rs485InputPos = 0;
                }
                else {
                    rs485InputBuffer[rs485InputPos++] = r;
                }
            }
        }
    }


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
                rs485_send_char((char)ch);
            }
    
            // --- RS-485 Receive (full message handling) ---
            if (rs485_receive_available()) {
                char c = rs485_receive_char();
    
                if (!receivingSlaveMessage) {
                    if ((uint8_t)c <= 0x0F) {
                        receivingSlaveMessage = true;
                        slaveBufferPos = 0;
                    } else {
                        multicore_fifo_push_timeout_us((uint32_t)c, FIFO_TIMEOUT);
                    }
                } else {
                    if (c == '\n' || slaveBufferPos >= (sizeof(slaveBuffer) - 2)) {
                        slaveBuffer[slaveBufferPos] = '\n';
                        slaveBuffer[slaveBufferPos + 1] = '\0';
                        for (uint8_t i = 0; i < slaveBufferPos; ++i) {
                            multicore_fifo_push_timeout_us((uint32_t)slaveBuffer[i], FIFO_TIMEOUT);
                        }
                        multicore_fifo_push_timeout_us((uint32_t)'\n', FIFO_TIMEOUT);
                        printf("%s", slaveBuffer);
                        receivingSlaveMessage = false;
                        slaveBufferPos = 0;
                    } else {
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
    
            if (rs485_receive_available()) {
                char c = rs485_receive_char();
                multicore_fifo_push_timeout_us((uint32_t)c, FIFO_TIMEOUT);
            }
    
            if (multicore_fifo_rvalid()) {
                if (sendingNewMessage) {
                    rs485_send_char((char)currentBoardMode.address);
                    outgoingBuffer[outgoingIndex++] = (char)currentBoardMode.address;
                    sendingNewMessage = false;
                }
                char c = (char)multicore_fifo_pop_blocking();
                rs485_send_char(c);
                outgoingBuffer[outgoingIndex++] = c;
                if (c == '\n') {
                    outgoingBuffer[outgoingIndex] = '\0';
                    printf("[RS485 SEND]: %s", outgoingBuffer);
                    outgoingIndex = 0;
                    sendingNewMessage = true;
                }
                if (outgoingIndex >= (sizeof(outgoingBuffer)-2)) {
                    outgoingIndex = 0;
                    sendingNewMessage = true;
                }
            }
        }
    }


    void core1_tasks_main() {
        multicore_fifo_push_blocking(1);
        watchdog_enable(1000, 1);
        
        currentBoardMode = determineBoardMode(0xFF);
        
        switch (currentBoardMode.mode) {
            case BoardModeType::HOST:
                core1_host_task();
                break;
            case BoardModeType::SLAVE:
                core1_slave_task();
                break;
            case BoardModeType::USB_ONLY:
                core1_usb_only_task();
                break;
            case BoardModeType::RS485_TERMINAL:
                core1_rs485_terminal_task();
                break;
            case BoardModeType::INVALID:
            default:
                while(true) {
                    watchdog_update();
                }
        }
    }
}