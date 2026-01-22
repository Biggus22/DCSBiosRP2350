#ifndef __DCSBIOS_RS485_H
#define __DCSBIOS_RS485_H

#include <stddef.h>

#include "hardware/uart.h"
#include "hardware/gpio.h"

namespace DcsBios {
    void init_rs485_uart(uart_inst_t* uart, uint txPin, uint rxPin, uint enPin, uint32_t baudrate);
    void rs485_tx_enable();
    void rs485_tx_disable();
    void rs485_send_char(char c);
    void rs485_send_string(const char* str);
    void rs485_send_bytes(const uint8_t* data, size_t len, bool flush);
    bool rs485_receive_available();
    char rs485_receive_char();
    void rs485_flush();
}

#endif
