#include "rs485.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

namespace DcsBios {

// Internal static storage
static uart_inst_t* _rs485_uart = nullptr;
static uint _rs485_en_pin = 0;

void init_rs485_uart(uart_inst_t* uart, uint txPin, uint rxPin, uint enPin, uint32_t baudrate) {
    _rs485_uart = uart;
    _rs485_en_pin = enPin;

    uart_init(_rs485_uart, baudrate);

    gpio_set_function(txPin, GPIO_FUNC_UART);
    gpio_set_function(rxPin, GPIO_FUNC_UART);

    gpio_init(_rs485_en_pin);
    gpio_set_dir(_rs485_en_pin, GPIO_OUT);
    gpio_put(_rs485_en_pin, 0); // Start in receive mode

    uart_set_format(_rs485_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(_rs485_uart, true);  // ✅ FIFO stays enabled
}

void rs485_tx_enable() {
    gpio_put(_rs485_en_pin, 1);
}

void rs485_tx_disable() {
    gpio_put(_rs485_en_pin, 0);
}

void rs485_send_char(char c) {
    rs485_tx_enable();
    uart_putc_raw(_rs485_uart, c);
    uart_tx_wait_blocking(_rs485_uart);
    rs485_tx_disable();
}

void rs485_send_string(const char* str) {
    rs485_tx_enable();
    while (*str) {
        while (!uart_is_writable(_rs485_uart));
        uart_putc_raw(_rs485_uart, *str++);
        sleep_us(100);
    }
    rs485_flush();
}

void rs485_flush() {
    uart_tx_wait_blocking(_rs485_uart);
    sleep_us(400);  // Let last bits fully transmit
    rs485_tx_disable();
}

bool rs485_receive_available() {
    return uart_is_readable(_rs485_uart);
}

char rs485_receive_char() {
    return uart_getc(_rs485_uart);
}

}