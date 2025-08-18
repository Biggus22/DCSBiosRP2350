#ifndef __DCSBIOS_CORE1_TASKS_H
#define __DCSBIOS_CORE1_TASKS_H

// Declare RS485 configuration variables as extern
/*extern uart_inst_t* rs485_uart;
extern uint rs485_tx_pin;
extern uint rs485_rx_pin;
extern uint rs485_en_pin;
extern uint32_t rs485_baudrate;
*/
namespace DcsBios {
    void core1_task();
    void core1_host_task();
    void core1_slave_task();
    void core1_host_arduino_task();
    void core1_slave_arduino_task();
    void core1_usb_only_task();
    void core1_rs485_terminal_task();
}

#endif //__DCSBIOS_CORE1_TASKS_H