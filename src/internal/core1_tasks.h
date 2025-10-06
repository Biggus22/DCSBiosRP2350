#ifndef __DCSBIOS_CORE1_TASKS_H
#define __DCSBIOS_CORE1_TASKS_H

namespace DcsBios {
    void core1_host_task();
    void core1_slave_task();
    void core1_usb_only_task();
    void core1_rs485_terminal_task();
}

#endif // __DCSBIOS_CORE1_TASKS_H
