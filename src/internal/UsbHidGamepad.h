#ifndef __DCSBIOS_USB_HID_GAMEPAD_H
#define __DCSBIOS_USB_HID_GAMEPAD_H

#include <stdint.h>

// Composite USB HID gamepad for panels whose controls do not exist in the
// DCS-BIOS protocol. The panel polls its own inputs and feeds this class;
// reports are sent to the host on change only. The CDC interface used by
// DCS-BIOS is unaffected, and the class produces no traffic unless a panel
// calls it.
//
// Requires CFG_TUD_HID=1 and the composite descriptors in usb_descriptors.c
// (panel opt-in DCSBIOS_USB_HID_GAMEPAD, see CMakeLists.txt).

class UsbHidGamepad {
public:
    // Call once from main() after stdio_init_all().
    static void begin();

    // value: -32768..32767 absolute axis position (signed 16-bit, the
    // Windows gamepad convention).
    static void setAxis(int16_t value);

    // index: 0 or 1 (HID buttons 1 and 2).
    static void setButton(uint8_t index, bool pressed);

    // Attempt to transmit a pending report; call once per main-loop pass.
    static void task();
};

#endif // __DCSBIOS_USB_HID_GAMEPAD_H
