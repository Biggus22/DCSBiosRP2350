#ifndef __DCSBIOS_USB_IDENTITY_H
#define __DCSBIOS_USB_IDENTITY_H

// Per-panel USB identity for panels that expose the composite CDC + HID
// gamepad device (usb_descriptors.c). Each panel sets its own host-visible
// name with one line, normally near the top of the firmware .cpp:
//
//     DCSBIOS_USB_ID("DCSBiosRP2350", "F-4E ECS Panel")
//
// A panel-side definition always wins over the weak defaults; panels that
// omit it enumerate as "DCS-BIOS Panel". The unique flash ID is appended
// by the descriptor code as the serial number, keeping instances of the
// same panel type apart. Give each panel type a distinct product name (and
// ideally a distinct USBD_PID compile definition) so Windows treats them
// as separate devices.

#define DCSBIOS_USB_ID(manufacturer, product)                \
    extern "C" const char *usbd_manufacturer_name = (manufacturer); \
    extern "C" const char *usbd_product_name = (product);

// Optional: name of the HID interface itself (rarely user-visible).
#define DCSBIOS_USB_HID_NAME(name)                     \
    extern "C" const char *usbd_hid_interface_name = (name);

#endif // __DCSBIOS_USB_IDENTITY_H
