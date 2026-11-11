// Composite USB descriptors for panels that expose a HID gamepad interface
// alongside the DCS-BIOS CDC serial port.
//
// Based on the Pico SDK's pico_stdio_usb/stdio_usb_descriptors.c, with a HID
// gamepad interface appended. Compiled only for panels that opt in via
// DCSBIOS_USB_HID_GAMEPAD (see CMakeLists.txt); for every other panel this
// file is empty and the SDK's default CDC-only descriptors are used.
//
// Interface layout:
//   0/1  CDC-ACM  (DCS-BIOS protocol, unchanged endpoints 0x81/0x02/0x82)
//   2    vendor   (picotool soft-reset, as in the SDK default descriptors)
//   3    HID gamepad (X axis + 2 buttons, endpoint 0x83)

#include "pico/stdio_usb.h"
#include "pico/stdio_usb/reset_interface.h"
#include "pico/unique_id.h"
#include "tusb.h"

#if defined(DCSBIOS_USB_HID_GAMEPAD) && !PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS

#ifndef USBD_VID
#define USBD_VID (0x2E8A) // Raspberry Pi
#endif

#ifndef USBD_PID
#define USBD_PID (0x7F01) // DCSBiosRP2350 composite CDC + HID
#endif

// Per-panel USB identity. A panel overrides these by defining strong
// globals, normally via DCSBIOS_USB_ID(manufacturer, product) from
// UsbIdentity.h; the weak defaults below apply when it does not.
// The strings are consumed during USB enumeration, which happens after C++
// static initialisation, so panel-side definitions are always set in time.
// Compile definitions (USBD_MANUFACTURER / USBD_PRODUCT) only change the
// weak defaults; a panel-side definition always wins.
#ifdef USBD_MANUFACTURER
#define USBD_MANUFACTURER_DEFAULT USBD_MANUFACTURER
#else
#define USBD_MANUFACTURER_DEFAULT "DCSBiosRP2350"
#endif
#ifdef USBD_PRODUCT
#define USBD_PRODUCT_DEFAULT USBD_PRODUCT
#else
#define USBD_PRODUCT_DEFAULT "DCS-BIOS Panel"
#endif

__attribute__((weak)) const char *usbd_manufacturer_name = USBD_MANUFACTURER_DEFAULT;
__attribute__((weak)) const char *usbd_product_name = USBD_PRODUCT_DEFAULT;

// Name of the HID interface (rarely shown to the user; visible in some USB
// inspection tools). Panels may override via DCSBIOS_USB_HID_NAME().
#ifdef USBD_HID_INTERFACE_NAME
#define USBD_HID_INTERFACE_NAME_DEFAULT USBD_HID_INTERFACE_NAME
#else
#define USBD_HID_INTERFACE_NAME_DEFAULT "DCS-BIOS Gamepad"
#endif
__attribute__((weak)) const char *usbd_hid_interface_name = USBD_HID_INTERFACE_NAME_DEFAULT;

#define TUD_RPI_RESET_DESC_LEN  9

#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_RPI_RESET_DESC_LEN + TUD_HID_DESC_LEN)
#if !PICO_STDIO_USB_DEVICE_SELF_POWERED
#define USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE (0)
#define USBD_MAX_POWER_MA (250)
#else
#define USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE TUSB_DESC_CONFIG_ATT_SELF_POWERED
#define USBD_MAX_POWER_MA (1)
#endif

#define USBD_ITF_CDC       (0) // needs 2 interfaces
#define USBD_ITF_RPI_RESET (2)
#define USBD_ITF_HID       (3)
#define USBD_ITF_MAX       (4)

#define USBD_CDC_EP_CMD (0x81)
#define USBD_CDC_EP_OUT (0x02)
#define USBD_CDC_EP_IN (0x82)
#define USBD_CDC_CMD_MAX_SIZE (8)
#define USBD_CDC_IN_OUT_MAX_SIZE (64)

#define USBD_HID_EP_IN (0x83)
#define USBD_HID_EP_SIZE (16)
#define USBD_HID_EP_INTERVAL (1)

#define USBD_STR_0 (0x00)
#define USBD_STR_MANUF (0x01)
#define USBD_STR_PRODUCT (0x02)
#define USBD_STR_SERIAL (0x03)
#define USBD_STR_CDC (0x04)
#define USBD_STR_RPI_RESET (0x05)
#define USBD_STR_HID (0x06)

// Note: descriptors returned from callbacks must exist long enough for transfer to complete

static const tusb_desc_device_t usbd_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
// On Windows, if bcdUSB = 0x210 then a Microsoft OS 2.0 descriptor is required, else the device won't be detected
// This is only needed for driverless access to the reset interface - the CDC interface doesn't require these descriptors
// for driverless access, but will still not work if bcdUSB = 0x210 and no descriptor is provided. Therefore always
// use bcdUSB = 0x200 if the Microsoft OS 2.0 descriptor isn't enabled
#if PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE && PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_MS_OS_20_DESCRIPTOR
    .bcdUSB = 0x0210,
#else
    .bcdUSB = 0x0200,
#endif
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = USBD_STR_MANUF,
    .iProduct = USBD_STR_PRODUCT,
    .iSerialNumber = USBD_STR_SERIAL,
    .bNumConfigurations = 1,
};

#define TUD_RPI_RESET_DESCRIPTOR(_itfnum, _stridx) \
  /* Interface */\
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_VENDOR_SPECIFIC, RESET_INTERFACE_SUBCLASS, RESET_INTERFACE_PROTOCOL, _stridx,

// HID gamepad report: absolute X axis + 2 buttons (bits 0 and 1) + 6 bits
// of padding. 3 bytes per report, no report ID.
//
// The axis is SIGNED 16-bit (-32768..32767), the gamepad convention. Windows
// sign-extends multi-byte item values, so an unsigned 0xFFFF logical max is
// parsed as -1 and the axis scales nonsensically.
static const uint8_t usbd_hid_report_desc[] = {
    HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
    HID_USAGE(HID_USAGE_DESKTOP_GAMEPAD),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
        /* X axis, absolute, signed 16-bit */
        HID_USAGE(HID_USAGE_DESKTOP_X),
        HID_LOGICAL_MIN_N(-32768, 2),
        HID_LOGICAL_MAX_N(32767, 2),
        HID_REPORT_SIZE(16),
        HID_REPORT_COUNT(1),
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
        /* 2 buttons */
        HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),
        HID_USAGE_MIN(1),
        HID_USAGE_MAX(2),
        HID_LOGICAL_MIN(0),
        HID_LOGICAL_MAX(1),
        HID_REPORT_SIZE(1),
        HID_REPORT_COUNT(2),
        HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),
        /* 6 bits of padding */
        HID_REPORT_SIZE(1),
        HID_REPORT_COUNT(6),
        HID_INPUT(HID_CONSTANT),
    HID_COLLECTION_END
};

static const uint8_t usbd_desc_cfg[USBD_DESC_LEN] = {
    TUD_CONFIG_DESCRIPTOR(1, USBD_ITF_MAX, USBD_STR_0, USBD_DESC_LEN,
        USBD_CONFIGURATION_DESCRIPTOR_ATTRIBUTE, USBD_MAX_POWER_MA),

    TUD_CDC_DESCRIPTOR(USBD_ITF_CDC, USBD_STR_CDC, USBD_CDC_EP_CMD,
        USBD_CDC_CMD_MAX_SIZE, USBD_CDC_EP_OUT, USBD_CDC_EP_IN, USBD_CDC_IN_OUT_MAX_SIZE),

#if PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
    TUD_RPI_RESET_DESCRIPTOR(USBD_ITF_RPI_RESET, USBD_STR_RPI_RESET)
#endif

    TUD_HID_DESCRIPTOR(USBD_ITF_HID, USBD_STR_HID, 0, sizeof(usbd_hid_report_desc),
        USBD_HID_EP_IN, USBD_HID_EP_SIZE, USBD_HID_EP_INTERVAL)
};

static char usbd_serial_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];

// Runtime lookup: the panel-overridable strings are variables, so a static
// initializer table is not possible in C.
static const char *usbd_desc_str(uint8_t index) {
    switch (index) {
        case USBD_STR_MANUF: return usbd_manufacturer_name;
        case USBD_STR_PRODUCT: return usbd_product_name;
        case USBD_STR_SERIAL: return usbd_serial_str;
        case USBD_STR_CDC: return "Board CDC";
#if PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
        case USBD_STR_RPI_RESET: return "Reset";
#endif
        case USBD_STR_HID: return usbd_hid_interface_name;
        default: return NULL;
    }
}

const uint8_t *tud_descriptor_device_cb(void) {
    return (const uint8_t *)&usbd_desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(__unused uint8_t index) {
    return usbd_desc_cfg;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, __unused uint16_t langid) {
#ifndef USBD_DESC_STR_MAX
#define USBD_DESC_STR_MAX (20)
#elif USBD_DESC_STR_MAX > 127
#error USBD_DESC_STR_MAX too high (max is 127).
#elif USBD_DESC_STR_MAX < 17
#error USBD_DESC_STR_MAX too low (min is 17).
#endif
    static uint16_t desc_str[USBD_DESC_STR_MAX];

    // Assign the SN using the unique flash id
    if (!usbd_serial_str[0]) {
        pico_get_unique_board_id_string(usbd_serial_str, sizeof(usbd_serial_str));
    }

    uint8_t len;
    if (index == 0) {
        desc_str[1] = 0x0409; // supported language is English
        len = 1;
    } else {
        const char *str = usbd_desc_str(index);
        if (str == NULL) {
            return NULL;
        }
        for (len = 0; len < USBD_DESC_STR_MAX - 1 && str[len]; ++len) {
            desc_str[1 + len] = str[len];
        }
    }

    // first byte is length (including header), second byte is string type
    desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * len + 2));

    return desc_str;
}

// HID class driver callback: the GET_DESCRIPTOR(Report) request is answered
// from here, the length is taken from the interface descriptor above.
uint8_t const *tud_hid_descriptor_report_cb(__unused uint8_t instance) {
    return usbd_hid_report_desc;
}

#endif
