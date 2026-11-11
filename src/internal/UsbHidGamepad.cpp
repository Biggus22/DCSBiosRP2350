#include "UsbHidGamepad.h"

#if CFG_TUD_HID

#include "class/hid/hid_device.h"
#include "string.h"

namespace {

// Report layout: 2-byte X axis (little-endian) + 1 byte button bits.
constexpr uint8_t kReportLen = 3;

uint8_t report[kReportLen] = { 0, 0, 0 };
bool pending = false;

}  // namespace

void UsbHidGamepad::begin() {
    pending = true;  // send the initial report once the host is ready
}

void UsbHidGamepad::setAxis(int16_t value) {
    if (report[0] == ((uint8_t)value) && report[1] == ((uint8_t)((uint16_t)value >> 8))) return;
    report[0] = (uint8_t)value;
    report[1] = (uint8_t)((uint16_t)value >> 8);
    pending = true;
}

void UsbHidGamepad::setButton(uint8_t index, bool pressed) {
    const uint8_t bit = (uint8_t)(1u << index);
    const uint8_t old = report[2];
    if (pressed) {
        report[2] |= bit;
    } else {
        report[2] &= (uint8_t)~bit;
    }
    if (report[2] != old) pending = true;
}

void UsbHidGamepad::task() {
    if (!pending) return;
    if (!tud_hid_ready()) return;
    if (tud_hid_report(0, report, kReportLen)) pending = false;
}

// Mandatory HID class driver callbacks (not weak-stubbed in TinyUSB).

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t* buffer,
                               uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    if (reqlen < kReportLen) return 0;
    memcpy(buffer, report, kReportLen);
    return kReportLen;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const* buffer,
                           uint16_t bufsize) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
    // Input-only gamepad: no output/feature reports to handle.
}

#endif // CFG_TUD_HID
