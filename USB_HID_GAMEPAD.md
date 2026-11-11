# USB HID Gamepad Subsystem

Hybrid DCS-BIOS + USB HID for panels whose controls do not exist in the
DCS-BIOS protocol (e.g. the F-4E ECS panel's temperature pot and 3-position
toggle). The panel binds those controls in DCS like a normal joystick, while
the DCS-BIOS serial link keeps driving the panel's outputs.

Reference implementation: `examples/F-4E/F-4E_ECS_RP2350-Zero.cpp`.

## USB device layout

One composite device (the RP2350 has a single USB controller):

| Interface | Class        | Purpose                              | Endpoints     |
|-----------|--------------|--------------------------------------|---------------|
| 0/1       | CDC-ACM      | DCS-BIOS protocol (unchanged)        | 0x81, 0x02/0x82 |
| 2         | Vendor       | picotool soft reset                  | control only  |
| 3         | HID gamepad  | Panel inputs to the sim              | 0x83 IN       |

Descriptors live in `src/internal/usb_descriptors.c` (based on the SDK's
`stdio_usb_descriptors.c`, with the HID interface appended). HID gamepad
report: **signed 16-bit X axis + 2 buttons**, 3 bytes, no report ID.

## Panel opt-in

In `CMakeLists.txt`, the active `add_executable` is paired with:

```cmake
set(USB_HID_GAMEPAD 1)   # 0 for every non-HID panel
```

`USB_HID_GAMEPAD 1` adds these PRIVATE compile definitions:

- `CFG_TUD_HID=1` — activates TinyUSB's HID class driver (`hid_device.c` is
  compiled into every build; at 0 it is inert).
- `PICO_STDIO_USB_USE_DEFAULT_DESCRIPTORS=0` — empties the SDK's CDC-only
  descriptor file; the panel's own composite descriptors in
  `usb_descriptors.c` take over.
- `DCSBIOS_USB_HID_GAMEPAD` — gates the composite descriptor bodies.

Non-HID panels keep the SDK's stock descriptors byte-for-byte and are
unaffected by any of the HID code.

## USB ownership (important)

The Pico SDK's `pico_stdio_usb` keeps owning USB: it calls `tusb_init()` and
pumps `tud_task()` from its background IRQ. The firmware never touches that
machinery. **Do not define `LIB_TINYUSB_DEVICE`** — that flag tells the SDK
the application owns TinyUSB, which disables the SDK's USB bring-up. The
CDC DTR gating still applies: the Pico only services the serial port while
the host asserts DTR (pyserial and the DCS-BIOS client do; a naive
PowerShell/SerialPort reader must set `DtrEnable = $true`).

## Per-panel identity

A panel names itself with one line near the top of its `.cpp`:

```cpp
#include "internal/UsbIdentity.h"

DCSBIOS_USB_ID("DCSBiosRP2350", "F-4E ECS Panel")
DCSBIOS_USB_HID_NAME("ECS Gamepad")   // optional, rarely visible
```

These define strong `extern "C"` globals that override the weak defaults in
`usb_descriptors.c` (`"DCSBiosRP2350"` / `"DCS-BIOS Panel"`). The strings
are consumed during USB enumeration — after C++ static initialisation — so
a plain global is always set in time; there is no init-order contract.
Give each panel type a distinct product name (and ideally its own
`USBD_PID` compile definition) so Windows treats them as separate devices.
The serial number is the flash ID, so two panels of the same type stay
distinct regardless.

## The gamepad class

`src/internal/UsbHidGamepad.{h,cpp}` — static class:

| Call | Meaning |
|------|---------|
| `begin()` | Marks the report dirty so the first report goes out as soon as the host is ready. Call once after `stdio_init_all()`. |
| `setAxis(int16_t)` | Absolute X axis, -32768..32767. |
| `setButton(uint8_t index, bool)` | Buttons 1 and 2 (`index` 0/1). |
| `task()` | Transmits the report if dirty and the endpoint is free. Call once per main-loop pass. |

All three setters are send-on-change: they only mark the report dirty.
HID is a stateful protocol — the last report stands; never resend
periodically. `UsbHidGamepad.cpp` also provides the three mandatory
TinyUSB HID callbacks (`tud_hid_get_report_cb`, `tud_hid_set_report_cb`,
`tud_hid_descriptor_report_cb`) — these are *not* weak-stubbed, so every
HID-enabled panel links against exactly one copy (in the framework .cpp).

The axis is signed because Windows sign-extends multi-byte HID item values:
a descriptor using an unsigned `0xFFFF` logical max is parsed as -1 and the
axis scales nonsense. Use `HID_LOGICAL_MIN_N(-32768, 2)` /
`HID_LOGICAL_MAX_N(32767, 2)` (the gamepad convention).

## Writing panel inputs

Inputs feed `UsbHidGamepad` directly. They are deliberately **not**
`DcsBios::PollingInput` subclasses — the CDC line must carry nothing but
genuine DCS-BIOS messages, and `PollingInput` would emit switch/pot traffic
to the sim.

Pattern (see the ECS firmware for complete working classes):

- **Switches:** poll GPIO with pull-ups, ~20 ms debounce, map stable states
  to buttons (a 3-position toggle: up = button 1, down = button 2, centre =
  neither).
- **Pots:** sample ADC every ~5 ms, EWMA filter, only send when the scaled
  value leaves a deadband. Scale ADC 0..4095 → -32768..32767 with
  `int32_t` maths and clamp through the `int16_t` cast.

EWMA trap: the converging form is `filtered += (raw - filtered) >> n`.
Writing `filtered += raw - (filtered >> n)` converges to `(n+1) × raw`,
which then wraps through the int16 cast — the axis sweeps its full range
n+1 times per pot travel.

Main loop order:

```cpp
while (true) {
    myToggle.poll();       // HID inputs first
    myPot.poll();
    DcsBios::loop();       // protocol + outputs
    DcsBios::updateHeartbeat();
    UsbHidGamepad::task(); // flush pending HID report
    sleep_us(10);
}
```

## DCS side

The board appears in `joy.cpl` under its product name with one axis and two
buttons. Bind them in DCS's control menu like any joystick; use DCS's axis
calibration for detents/inversion rather than panel-side scaling.

## Bench testing without DCS

The firmware parses the **binary** DCS-BIOS protocol only — text like
`PLT_INT_LIGHT_CONSOLE 65535` is ignored (the parser discards everything
until four `0x55` sync bytes). To exercise outputs with no sim and no DCS
client, inject frames over the COM port (DTR asserted):

```
55 55 55 55 | addrLo addrHi | countLo countHi | dataLo dataHi ...
```

e.g. full brightness on `F_4E_PLT_INT_LIGHT_CONSOLE` (0x2D8A):
`55 55 55 55 8A 2D 02 00 FF FF`.

A standalone panel with no DCS connection shows only the boot effect (e.g.
the 5-second startup green); a completely dark strip with callbacks firing
means the LED chain hardware — check the first LED first, since one damaged
SK6812 at the head blacks out everything behind it.

## Flashing

`picotool load -fx` from a *running* composite HID device is unreliable —
the reset-to-BOOTSEL fires but picotool often cannot re-acquire the device
and the board is left in BOOTSEL. Flash HID panels from manual BOOTSEL
(BOOTSEL button, or power-cycle with it held); stock-descriptor panels are
unaffected.
