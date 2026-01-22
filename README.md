This is a fork of Justin's DCSBiosRP2350 repository.
This repository is intended to be a development fork to implement hardware that I need for my simpit.
It is a WIP. 

## Arduino RS485 compatibility (WIP)

This branch adds a compatibility path for Arduino DCS‑BIOS RS485 slaves. It is gated behind the
`DCSBIOS_RS485_ARDUINO` compile flag to avoid changing existing Pico‑to‑Pico behavior.

### Active‑only polling

When enabled, the RS485 master only polls slaves that recently replied. If none are known,
it falls back to a full scan. Tunables are in [src/internal/rs485_arduino.h](src/internal/rs485_arduino.h):

- `DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY`
- `DCSBIOS_RS485_ARDUINO_MAX_MISSES`
- `DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US`
- `DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US`

### Framing test (optional)

There is a lightweight framing test at
[test_programs/rs485_arduino_framing_test.cpp](test_programs/rs485_arduino_framing_test.cpp).
To build it, uncomment the corresponding `add_executable` line in [CMakeLists.txt](CMakeLists.txt).

### Pico master example (optional)

An Arduino-compatible RS485 master example is provided at
[examples/RS485_Arduino_Master_Pico.cpp](examples/RS485_Arduino_Master_Pico.cpp).
Enable it by uncommenting the `add_executable` line in [CMakeLists.txt](CMakeLists.txt)
and compiling with `DCSBIOS_RS485_ARDUINO` defined.
