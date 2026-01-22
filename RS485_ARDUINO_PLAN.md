# Arduino RS485 Compatibility Plan & Progress

## Goal
Enable a Pico (RP2350) DCS‑BIOS master to communicate with Arduino DCS‑BIOS RS485 slave devices **without modifying the Arduino library**.

## Constraints
- No changes to Arduino DCS‑BIOS library files.
- All changes live in this repository only.

## Summary of What’s Implemented
- **Arduino RS485 framing & polling (Pico master path)** behind `DCSBIOS_RS485_ARDUINO`.
- **Active‑only polling** with fallback to full scan.
- **Configurable timings** (poll interval, response timeout, inter‑frame gap, max misses).
- **Non‑hardware framing test** (optional).
- **Pico master example** (optional).

## Key Files Added/Changed
- Arduino RS485 helpers:
  - `src/internal/rs485_arduino.h`
  - `src/internal/rs485_arduino.cpp`
- RS485 byte‑send helper:
  - `src/internal/rs485.h`
  - `src/internal/rs485.cpp`
- Arduino RS485 host loop (compile‑time gated):
  - `src/internal/core1_tasks.cpp`
- Pico master example:
  - `examples/RS485_Arduino_Master_Pico.cpp`
- Optional framing test:
  - `test_programs/rs485_arduino_framing_test.cpp`
- Build configuration:
  - `CMakeLists.txt` (now builds the Pico master example and defines `DCSBIOS_RS485_ARDUINO`)

## Current Build Target
`CMakeLists.txt` now builds:
- `examples/RS485_Arduino_Master_Pico.cpp`
- `DCSBIOS_RS485_ARDUINO` is defined for the target

## Notes on Arduino RS485 Protocol (for reference)
- Frame: `address`, `msgType`, `length`, `data`, `checksum`.
- Broadcast export: `address=0`, `msgType=0`, `length>0`.
- Slave poll: `address=<slave>`, `msgType=0`, `length=0`.
- Slave reply: `length`, `msgType=0`, `data`, `checksum`.
- Arduino requires an inter‑frame quiet gap (~500µs). Default here is **600µs**.

## Configuration Knobs
Defined in `src/internal/rs485_arduino.h`:
- `DCSBIOS_RS485_ARDUINO_ACTIVE_ONLY`
- `DCSBIOS_RS485_ARDUINO_MAX_MISSES`
- `DCSBIOS_RS485_ARDUINO_POLL_INTERVAL_US`
- `DCSBIOS_RS485_ARDUINO_RESPONSE_TIMEOUT_US`
- `DCSBIOS_RS485_ARDUINO_INTERFRAME_GAP_US`
- `DCSBIOS_RS485_ARDUINO_FRAME_MAX`

## Progress Checklist
- [x] Branch created and pushed: `feature/rs485-arduino-slaves`
- [x] Arduino RS485 framing/polling implemented (Pico master path)
- [x] Active‑only polling implemented
- [x] Optional framing test added
- [x] Pico master example added and enabled
- [x] Build passes in current configuration

## Waiting on Hardware
We need real Pico + Arduino RS485 slave devices to:
- Verify timing (gap, polling interval, timeout)
- Validate TXEN control + inter‑frame silence
- Measure latency/throughput
- Confirm full interoperability with Arduino slaves

## Next Steps Once Hardware Is Available
1. **Wire RS485 transceiver** (Pico TX/RX + DE/RE to `RS485_EN` pin).
2. Flash **Pico master example**.
3. Flash Arduino slave(s) with standard Arduino DCS‑BIOS RS485 sketches.
4. Observe:
   - Poll success rate
   - Response latency
   - Miss/timeout behavior
5. Tune timing constants in `rs485_arduino.h`.

## Notes
- We are intentionally **not** adding Arduino example sketches to this repo (they live in the Arduino library).
- The Pico master example exists here to demonstrate Arduino‑compatible RS485 behavior on this platform.
