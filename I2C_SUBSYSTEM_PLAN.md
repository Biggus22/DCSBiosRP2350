# I2C Subsystem — Implementation Plan

> **IMPORTANT**: A new branch must be created for this work. Until this subsystem
> is proven stable on hardware, all commits relating to this implementation may
> only be made against that branch. Merge to main only after the Phase 1
> verification sequence passes end-to-end.

## Scope

Implement an I2C master/slave subsystem for DCS-BIOS gauge control. The RP2350
acts as an I2C **master** (primary role) driving remote gauge-controller slaves
(Arduino Nano, CH32V003) or optionally as an **I2C slave** addressed by an
external master. Wire protocol uses variable-length command frames with
Maxim/Dallas CRC-8.

## Design decisions

| Decision | Value |
|----------|-------|
| Target board | Generic Pico 2 (`PICO_BOARD`) |
| I2C instance | `i2c1` on GP6 (SDA), GP7 (SCL) |
| Bus speed | 400 kHz default, 100 kHz fallback |
| Pull-ups | 4.7 kΩ to 3.3 V (cable <1 m) |
| Slave voltage | 3.3 V logic on bus; 5 V slaves must level-shift |
| Multi-master | Not supported — RP2350 is either master or slave, not both |
| Update rate | ~30 Hz (DCS-BIOS native cadence) |
| Protocol version | 0x01, reported via SYSTEM register |
| Board mode | 0xF (USB_ONLY) hardcoded for Phase 1 PoC |

## Architecture

```
DCS-BIOS ──► Core 0 parser ──► Int16Buffer ──► I2cOutputListener
    (USB CDC)                                  │
                                               ├── I2cBusHwMaster (i2c1)
                                               │      └── sendFrame([reg][cmd][len][data][crc8])
                                               │
                                           Remote Nano/CH32V003
                                            └── decodes frame
                                            └── analogWrite(LED, brightness)
```

**Output path** (DCS-BIOS → I2C slave): `I2cOutputListener` extends
`Int16Buffer`, registers as an `ExportStreamListener`. On DCS-BIOS write,
its `loop()` redirect scales the value and calls `bus_->sendFrame()`.

**Input path** (I2C slave → DCS-BIOS — Phase 2+): `I2cInputListener`
extends `PollingInput`, reads incoming frames, calls
`sendDcsBiosMessage()`.

## Wire protocol

### Frame format

```
[reg:1][cmd:1][len:1][data:len][crc8:1]
```

- **reg**: register/gauge ID. `0x00` = SYSTEM, `0x01..0x7E` = user.
  Bit `0x80` = response flag (slave→master direction).
- **cmd**: action on the addressed register.
- **len**: payload byte count.
- **data**: cmd-specific payload.
- **crc8**: Maxim/Dallas CRC-8 (poly 0x07, init 0x00, no final XOR) over
  `[reg..data]`.

### Command IDs

| ID | Name | Phase | Payload |
|----|------|-------|---------|
| 0x01 | SET_POSITION | 1 | varies by scale mode (see I2cOutputListener) |
| 0x02 | SET_ANGLE | 2 | `uint16_le` centidegrees (0..31500) |
| 0x03 | HOME_SWEEP | 2 | none |
| 0x04 | HOME_SENSOR | 3 | `uint8` sensor pin index |
| 0x05 | STATUS_REQ | 2 | none; response = `uint8` state |
| 0x06 | SET_MODE | 2 | `uint8` mode flags |
| 0x07 | CONFIG_GET | 3 | `uint8` key; response = config blob |
| 0x08 | CONFIG_SET | 3 | `uint8` key + value |
| 0x09 | SET_BACKLIGHT | 1 | `uint8` brightness (0..255) |
| 0x7F | RESET | 1 | none |

## Phase 1 deliverables

### Naming

- `I2cFrame.{h,cpp}` — frame encode/decode, Dallas CRC-8
- `I2cBus.h` — abstract bus interface
- `I2cBusHwMaster.{h,cpp}` — concrete i2c1 master implementation
- `I2cCommand.{h,cpp}` — command enum + handler registry
- `I2cProtocol.{h,cpp}` — SYSTEM register default handlers
- `I2cOutputListener.{h,cpp}` — DCS-BIOS→I2C listener (Int16Buffer subclass)
- `I2cInputListener` — documented placeholder; implemented in Phase 2+
- `examples/pico2/pico2_i2c_led_demo.cpp` — Pico 2 demo firmware
- `test_programs/i2c_frame_unittest.cpp` — frame unit tests
- `arduino_sketches/i2c_gauge_slave_led/i2c_gauge_slave_led.ino` — ref sketch
- `arduino_sketches/README.md` — wiring + voltage rules

### I2cOutputListener::ScaleMode

| Mode | Value | Scaling |
|------|-------|---------|
| `LED_TOGGLE` | 0 | `(val != 0) ? 1 : 0` |
| `LED_BRIGHTNESS` | 1 | `(val * 255) / 65535` (Phase 1 PoC) |
| `STEPS_16BIT` | 2 | `val & 0xFFFF` (Phase 2) |
| `ANGLE_CENTIDEG` | 3 | `(val * 31500) / 65535` (Phase 2) |
| `BACKLIGHT` | 4 | `(val * 255) / 65535` sent as `SET_BACKLIGHT` (0x09) |

### Edit locations

| File | Change |
|------|--------|
| `src/internal/FoxConfig.h:178-179` | Uncomment `#define I2C1_SDA 6` / `I2C1_SCL 7` |
| `CMakeLists.txt:118` | Add `pico_i2c_slave` to `target_link_libraries` |
| `CMakeLists.txt:152-153` | Add 6 new `.cpp` files to `target_sources` |
| `CMakeLists.txt:154` | Add commented-out `add_executable` toggles |
| `src/DcsBios.h:164` | Add 6 new `#include` lines |

## Verification

1. Build `i2c_frame_unittest.cpp` → flash → USB CDC: all 5 tests PASS.
2. Build `pico2_i2c_led_demo.cpp` → flash to Pico 2.
3. Load `i2c_gauge_slave_led.ino` → Nano.
4. Wire: GP6→A4, GP7→A5, GND→GND, 4.7kΩ to 3.3V.
5. Run DCS F-4E → adjust console lighting → Nano LED fades 0-255.
6. Disconnect SDA → no wedge → reconnect → resumes.

## Phase sequence

| Phase | Goal |
|-------|------|
| **1** | Transport + protocol skeleton + LED brightness PoC |
| 2 | STEPS_16BIT / ANGLE_CENTIDEG + real X27 over I2C + local |
| 3 | RP2350-as-I2C-slave (`I2cBusHwSlave`, `I2cInputListener`) |
| 4 | PIO I2C backend (conditional) |
| 5 | HOST orchestration + slave health monitoring |

## API signatures

See `src/internal/I2c*.h` headers and `I2C_SUBSYSTEM_PLAN.md` in the working
set for complete class definitions.
