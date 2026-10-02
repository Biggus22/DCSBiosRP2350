# DCSBiosRP2350 — Agent Guide

## Build system

- **CMake + Ninja** via Raspberry Pi Pico VS Code Extension v2.2.0
- Pico SDK at `~/.pico-sdk/sdk/2.2.0/`, toolchain at `~/.pico-sdk/toolchain/14_2_Rel1/`
- Build: `ninja -C build` (or VS Code task "Compile Project")
- Flash: picotool (`picotool load build/DCSBiosRP2350.uf2 -fx`) or OpenOCD via VS Code task "Flash"

## Active build target

**Only ONE `add_executable` may be uncommented in `CMakeLists.txt` at a time.** All others must be commented out. The project name is always `DCSBiosRP2350` regardless of which `.cpp` is the entrypoint. To switch which firmware to build, comment the current line and uncomment another.

## Board selection

Exactly one of `FOX1_BOARD`, `FOX2_BOARD`, or `PICO_BOARD` must be `#define`d before including `DcsBios.h`. This is typically done at the top of each firmware `.cpp` file. Pin definitions live in `src/internal/FoxConfig.h`.

- FOX2 has I2C0 SDA/SCL **swapped** (board error): SDA=GP5, SCL=GP4
- RS485 on UART0: TX=GP0, RX=GP1, EN=GP2 (all boards)

## Entrypoint & dual-core architecture

All firmware files `#include "DcsBios.h"` — that is the single framework entrypoint. The pattern is always:

```cpp
#define PICO_BOARD          // or FOX1_BOARD / FOX2_BOARD
#include "DcsBios.h"
#include "internal/FoxConfig.h"
// ... other includes ...

int main() {
    stdio_init_all();
    DcsBios::initHeartbeat(HEARTBEAT_LED);
    DcsBios::initDeviceAddressPins(ADDR0, ADDR1, ADDR2, ADDR3);
    uint8_t addr = DcsBios::readDeviceAddress();
    DcsBios::currentBoardMode = DcsBios::determineBoardMode(addr);
    DcsBios::init_rs485_uart(uart0, UART0_TX, UART0_RX, RS485_EN, 250000);
    multicore_launch_core1(DcsBios::core1_task);
    DcsBios::setup();
    while (true) {
        DcsBios::loop();
        DcsBios::updateHeartbeat();
        sleep_us(10);
    }
}
```

- **Core 0**: DCS-BIOS protocol parser, input polling (switches, encoders, pots), output listeners (LEDs, displays, servos, steppers)
- **Core 1**: Board-mode-specific task (HOST/SLAVE/USB_ONLY/RS485_TERMINAL) — runs USB CDC ↔ RS485 bridging
- Inter-core FIFO: 32-bit multicore FIFO, low 8 bits = ASCII byte, with mutex for atomic sends

## Board modes (selected by 4 address pins, GPIO 36-39)

| Address | Mode | Core 1 behavior |
|---------|------|----------------|
| 0x0 | HOST | USB CDC → RS485 broadcast, poll slaves, forward responses |
| 0x1–0xD | SLAVE | RS485 → Core0 parser, Core0 events → RS485 (prepend address) |
| 0xE | RS485_TERMINAL | Bidirectional RS485 ↔ USB serial terminal |
| 0xF | USB_ONLY | USB CDC → Core0 parser only (no RS485) |

## Arduino RS485 compatibility

Gated behind `DCSBIOS_RS485_ARDUINO` compile flag (`target_compile_definitions(... PRIVATE DCSBIOS_RS485_ARDUINO)`). When enabled, the HOST mode uses binary frame protocol `[addr][msgType][len][data...][checksum]` with active-only polling. Tunables in `src/internal/rs485_arduino.h`.

## IO expanders & drivers

| Component | Interface | File |
|-----------|-----------|------|
| AW9523B | I2C (0x58/0x59/0x5B) | `aw9523b.h/.cpp` |
| MCP23S17 | SPI | `MCP23S17.h/.cpp` |
| PCF8575 | I2C | `PCF8575.h/.cpp` |
| ST7789 | SPI | `ST7789.h/.cpp` |
| EPaper 2.13" | SPI | `EPaper213.h/.cpp` |
| TM1637 | 2-wire | `TM1637.h/.cpp` |
| HT16K33A | I2C | `ht16k33a.h/.cpp` |
| WS2812/NeoPixel | PIO | `ws2812.h/.cpp` + `ws2812.pio` |
| PIO I2C | PIO | `pio_i2c.h/.cpp` + `i2c.pio` |

## I2C subsystem

The RP2350 is the I2C **master** on `i2c1` (GP6 SDA / GP7 SCL, 100 kHz default).
Two directions exist:

- **sim -> panel** (DCS-BIOS drives a remote output): `I2cOutputListener`, a
  global `Int16Buffer` subclass. Edge-triggered, sends only on change. Scale
  modes: `LED_TOGGLE`, `LED_BRIGHTNESS`, `STEPS_16BIT`, `ANGLE_CENTIDEG`,
  `BACKLIGHT` (cmd 0x09).
- **panel -> sim** (an input sub-panel on the same PCB reports switches/pots):
  construct an `I2cInputPanel` (owns the transaction, `poll()` it once per pass
  *before* `DcsBios::loop()`), then wrap it in `I2cInputSwitch2Pos`,
  `I2cInputSwitch3Pos2Bit` or `I2cInputPotentiometer`. These are
  `PollingInput` subclasses with the same debounce/EWMA/hysteresis semantics as
  the GPIO-backed classes.

Wire frame is `[reg][cmd][len][data...][crc8]`, CRC-8 poly 0x07 init 0x00 no final
XOR (CRC-8/ATM, **not** Dallas/Maxim 1-Wire). A slave sets bit 0x80 in `reg` on
its response frames. `I2cBusHwMaster::readFrameSized()` is the only working read
path — the older `readFrame()` always requests 254 bytes and cannot talk to a
real slave. `I2cBus::recoverBus()` hand-clocks SCL out of a wedged state
(SDK 2.x removed `i2c_recover_bus`); the poller calls it every 20 failed polls.

Reference slave sketches live in `arduino_sketches/`, each with a copy of
`i2c_slave_defs.h` (keep the copies in sync by hand — they have drifted before).
External 4.7 kΩ pull-ups to 3.3 V are mandatory; the internal ones are too weak.

## Test programs

Standalone `.cpp` files in `test_programs/` — no test framework. Each is a complete firmware binary. Build by uncommenting its `add_executable` in `CMakeLists.txt`. The RS485 framing test (`rs485_arduino_framing_test.cpp`) runs on the host PC's serial console.

## Directory map

| Path | Purpose |
|------|---------|
| `src/internal/` | All framework source — drivers, protocol, RS485, core tasks |
| `src/DcsBios.h` | Master header — include this in all firmware |
| `examples/F-4E/` | F-4E Phantom panel firmwares |
| `test_programs/` | Standalone hardware test binaries |
| `Python Scripts/dcsbios.py` | DCS-BIOS utility script |
| `tools/font_to_c_bitmap.py` | Font → C bitmap converter |
| `LEFT_SUBPANEL_PINMAP.md` | Reference: F-4E left subpanel pinout |
