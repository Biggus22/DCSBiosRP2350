# Transponder Panel — Hardware Plan

> Scope: new PCB based on the **Waveshare RP2350-Zero**, using 2× 74HC165
> shift registers for input expansion, a single I2C bus with six slaves, and
> RS485. Captures the wiring decision and pin allocation agreed during
> planning (2026-08-10).

## Board: RP2350-Zero constraints

- **29 usable GPIOs**: GP0–15, GP17–25, GP26–29.
- **GP16 is dedicated to the onboard WS2812 RGB LED** — do not route anything
  else to it.
- **GPIO36–39 are NOT available**: they are the QSPI flash pins
  (QSPI_SS/SCLK/SD0/SD1), consumed by the onboard W25Q16 and the BOOT button.
  The framework's existing `ADDR0-3` board-mode address scheme
  (`src/internal/FoxConfig.h:32-35`) cannot be used on this board.
- Board mode is therefore **fixed in firmware** (hardcoded address), matching
  the existing Zero examples (e.g. `boardAddress = 0xF` in
  `examples/F-4E/F-4E_VORILS_RP2350-Zero.cpp`).

## 74HC165 connection: daisy-chain on hardware SPI1

Two 74HC165s form a **single 16-input serial chain on 3 MCU pins**.

```
         3V3 ──┐            3V3 ──┐
         GND ──┤            GND ──┤
    ┌──────────┼────────┐   ┌─────┼──────────┐
    │ 74HC165 A         │   │  74HC165 B    │
    │          CLK ─────┼───┼──► CLK        │   P0-P7 = inputs 8-15
    │          SH/LD ───┼───┼──► SH/LD      │
    │          CE  ─────┼───┼──► CE         │
    │ P0-P7 = in0-7     │   │               │
    │  Q7 (pin9) ───────┼───┼──► DS (pin10) │
    └───────────────────┘   └──► Q7 (pin9) ─┼──► MCU GP12 (SPI1 MISO)
                                MCU GP14 (SPI1 SCK) ──► both CLK
                                MCU GP13 (SH/LD) ─────► both SH/LD
```

- **VCC = 3.3 V** (74HC is 2–6 V logic; no level shifting needed).
- **CE tied low** on both chips (always enabled after load).
- **GP14 = SPI1 SCK** → CLK of both chips.
- **GP13 = SH/LD** — driven as a plain GPIO.
- **GP12 = SPI1 MISO** ← Q7 of chip B (the chip wired to MISO).
- **SPI mode 1** (CPOL=0, CPHA=1): the 165 updates Q7 on the rising clock edge,
  so sampling on the falling edge reads clean data MSB-first with no
  bit-discard tricks. (Mode 0 would need an extra dummy clock + first-bit
  discard.)
- **Read sequence** per scan:
  1. GP13 low → high (latch parallel inputs)
  2. `spi_read_blocking(spi1, 0x00, out, 2)` → 2 bytes
- **Byte order**: first byte = chip B (inputs 8–15, the chip wired to MISO),
  second byte = chip A (inputs 0–7).

### Pin 10 (DS / SER) — serial data input

- **Chip B**: pin 10 ← chip A's Q7 (pin 9). This is the daisy-chain link.
- **Chip A** (farthest from MCU): pin 10 is the **tail of the chain —
  intentionally unconnected**. It is fine to leave it floating because exactly
  16 bits are clocked for two chips: whatever state that pin carries is
  clocked into the LSB position behind the last bit read and never reaches the
  MCU. **Best practice: tie chip A's pin 10 to GND** so the discarded LSB is a
  defined 0.
- If a third 165 is ever added, the third chip's Q7 connects to chip A's pin 10.

## RP2350-Zero pin allocation

| Function | Pins | Notes |
|----------|------|-------|
| RS485 UART0 | GP0 / GP1 / GP2 | TX / RX / EN (existing FoxConfig) |
| I2C bus (6 slaves) | GP6 / GP7 | `i2c1`, 100 kHz; 4.7 kΩ pull-ups (2.2 kΩ if 400 kHz) |
| 74HC165 chain | GP14 / GP12 / GP13 | SCK / MISO / SH-LD via SPI1 |
| Fixed board address | — | hardcoded in firmware |
| Reserved | GP16 | onboard WS2812 — do not route |
| Free | GP3, 4, 5, 8, 9, 10, 11, 15, 17–25, 26–29 | 17+ pins for encoders, LEDs, WS2812, etc. |

## Software deliverables

1. **New `WAVESHARE_RP2350_ZERO_BOARD` block in `src/internal/FoxConfig.h`**
   (the define is already whitelisted at FoxConfig.h:12).
2. **`HC165Input.{h,cpp}`** — small driver: init SPI1 (mode 1), owns GP13
   latch; `scan()` latches + reads 2 bytes into a `volatile uint16_t` (or byte
   array). One scan per `DcsBios::loop()` cycle.
3. **Reuse existing `Mat*` input classes** (`src/internal/Buttons.h:137`,
   `MatActionButton`/`MatActionButtonSet`) pointing at
   `&hc165State.byte[x]` — the storage-address pattern they already support.
   No new input plumbing needed.
4. **Example firmware** showing chain read on GP12/13/14.

## Hardware gotchas

- Do **not** pull GP16 anywhere on the carrier — it is the RGB LED line.
- Put **10 kΩ pull-ups on all 16 165 inputs** (switches to GND, active-low).
- Add **100 nF VCC decoupling** per 165 chip.
- **74HC165 has no internal pull-ups** — external pull-ups are mandatory.
- 74HC165 is **read-only** (inputs only, no output capability).
- Address pins ADDR0-3 (GPIO36-39) cannot be used on the Zero — firmware-fixed
  address, or repurpose 4 free GPIOs later if a DIP switch is ever wanted.

## Open items / next steps

- Implement `HC165Input` driver + FoxConfig board block + example once out of
  plan mode.
- Confirm the six I2C slave device types/addresses before finalizing bus
  pull-up values.
