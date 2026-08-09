# Arduino Sketches — DCS-BIOS I2C Slave Reference

## Wiring

| Signal | Pico 2 (RP2350) | Arduino Nano | CH32V003 |
|--------|-----------------|-------------|----------|
| SDA    | GP6             | A4          | PA1      |
| SCL    | GP7             | A5          | PA2      |
| GND    | GND             | GND         | GND      |
| VCC    | 3.3V            | 5V or 3.3V  | 3.3V     |

### Hall zero sensor (Switec gauge slave only, ATtiny1614)

A3144 hall switch on PA3 is used as the gauge zero (homing) sensor:

| A3144 | ATtiny1614 |
|-------|-----------|
| VCC   | 3.3V      |
| GND   | GND       |
| OUT   | PA3       |

- A3144 output is **open-collector, active-low** — the sketch uses the ATtiny
  internal pull-up (`INPUT_PULLUP`); an external 10 kΩ pull-up to 3.3V is
  recommended for reliable triggering.
- The Switec gauge has **no physical stop** — the A3144 is the only zero
  reference. Homing sweeps the full ~345° travel until the sensor triggers;
  that point becomes position 0. Runs at boot and on `CMD_HOME_SENSOR` (0x04)
  or `CMD_HOME_SWEEP` (0x03). If the first sweep misses the magnet, it sweeps
  the opposite direction once, then stops without a reference.

## Voltage levels

- RP2350 I/O is 3.3V. The I2C bus MUST be pulled up to 3.3V.
- Pull-up resistors: 4.7kΩ to 3.3V (sufficient for <1m cable at 400kHz).
- Arduino Nano at 5V: use a level shifter on SDA/SCL, OR power the Nano at 3.3V.
- CH32V003 at 3.3V: connects directly.

## Bus parameters

- Speed: 400 kHz (default), drop to 100 kHz for longer/noisier cables.
- Slave address: 0x08 (configurable in each sketch).

## Protocol

Frame format: `[reg:1][cmd:1][len:1][data:len][crc8:1]`

CRC-8: Maxim/Dallas polynomial 0x07, init 0x00, over `[reg..data]`.

See `src/internal/I2cFrame.h` in the main project for full spec.
