# Arduino Sketches — DCS-BIOS I2C Slave Reference

## Wiring

| Signal | Pico 2 (RP2350) | Arduino Nano | CH32V003 |
|--------|-----------------|-------------|----------|
| SDA    | GP6             | A4          | PA1      |
| SCL    | GP7             | A5          | PA2      |
| GND    | GND             | GND         | GND      |
| VCC    | 3.3V            | 5V or 3.3V  | 3.3V     |

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
