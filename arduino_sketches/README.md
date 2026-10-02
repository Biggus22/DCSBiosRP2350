# Arduino Sketches — DCS-BIOS I2C Slave Reference

## Wiring

| Signal | Pico 2 (RP2350) | Arduino Nano | CH32V003 |
|--------|-----------------|-------------|----------|
| SDA    | GP6             | A4          | PC1      |
| SCL    | GP7             | A5          | PC2      |
| GND    | GND             | GND         | GND      |
| VCC    | 3.3V            | 5V or 3.3V  | 3.3V     |

The CH32V003's I2C1 is fixed to PC1/PC2, so `Wire.begin()` takes no pin
arguments. I2C2 is on PD0/PD1 — that is the SWD pair, so don't use it for I2C.
(The ATtiny1614 gauge slaves are different: they use `Wire.swap(1)` for
TWI0 pinswap-1, giving SDA = PA1 / SCL = PA2.)

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

- Speed: 100 kHz (the master's `I2C_DEFAULT_BAUD` in
  `src/internal/I2cBusHwMaster.h`). 400 kHz has to be requested explicitly at
  the `I2cBusHwMaster` constructor.
- Slave address: 0x08 (configurable in each sketch).
- 4.7 kΩ pull-ups to 3.3V are **mandatory** — the RP2350 side only enables the
  internal pull-ups (~50–80 kΩ), which is not enough.

## Flashing

| Target | Tool |
|--------|------|
| RP2350-Zero / Pico 2 | `picotool load build/DCSBiosRP2350.uf2 -fx` |
| ATtiny1614 (megaTinyCore) | `avrdude -p t1614 -c jtag2updi -P <port> -b115200 -e -Uflash:w:<hex>:i` |
| CH32V003 (CH32_Arduino core) | minichlink over WCH-Link SWD (see below) — **not** avrdude, **not** `arduino-cli upload` |

### CH32V003 via minichlink (WCH-Link, SWD)

Build the sketch first with the **CH32_Arduino** core (`CH32_Arduino:ch32v:CH32V00x_EVT`,
pnum=CH32V003F4) — do NOT build against the WCH:ch32v package, its Wire library
has the slave path compiled out. Then flash the resulting `.bin` with minichlink
(from the [ch32v003fun](https://github.com/cnlohr/ch32v003fun) repo; on this bench
at `H:\VS Code repositories\Arduino VS Code\tools\minichlink\`, with
`libusb-1.0.dll` kept next to the exe):

```powershell
.\tools\minichlink\minichlink.exe                                  # probe + chip detect
.\tools\minichlink\minichlink.exe -w <bin> flash
.\tools\minichlink\minichlink.exe -r <out> flash 16384             # read-back verify
.\tools\minichlink\minichlink.exe -r <out> 0x1FFFF800 16          # option bytes
.\tools\minichlink\minichlink.exe -D                              # PD7 -> GPIO (one-way option-byte change)
```

Gotchas:

- minichlink needs **WinUSB** bound to the probe's Interface 0 (MI_00, VID_1A86/PID_8010
  in WCH-Link mode) — set with Zadig, List All Devices, on `WCH-Link (Interface 0)`
  only; leave the serial interface (MI_01, COM port) alone. The probe must be in
  WCH-Link mode (PID_8010); CMSIS-DAP mode (PID_8012) will not work.
- `arduino-cli upload` (OpenOCD `wlinke`) needs the **WCH vendor driver** on that same
  interface instead — the two tools are mutually exclusive per driver binding. On this
  part, minichlink is the working path.
- `-r` requires an address AND a size; a bare option keyword is rejected despite what
  the usage text suggests.
- SOP16 parts (CH32V003A4M6) do not break out PD0/SWCLK — debug is the 1-wire SDI on
  PD1 plus GND.
- PD7 is NRST out of reset, not GPIO — `minichlink -D` flips the option byte once per
  chip (reversible with lowercase `-d`).

## Protocol

Frame format: `[reg:1][cmd:1][len:1][data:len][crc8:1]`

CRC-8: poly 0x07, init 0x00, no final XOR, over `[reg..data]`. This is
CRC-8/ATM (SMBus) — the bitwise form in `i2c_slave_defs.h` is byte-exact with
`i2cFrame_crc8()` in `src/internal/I2cFrame.cpp`; it is *not* the Dallas/Maxim
1-Wire CRC (that uses the reflected 0x8C polynomial).

Frames sent by a slave set bit 0x80 in the register byte.

See `src/internal/I2cFrame.h` in the main project for full spec.
