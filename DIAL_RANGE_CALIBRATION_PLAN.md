# Dial-Range Mapping & Calibration Plan

## Goal
Let I2C gauge slaves drive the needle over a **sub-range** of the motor's full
rotation (e.g. a dial that only uses part of the ~345° travel), and make the
range **calibratable without repeatedly re-flashing the ATtiny slaves**.

Reflashing an ATtiny via the jtag2updi Nano is slow and tedious; reflashing the
Pico 2 master is fast (picotool over USB). So all range configuration and
tuning lives on the **master**; the slave becomes a dumb absolute-step follower.

## Constraints
- PCB coil wiring is fixed — motor direction can only be corrected in software.
- The A3144 hall switch is the only repeatable position reference; the dial's
  true zero mark may be an **offset** from the hall point.
- I2C slaves are not only stepper gauges — keep the protocol generic
  (they may be IO expanders or other specialized devices).
- DCS-BIOS output may represent **actual flight data** (a physical quantity to
  map) or **just needle position** (0-65535 already maps linearly to the dial).

## Key Design Decisions
1. **Mapping lives on the master.** The slave accepts an *absolute step count*
   in `[0, STEPS]` and does no scaling or reversal math.
2. **Dial range = `{dcsMin, dcsMax, stepMin, stepMax}`.** Linear map:
   `step = stepMin + (raw - dcsMin) * (stepMax - stepMin) / (dcsMax - dcsMin)`,
   clamped to `[0, maxSteps]`. `stepMax < stepMin` encodes reversal.
3. **Hall home offset is captured by `stepMin`/`stepMax` directly.** Steps are
   absolute in the slave's coordinate frame (hall = `0` or `STEPS`). The
   calibrator moves the needle to the dial's zero mark and records that step as
   `stepMin` — the offset from the hall is included, no separate field needed.
4. **Reversal is encoded in the master mapping** (`stepMax < stepMin`), but the
   slave keeps a compile-time `HALL_HOME_REF` (`0` or `STEPS`). This is
   required by the SwitecX25 library: `stepDown()` refuses to go below step 0,
   so a reversed-wired gauge must anchor the hall at `STEPS` to give the needle
   room to travel in the correct physical direction.

## Changes

### 1. Slave — dumb absolute-step follower (one-time flash, L + R)
Files: `arduino_sketches/i2c_gauge_slave_switec/`, `arduino_sketches/i2c_gauge_slave_switec_r/`
- `handleSetPosition(uint16_t)` → clamp `[0, STEPS]` → `motor1.setPosition()`.
  Delete all scaling + reversal math.
- Replace `GAUGE_SCALE_REVERSED` with `HALL_HOME_REF` (`0` or `STEPS`), used
  only in `setHomePosition()`. Fuel-flow boards = `STEPS`.
- Add `STATUS_REQ` (0x05) readback: `onReceive` stores a response frame
  `{reg:0x80, cmd:0x05, len:2, currentStep LE16, crc}`; `Wire.onRequest`
  transmits it. Matches the master's write-restart-read `readFrame` pattern.
- Direction filter, hall homing, backlight handling unchanged.

### 2. Master — new `DialRangeOutputListener`
Files: `src/internal/I2cOutputListener.{h,cpp}`
- New scale mode / class with config `{dcsMin, dcsMax, stepMin, stepMax, maxSteps=1035}`.
- Handles both data shapes: real `dcsMin/dcsMax` for flight data, or
  `dcsMin=0, dcsMax=65535` when the value is already needle position.
- `examples/pico2/pico2_i2c_switec_demo.cpp`: replace the two fuel-flow
  listeners with `DialRangeOutputListener` (left 0x08, right 0x09).

### 3. Calibration tool
File: `test_programs/pico2_i2c_gauge_calibrator.cpp` (new)
- CLI over I2C, pattern from `X27_angle_calibrator.cpp`:
  - `g <addr>` select gauge
  - `z` home (send HOME_SENSOR)
  - `p <steps>` move to absolute step
  - `+` / `-` nudge
  - `m0` / `m1` mark current step as min / max dial position
  - `r` read back current step (STATUS_REQ)
  - `d <min> <max>` set DCS value range
  - `i` toggle reversal (swap stepMin/stepMax)
  - `t` test sweep
  - `s` print ready-to-paste config struct
  - `b <0-255>` backlight test
- Workflow: home → nudge to dial zero mark → `m0` → move to full-scale mark →
  `m1` → set DCS range → sweep-test → print struct → paste into demo.
  **ATtiny is never re-flashed for calibration.**

### 4. Host-side simulator
File: `Python Scripts/dial_range_sim.py` (new)
- Takes the same config, prints a mapping table + ASCII sweep trace
  (DCS value → step → degrees). Sanity-checks math and reversal before any
  hardware is touched.

### 5. Build config
- `CMakeLists.txt`: add the calibrator target (commented out; demo stays active).

## Verification
1. Compile both slaves with arduino-cli → flash via jtag2updi Nano → byte-verify.
2. Build calibrator on Pico → live-test both gauges.
3. Build demo with baked structs → confirm both gauges track correctly.

## Files Touched
- `arduino_sketches/i2c_gauge_slave_switec/i2c_gauge_slave_switec.ino`
- `arduino_sketches/i2c_gauge_slave_switec_r/i2c_gauge_slave_switec_r.ino`
- `arduino_sketches/README.md` — protocol / STATUS_REQ documentation
- `src/internal/I2cOutputListener.{h,cpp}`
- `examples/pico2/pico2_i2c_switec_demo.cpp`
- `test_programs/pico2_i2c_gauge_calibrator.cpp` (new)
- `Python Scripts/dial_range_sim.py` (new)
- `CMakeLists.txt`

## Status
- [x] Design agreed (master-side config, master-side reversal, STATUS_REQ readback, host simulator)
- [ ] Slave: absolute-step follower + STATUS_REQ + HALL_HOME_REF
- [ ] Master: `DialRangeOutputListener`
- [ ] Calibrator tool
- [ ] Host simulator
- [ ] Demo updated with new listener
- [ ] On-hardware verification
