# Dial-Range Mapping & Calibration Plan (v2)

## Architecture

**Calibrate once on a Pico test bench, then bake the config into the slave.
Production slaves are smart; the master is dumb.**

The calibration tool lives on the Pico 2 master. Each ATtiny slave runs a
"dumb follower" calibration firmware during calibration (accepts absolute
steps), then gets re-flashed with a "smart" production firmware that holds
the dial-range config and does its own DCS-to-step mapping. This is a
one-time flash per gauge — calibration config is static once set.

The master sends the raw DCS-BIOS 16-bit value (0-65535) to each slave.
The slave maps it to its calibrated step range locally. The master does
zero per-gauge math — it just shovels values over I2C.

## Constraints
- PCB coil wiring is fixed — motor direction can only be corrected in software.
- The A3144 hall switch is the only repeatable position reference; the dial's
  true zero mark may be an **offset** from the hall point.
- I2C slaves are not only stepper gauges — keep the protocol generic
  (they may be IO expanders or other specialized devices).
- DCS-BIOS output may represent **actual flight data** (a physical quantity to
  map) or **just needle position** (0-65535 already maps linearly to the dial).
- Reflashing ATtiny via jtag2updi is slow, but this is a one-time event per
  gauge during initial build — acceptable.

## Key Design Decisions
1. **Mapping lives on the slave.** The slave holds `{dcsMin, dcsMax, stepMin,
   stepMax}` and does the linear map itself. The master is fire-and-forget:
   send raw DCS value, done.
2. **Dial range = `{dcsMin, dcsMax, stepMin, stepMax}`.** Linear map:
   `step = stepMin + (raw - dcsMin) * (stepMax - stepMin) / (dcsMax - dcsMin)`,
   clamped to `[0, STEPS]`. `stepMax < stepMin` encodes reversal.
3. **Hall home offset is captured by `stepMin`/`stepMax` directly.** Steps are
   absolute in the slave's coordinate frame (hall = `0` or `STEPS`). The
   calibrator moves the needle to the dial's zero mark and records that step as
   `stepMin` — the offset from the hall is included, no separate field needed.
4. **Reversal is encoded in the config** (`stepMax < stepMin`), but the slave
   keeps a compile-time `HALL_HOME_REF` (`0` or `STEPS`). This is required by
   the SwitecX25 library: `stepDown()` refuses to go below step 0, so a
   reversed-wired gauge must anchor the hall at `STEPS` to give the needle
   room to travel in the correct physical direction.
5. **Two firmware phases per gauge:**
   - **Calibration firmware** — dumb absolute-step follower (same as today's
     slave, plus `STATUS_REQ` readback). Flash once with the calibrator Pico.
   - **Production firmware** — smart slave with baked-in range config. Receives
     raw DCS values, maps to steps locally. This is the firmware that ships.

## Protocol Changes

### New command: `CMD_SET_DCS_VALUE` (0x0A)

```
Frame: [reg=0x01][cmd=0x0A][len=2][dcsValue LE16][crc8]
```

Slave receives the raw 16-bit DCS-BIOS value and applies the dial-range
mapping internally, then drives the motor. This replaces `CMD_SET_POSITION`
in production. The master just sends the raw value — no scaling, no config.

### Existing commands retained
- `CMD_SET_POSITION` (0x01) — absolute step, used by the calibration tool
- `CMD_HOME_SWEEP` (0x03) / `CMD_HOME_SENSOR` (0x04) — homing
- `CMD_STATUS_REQ` (0x05) — readback current step
- `CMD_CONFIG_SET` (0x08) — for future runtime config (EEPROM)
- `CMD_SET_BACKLIGHT` (0x09) — backlight PWM

### New command: `CMD_CONFIG_SET` payload for range config

```
Frame: [reg=0x01][cmd=0x08][len=12][dcsMin LE16][dcsMax LE16][stepMin LE16][stepMax LE16][reserved 4 bytes][crc8]
```

Future use: write calibration config to EEPROM at runtime without reflashing.
Not needed for v1 — config is compiled in.

## Changes

### 1. Slave — calibration firmware (one-time flash, L + R)

This is the existing slave firmware with minor additions for calibration
support. Used only during the calibration phase.

Files: `arduino_sketches/i2c_gauge_slave_switec/`, `arduino_sketches/i2c_gauge_slave_switec_r/`

- Keep `handleSetPosition(uint16_t)` as-is: accepts absolute step count,
  clamps to `[0, STEPS]`, drives motor. This is what the calibrator uses.
- Add `STATUS_REQ` (0x05) readback: `onReceive` stores a response frame
  `{reg:0x80, cmd:0x05, len:2, currentStep LE16, crc}`; `Wire.onRequest`
  transmits it. Matches the master's write-restart-read `readFrame` pattern.
- Direction filter, hall homing, backlight handling unchanged.
- `GAUGE_SCALE_REVERSED` and `HALL_HOME_REF` unchanged for now — these
  are only relevant during homing and are overwritten by the calibrator.

### 2. Slave — production firmware (ships on the gauge)

New file: `arduino_sketches/i2c_gauge_slave_switec/i2c_gauge_slave_production.ino`
(and corresponding `_r` variant).

Based on the calibration firmware with these changes:

- **Delete** `handleSetPosition()` and the `CMD_SET_POSITION` handler.
- **Add** calibration config struct (compiled in):

```c
// --- Dial-range calibration config ---
// Written by the calibrator tool. Paste output here.
static const struct {
    uint16_t dcsMin;    // DCS-BIOS value at dial zero
    uint16_t dcsMax;    // DCS-BIOS value at dial full scale
    uint16_t stepMin;   // Motor step at dial zero (absolute, from hall)
    uint16_t stepMax;   // Motor step at dial full scale (may be < stepMin for reversal)
} dialRange = {
    .dcsMin  = 0,
    .dcsMax  = 65535,
    .stepMin = 1035,    // hall at STEPS (reversed wiring)
    .stepMax = 0,       // zero flow = step 0 (far end from hall)
};
#define HALL_HOME_REF STEPS   // 0 or STEPS — set by calibrator
```

- **Add** `handleSetDcsValue(uint16_t rawValue)`:

```c
static void handleSetDcsValue(uint16_t rawValue) {
    // Clamp DCS input to configured range
    if (rawValue < dialRange.dcsMin) rawValue = dialRange.dcsMin;
    if (rawValue > dialRange.dcsMax) rawValue = dialRange.dcsMax;

    // Linear map: DCS value → absolute step
    uint32_t num = (uint32_t)(rawValue - dialRange.dcsMin)
                 * (uint32_t)(abs((int16_t)(dialRange.stepMax - dialRange.stepMin)));
    uint32_t den = (uint32_t)(dialRange.dcsMax - dialRange.dcsMin);
    uint16_t target;
    if (dialRange.stepMax >= dialRange.stepMin) {
        target = dialRange.stepMin + (uint16_t)(num / den);
    } else {
        target = dialRange.stepMin - (uint16_t)(num / den);
    }

    // Direction-locked filter (unchanged from calibration firmware)
    if (lastDirection >= 0 && target >= lastTarget) {
        lastDirection = 1;
    } else if (lastDirection <= 0 && target <= lastTarget) {
        lastDirection = -1;
    } else if (abs((int16_t)target - (int16_t)lastTarget) > DIR_FILTER_THRESHOLD) {
        lastDirection = (target > lastTarget) ? 1 : -1;
    } else {
        return;
    }
    lastTarget = target;
    motor1.setPosition(target);
}
```

- **Replace** `CMD_SET_POSITION` handler with `CMD_SET_DCS_VALUE` (0x0A).
- **Retain** `CMD_HOME_SWEEP`, `CMD_HOME_SENSOR`, `CMD_STATUS_REQ`,
  `CMD_SET_BACKLIGHT`.
- Replace `GAUGE_SCALE_REVERSED` with `HALL_HOME_REF` (`0` or `STEPS`),
  used only in `setHomePosition()`.

### 3. Master — no per-gauge config needed

The master becomes simpler. For gauges using `CMD_SET_DCS_VALUE`:

Files: `src/internal/I2cOutputListener.{h,cpp}`

- Add new scale mode `DCS_RAW = 5`: sends the raw 16-bit DCS value via
  `CMD_SET_DCS_VALUE` (0x0A) with no scaling.

```cpp
case DCS_RAW:
    dataBuf[0] = raw & 0xFF;
    dataBuf[1] = (raw >> 8) & 0xFF;
    dataLen = 2;
    cmd = I2C_CMD_SET_DCS_VALUE;  // 0x0A
    break;
```

- No `DialRangeOutputListener` class needed — the existing
  `I2cOutputListener` with `DCS_RAW` mode handles it.

### 4. Calibrator tool

File: `test_programs/pico2_i2c_gauge_calibrator.cpp` (new)

CLI over I2C, pattern from `X27_angle_calibrator.cpp`:

- `g <addr>` — select gauge
- `z` — home (send HOME_SENSOR)
- `p <steps>` — move to absolute step
- `+` / `-` — nudge (±1 step)
- `m0` / `m1` — mark current step as min / max dial position
- `r` — read back current step (STATUS_REQ)
- `d <min> <max>` — set DCS value range
- `i` — toggle reversal (swap stepMin/stepMax)
- `t` — test sweep (0% → 100% → 0%)
- `s` — print ready-to-paste production config struct
- `b <0-255>` — backlight test

**Workflow:**
1. Flash calibration firmware on slave (one-time, same as today's firmware)
2. Connect calibrator Pico to slave bus
3. Home → nudge to dial zero mark → `m0` → move to full-scale mark → `m1`
4. Set DCS range → sweep-test → `s` to print config
5. Paste config into production firmware source
6. Flash production firmware on slave
7. Connect slave to the real Pico 2 master (which sends `DCS_RAW` values)

**ATtiny is re-flashed exactly twice:** once for calibration mode, once for
production. Both flashes are during initial build, not during operation.

### 5. Host-side simulator

File: `Python Scripts/dial_range_sim.py` (new)

Takes the same config (`dcsMin, dcsMax, stepMin, stepMax`), prints a
mapping table + ASCII sweep trace (DCS value → step → degrees).
Sanity-checks math and reversal before any hardware is touched.

### 6. Build config

- `CMakeLists.txt`: add the calibrator target (commented out; demo stays active)

## Verification
1. Compile calibration slave with arduino-cli → flash via jtag2updi Nano →
   byte-verify.
2. Build calibrator on Pico → live-test both gauges → print config structs.
3. Paste configs into production firmware → compile → flash slaves.
4. Connect production slaves to the real Pico 2 master → confirm both gauges
   track DCS-BIOS values correctly.

## Files Touched
- `arduino_sketches/i2c_gauge_slave_switec/i2c_gauge_slave_switec.ino` (calibration)
- `arduino_sketches/i2c_gauge_slave_switec_r/i2c_gauge_slave_switec_r.ino` (calibration)
- `arduino_sketches/i2c_gauge_slave_switec/i2c_gauge_slave_production.ino` (new — smart slave)
- `arduino_sketches/i2c_gauge_slave_switec_r/i2c_gauge_slave_production_r.ino` (new — smart slave)
- `arduino_sketches/i2c_slave_defs.h` (add CMD_SET_DCS_VALUE 0x0A)
- `arduino_sketches/README.md` — protocol / STATUS_REQ documentation
- `src/internal/I2cOutputListener.{h,cpp}` (add DCS_RAW scale mode)
- `src/internal/I2cCommand.h` (add I2C_CMD_SET_DCS_VALUE 0x0A)
- `examples/pico2/pico2_i2c_switec_demo.cpp` (update to use DCS_RAW mode)
- `test_programs/pico2_i2c_gauge_calibrator.cpp` (new)
- `Python Scripts/dial_range_sim.py` (new)
- `CMakeLists.txt`

## Status
- [x] Design agreed — master-dumb / slave-smart architecture (v2)
- [ ] Protocol: add CMD_SET_DCS_VALUE (0x0A)
- [ ] Slave: calibration firmware (existing + STATUS_REQ)
- [ ] Slave: production firmware with baked-in dial-range config
- [ ] Master: DCS_RAW scale mode in I2cOutputListener
- [ ] Calibrator tool
- [ ] Host simulator
- [ ] Demo updated with DCS_RAW mode
- [ ] On-hardware verification
