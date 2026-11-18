## Why

A new I2C sensor (TMAG5273 3D Hall, plus at least one additional sensor) has been physically added to the PB6/PB7 I2C1 "sensor bus", but there is no firmware that confirms which devices actually ACK on that bus. Existing sketches (blink_test, tmag_test, i2c_scanner) cover other concerns; none is scoped to a post-wiring bring-up scan that proves the new hardware is reachable. RTT-only debug is now the agreed console, and the secondary PB3/PB4 I2C2 path is being retired in favor of TX-only UART/no use, so bus detection must be owned by I2C1.

## What Changes

- Reuse/adapt the proven multi-speed I2C scan logic against I2C1 (PB7 SDA, PB6 SCL) only.
- Ensure the new sensor (and any others on PB6/PB7) are identified via address probe table.
- Report results through SEGGER RTT over SWD (no serial console dependency).
- Document RTT as the canonical debug console; note I2C2 (PB3/PB4) is intentionally unused for testing so those pins stay free.
- No breaking changes to production pin map in platformio.ini.

## Capabilities

### New Capabilities
- `i2c-device-detection`: Bring-up detection of I2C devices on the PB6/PB7 sensor bus, with address identification and RTT reporting.

### Modified Capabilities
- (none)

## Impact

- Affected code: `firmware/` (i2c scanner / bring-up example).
- Build config: `platformio.ini` (reuse `[env:i2c_scanner]` or add a new env; no new lib_deps beyond existing).
- Debug flow: probe-rs primary, pyocd fallback; SEGGER RTT over SWD.
- Hardware: PB6/PB7 bus, internal pull-ups already fitted. No change to PA0/PA1 LED confirmation UX.
