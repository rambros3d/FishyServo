## Why

The custom N20-Servo PCB (STM32H503KBU6) lacks external physical pull-up resistors on the dedicated sensor I2C bus (PB7/PB6). Before operating the TMAG5273 magnetic sensor in full application firmware, we need an automated multi-speed I2C diagnostic scanner that enables internal pull-ups on PB6/PB7 and streams test results over SWD using SEGGER RTT via ST-Link / pyOCD, validating hardware communication without needing external UART adapters or soldering.

## What Changes

- Add an `i2c-scanner` capability to scan I2C addresses (0x03 to 0x77) across multiple clock speeds (10 kHz, 50 kHz, 100 kHz, 400 kHz).
- Reconfigure STM32 GPIO registers to assert internal pull-ups on PB6 and PB7 in Alternate Function Open-Drain mode across clock speed transitions.
- Integrate lightweight SEGGER RTT logging over the existing SWD connection (`SWDIO`/`SWCLK`) so debug output can be monitored with `pyocd rtt` without disabling SWD.
- Add a new PlatformIO build environment `[env:i2c_scanner]` in `firmware/platformio.ini`.
- Provide automated flashing and live testing instructions with expected TMAG5273 address detection (0x35, 0x22, 0x78, or 0x49).

## Capabilities

### New Capabilities
- `i2c-scanner`: Multi-speed I2C bus probing with internal pull-up support and real-time RTT debug streaming over SWD.

### Modified Capabilities
<!-- None: openspec/specs is currently empty -->

## Impact

- **Firmware**: Adds `firmware/i2c_scanner/main.cpp` and minimal RTT support files (`firmware/i2c_scanner/SEGGER_RTT*`).
- **Build System**: Adds `[env:i2c_scanner]` to `firmware/platformio.ini`.
- **Hardware/Tooling**: Uses the existing ST-Link SWD connection and `pyocd rtt` on the host to monitor scan results.
