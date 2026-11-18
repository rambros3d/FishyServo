## Why

The N20-Servo board (STM32H503KBU6) integrates dual TI DRV8411A H-bridges, a TI TMAG5273 3D magnetic Hall sensor on I2C1, and a host I2C2 target interface. The current firmware only performs basic sensor polling and LED brightness testing. Upgrading the firmware to full closed-loop DC servo control with BareFOC turns the board into an intelligent, register-addressable smart micro-actuator with closed-loop angle control and current mirror telemetry.

## What Changes

- Link `N20-Servo/firmware` with BareFOC ecosystem (`barefoc`, `barefoc-drivers`, `barefoc-multibus`, `barefoc-stm32`, `tmag-rs`).
- Configure Channel D motor driver on `PA9` (`TIM1_CH2`) and `PB0` (`TIM3_CH3`) via `DCDriver2PWM`.
- Configure integrated current mirror sensing on `PA5` (`ADC1_INP19`) via `CurrentMirrorSense`.
- Interface with `Tmag5273Sensor` on `I2C1` (`PB6`/`PB7`) for continuous shaft position feedback.
- Implement closed-loop position (angle) servo control loop via `DCMotor` running at 1 kHz.
- Expose the BareFOC Multibus register map over `I2C2` (`PB3`/`PB4`) at target address `0x40`.

## Capabilities

### New Capabilities
- `n20-closed-loop-servo`: Closed-loop position servo control on Channel D with TMAG5273 angle feedback, DRV8411A current mirror sensing, and I2C2 Multibus target register control.

### Modified Capabilities
<!-- None -->

## Impact

- **Affected Directory**: `firmware/` (`Cargo.toml`, `src/main.rs`).
- **Dependencies**: Adds local path dependencies to `bareFOC` crates.
- **Hardware Peripherals**: `TIM1`, `TIM3`, `TIM2` (LEDs), `I2C1` (TMAG5273), `I2C2` (Host Target), `ADC1` (`ISENS_D`).
