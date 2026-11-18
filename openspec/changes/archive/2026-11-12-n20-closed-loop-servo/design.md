## Context

The N20-Servo board integrates an STM32H503KBU6 microcontroller, two TI DRV8411A dual H-bridge drivers (supporting channels A through D), and an onboard TI TMAG5273 3D magnetic position sensor. To operate as an intelligent position servo, Channel D (`IN_D1` on `PA9`, `IN_D2` on `PB0`, `ISENS_D` on `PA5`) will be driven by BareFOC's `DCMotor` controller, with position feedback from `Tmag5273Sensor` and register control over the host `I2C2` bus (`PB3`/`PB4`) using `barefoc-multibus`.

## Goals / Non-Goals

**Goals:**
- Safe boot: Ensure all H-bridges (A, B, C, D) start in high-impedance Coast mode (`0/0`).
- Drive Channel D with `DCDriver2PWM` across `TIM1_CH2` (`PA9`) and `TIM3_CH3` (`PB0`) at 25 kHz.
- Read shaft angle from `Tmag5273Sensor` on `I2C1` (`PB6`/`PB7`) in 1-DOF joint mode.
- Sample load current from `PA5` (`ADC1_INP19`) using `CurrentMirrorSense` ($A_{IPROPI} = 200\ \mu\text{A/A}$).
- Run 1 kHz closed-loop angle servo control with velocity and voltage limits.
- Host communication via `barefoc-multibus` server over `I2C2` at address `0x40`.

**Non-Goals:**
- Simultaneous driving of channels A, B, C (reserved for multi-motor firmware expansion).
- Sensorless commutation (the board has a dedicated hardware magnetic encoder).

## Decisions

### 1. Dual-Timer PWM for Channel D
- **Choice**: `PA9` is mapped to `TIM1_CH2`, and `PB0` is mapped to `TIM3_CH3`. Both timers are configured for 25 kHz edge-aligned PWM with initial duty cycle 0.
- **Rationale**: `DCDriver2PWM<P1, P2>` in `barefoc-drivers` is generic over two independent `embedded_hal::pwm::SetDutyCycle` channels, allowing clean composition across two hardware timers without timer synchronization issues because only one half-bridge is active at any time.

### 2. Motor Task & I2C2 Multibus Architecture
- **Choice**:
  - `motor_task`: Periodic 1 kHz Embassy timer tick calling `motor.move_motor()` and updating shared telemetry.
  - `i2c_target_task`: Background task running `barefoc_stm32::bus_target_i2c::Stm32I2cTarget` on `I2C2` at address `0x40`, dispatching reads/writes to `barefoc_multibus::table::RegMap`.
- **Rationale**: Keeps motor control strictly deterministic and jitter-free, while host I2C communication is processed asynchronously.

### 3. Integrated Current Mirror Calibration
- **Choice**: During startup before enabling PWM, perform 500 samples of `PA5` (`ADC1_INP19`) to record quiescent zero-current offset, subtracting it from real-time load telemetry.
- **Rationale**: Accounts for analog routing offset and ADC baseline error.

## Risks / Trade-offs

- **[Risk] High stall current burns out micro N20 motor** → **Mitigation**: Constrain `voltage_limit` to 5.0V / 6.0V in `MotorConfig`, and monitor `CurrentMirrorSense` current feedback to clamp output effort if stall is detected.
- **[Risk] Motor direction opposite to encoder angle growth** → **Mitigation**: Swap logical polarity or PWM pin assignments if measured angle decreases under positive commanded voltage.
