## 1. Dependencies and Build Configuration

- [x] 1.1 Update `firmware/Cargo.toml` with BareFOC path dependencies and `[profile.dev] panic = "abort"`
- [x] 1.2 Verify `cargo check` compiles successfully for `thumbv8m.main-none-eabihf`

## 2. Hardware Drivers & Setup

- [x] 2.1 Configure safe boot Coast mode on Channels A, B, C, D
- [x] 2.2 Configure `TIM1_CH2` (`PA9`) and `TIM3_CH3` (`PB0`) 25 kHz PWM and wrap in `DCDriver2PWM`
- [x] 2.3 Configure `I2C1` (`PB6`/`PB7`) with `Tmag5273Sensor`
- [x] 2.4 Configure `ADC1` (`PA5`) with `CurrentMirrorSense`

## 3. Closed-Loop Servo & Multibus Integration

- [x] 3.1 Initialize `DCMotor` in `MotionControlType::angle` with cascaded PID parameters
- [x] 3.2 Implement periodic 1 kHz motor control task
- [x] 3.3 Configure `I2C2` (`PB3`/`PB4`) target at address `0x40` running `barefoc-multibus` server
- [x] 3.4 Verify clean build with `cargo build --release`
