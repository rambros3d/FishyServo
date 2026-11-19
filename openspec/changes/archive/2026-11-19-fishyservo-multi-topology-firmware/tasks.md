## 1. Quad H-Bridge Motor Drivers & ADC

- [x] 1.1 Configure TIM1, TIM2, and TIM3 PWM outputs for Bridges A, B, C, D in `FishyServo/firmware/src/main.rs`.
- [x] 1.2 Configure ADC1 channels for `ISENS_A`..`ISENS_D` (`PA2..PA5`).
- [x] 1.3 Instantiate 4 `DCMotor` instances.

## 2. Sensor Pool & ActuatorAssembly

- [x] 2.1 Set up `I2C1` sensor polling loop for onboard TMAG5273 and secondary external sensor.
- [x] 2.2 Instantiate `ActuatorAssembly<4, 4>` managing the 4 motors and 4 sensor channels.
- [x] 2.3 Wire 1 kHz control loop executing `assembly.step_all()`.

## 3. Comms & Transports

- [x] 3.1 Mount `multi_motor_profile` on `I2C2` (`0x40`) for MultiBus register access.
- [x] 3.2 Mount `Commander4Axis` on `USART1` / RTT.
- [x] 3.3 Verify live switching across all 4 combinations and test `#![no_std]` compilation.
