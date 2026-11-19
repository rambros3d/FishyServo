## Why

FishyServo features dual DRV8411 dual H-bridges (driving 4 motor channels) and dual I2C buses (I2C2 host comms, I2C1 sensor bus). Currently, the firmware runs a single DC motor on Bridge D with hardcoded registers. Upgrading the firmware to wire all 4 motor bridges and I2C1 sensors into `ActuatorAssembly` enables live, zero-reflash switching across 4x 1-DOF, 2x 2-DOF, 1x 1-DOF + 1x 2-DOF, and 2x 1-DOF + 1x 2-DOF over MultiBus and Commander/Fibre.

## What Changes

- In `FishyServo/firmware/src/main.rs`:
  - Initialize all 4 DRV8411 H-bridges: Bridge A (`PA15`/`PB5`), Bridge B (`PA8`/`PB15`), Bridge C (`PA6`/`PA7`), Bridge D (`PA9`/`PB0`).
  - Wire Current Sense ADC channels: `PA2` (ISENS_A), `PA3` (ISENS_B), `PA4` (ISENS_C), `PA5` (ISENS_D).
  - Initialize `I2C1` sensor polling (onboard TMAG5273 + secondary external sensor) delivering a 4-channel sensor pool.
  - Instantiate `ActuatorAssembly<4, 4>` managing the 4 motors and sensor mappings.
  - Mount `multi_motor_profile` on `I2C2` (`0x40`) for MultiBus register control.
  - Mount `Commander` on `USART1` / RTT.
  - Wire `save_to_flash` and `load_from_flash` restoring topology on boot.

## Capabilities

### New Capabilities
- `fishyservo-multi-topology`: Four-motor driver and multi-topology kinematics firmware on FishyServo.

## Impact

- `FishyServo`: `firmware/src/main.rs`.
