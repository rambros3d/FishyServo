# Design: BareFOC Flash-Once-and-Play Autonomy

## Context

See proposal.md - Why.
FishyServo runs on STM32H503KBU6 controlling four DC micro-gear motors (N20) with DRV8411A dual H-bridges and TMAG5273 magnetic angle sensors. Production units must operate deterministically immediately upon flashing, without requiring host scripts to configure addresses, null sensors, or establish safe starting setpoints.

## Goals / Non-Goals

**Goals:**
- Hydrate initial target position setpoints from measured shaft angles during boot before closed-loop regulation is activated.
- Synchronize runtime MultiBus register writes from `SHARED_QUAD` into `SHARED_CONTROL_TABLE` before flash save operations (`0x55` to register `0x64`).
- Ensure multi-tier address resolution (Tier 1 RAM Mailbox, Tier 2 NVM ControlTable, Tier 3 Silicon UID fallback).
- Auto-discover TMAG5273 across bus frequencies (20 kHz, 50 kHz, 100 kHz) and candidate addresses (0x35, 0x22, 0x78, 0x44) with graceful fallback to dummy sensors.
- Null ADC current sense baselines using 200-sample calibration during safe coast mode at boot.
- Enforce emergency overcurrent stall trip at > 600.0 mA within 2 ms loop iterations, entering high-impedance Coast mode.

**Non-Goals:**
- Modifying CAN-FD hardware or adding external EEPROM ICs.
- Modifying default baud rates (115200 baud) or fundamental SimpleFOC register mappings.

## Decisions

### Decision 1: Initial Angle Hydration in Bootloader/Startup Sequence
- **Choice**: Measure initial shaft angle via `sensor.get_sensor_angle()` and write this value to `motor.target`, `joint_targets[i]`, and `SHARED_QUAD.motors[i].target` during motor initialization before enabling closed loop.
- **Alternatives Considered**: Defaulting to 0.0 rad target setpoint. Rejected because commanding 0.0 rad on power-up causes violent rotation and potential mechanical damage to attached linkages.

### Decision 2: MultiBus to Control Table Synchronization
- **Choice**: During the 1 kHz motor loop and/or before servicing `CMD_SAVE_CONFIG`, synchronize motor PID gains, velocity limits, and voltage limits from `SHARED_QUAD` into `SHARED_CONTROL_TABLE` fields.
- **Alternatives Considered**: Dual-writing directly in the MultiBus I2C ISR/service task. Synchronizing prior to flash save is race-free, atomic under `SHARED_CONTROL_TABLE` lock, and ensures persistent sectors match live tuning state.

### Decision 3: Multi-Tier Deterministic Address Binding
- **Choice**: Address priority hierarchy:
  1. Tier 1: SoftResetMailbox (`0x2000_7FE0`) if magic token is valid.
  2. Tier 2: Persisted NVM ControlTable node ID (Bank 2 Sectors 14 & 15) if valid (0x08..=0x77).
  3. Tier 3: CRC8 or modulo hash of STM32H503 factory 96-bit silicon unique ID (`0x08FF_F800`) mapped into valid standard 7-bit I2C range (0x10..=0x70).
- **Alternatives Considered**: Hardcoded default address (0x40). Rejected because multiple servos on the same bus would collide on cold boot.

### Decision 4: Fast Overcurrent Disarm
- **Choice**: Low-pass filtered current readings sampled at 1 kHz per motor; if current magnitude exceeds 600.0 mA, immediately call `motor.disable()`, zero bridge PWM duty cycles, and clear `sq.motors[i].enabled`.
- **Alternatives Considered**: Relying on thermal shutdown in DRV8411A. Rejected because thermal trip takes hundreds of milliseconds and can burn N20 motor windings.

## Risks / Trade-offs

- [Risk] Uncalibrated sensor reads noisy angle on startup -> [Mitigation] Sensor reads multiple samples or verifies CRC/valid flags before seeding setpoint; if sensor read fails, system defaults to open-loop safe velocity mode.
- [Risk] Flash wear on repetitive `0x55` writes -> [Mitigation] Bank 2 ping-pong Sectors 14 & 15 cycle writes and check for payload dirty flags before triggering flash erase.
- [Risk] I2C target busy while flash write executes -> [Mitigation] Flash operations use non-blocking or critical-section bounded writes on STM32H5 separate bank (Bank 2) while execution continues in Bank 1.
