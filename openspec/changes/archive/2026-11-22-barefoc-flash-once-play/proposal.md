# Proposal

## Why

The FishyServo hardware platform requires true "flash once and play" autonomy for deployment in robotics and multi-axis actuator assemblies. In production, devices are programmed once during manufacturing and must subsequently boot up, discover attached magnetic angle sensors, zero current offsets, hold their mechanical resting position without violent set-zero jerks, synchronize in-flight MultiBus tuning parameters to non-volatile flash storage across power cycles, and protect hardware against overcurrent stalls without external host setup scripts.

## What Changes

- **Cold Boot Setpoint Hydration**: Update motor initialization so the target position setpoint hydrates from the measured shaft angle upon sensor lock, preventing violent mechanical jumps to 0.0 rad on power-up.
- **MultiBus to Control Table Synchronization**: Synchronize runtime MultiBus register writes (`SHARED_QUAD`) into `SHARED_CONTROL_TABLE` so that the `0x55` flash save command on register `0x64` commits live PID gains, limits, mechanical offsets, and node IDs to Bank 2 ping-pong Sectors 14 & 15.
- **Autonomous Multi-Tier Startup & Zeroing**: Formally specify the zero-configuration auto-discovery sequence for the TMAG5273 magnetic angle sensor, the 200-sample ADC current baseline nulling window, and deterministic address binding across Tier 1 (RAM Mailbox), Tier 2 (NVM), and Tier 3 (Silicon UID).
- **Overcurrent Emergency Stall Protection**: Specify immediate motor disarm and bridge shutdown into high-impedance Coast mode within 2 ms when motor current exceeds 600.0 mA.
- **Hardware Qualification Suite**: Add automated hardware verification scripts to rigorously test all five operational phases (safe boot, closed-loop regulation, bus telemetry, flash durability, and safety failsafes).

## Capabilities

### Modified Capabilities
- `n20-servo`: Adds requirements for hold-position startup, synchronized MultiBus NVM persistence, zero-configuration sensor discovery with fallback, and overcurrent stall protection.

## Impact

- Affected Code: `firmware/src/main.rs`, `firmware/src/n20_bin.rs`, `firmware/src/control_table.rs`.
- Host Tooling: `test-scripts/`, `host/scripts/verify_flash_once_play.py`.
- Breaking Changes: None. Default baud rates, register maps, and CAN/I2C protocols remain 100% backwards-compatible.
