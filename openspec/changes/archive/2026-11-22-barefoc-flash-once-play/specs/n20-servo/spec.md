# Spec Delta

## ADDED Requirements

### Requirement: Cold Boot Setpoint Hydration
The system SHALL initialize and hydrate the motor position setpoint directly from the measured physical shaft angle upon sensor lock during system boot, preventing violent mechanical jerks or uncontrolled motion toward 0.0 rad.

#### Scenario: Angle setpoint hydration on startup
- **WHEN** the system completes sensor initialization and obtains a valid angle reading from the angle sensor
- **THEN** the initial motor target setpoint and shared target states are set to the measured physical shaft angle before closed-loop regulation is enabled

### Requirement: MultiBus to Control Table Synchronization
The system SHALL continuously synchronize runtime MultiBus register writes (`SHARED_QUAD`) into `SHARED_CONTROL_TABLE` so that the `0x55` flash save command on register `0x64` commits live PID gains, limits, mechanical offsets, and node IDs to Bank 2 ping-pong Sectors 14 & 15.

#### Scenario: Committing live tuning parameters to flash
- **WHEN** a host updates motor PID gains, velocity limits, or voltage limits over MultiBus and issues command `0x55` to register `0x64`
- **THEN** `SHARED_CONTROL_TABLE` is updated with the live motor parameters and persisted to Flash Bank 2 Sectors 14 & 15

### Requirement: Autonomous Multi-Tier Startup and Zeroing
The system SHALL execute an autonomous zero-configuration auto-discovery sequence for the TMAG5273 magnetic angle sensor across candidate bus frequencies and addresses, perform a 200-sample ADC current baseline nulling calibration window at boot, and deterministically bind the bus address across Tier 1 (RAM Mailbox), Tier 2 (NVM), and Tier 3 (Silicon UID).

#### Scenario: Autonomous boot discovery and calibration
- **WHEN** the system cold boots
- **THEN** the angle sensor bus is scanned and auto-configured with graceful dummy fallback, current sense baselines are zeroed over 200 samples, and the active node ID is resolved across Tier 1, Tier 2, and Tier 3

### Requirement: Overcurrent Emergency Stall Protection
The system SHALL immediately disarm the motor and shut down the bridge into high-impedance Coast mode within 2 ms when motor current exceeds 600.0 mA.

#### Scenario: Current limit trip
- **WHEN** measured filtered motor current exceeds 600.0 mA
- **THEN** the motor driver is disabled and bridge PWM outputs are zeroed within 2 ms to prevent hardware damage
