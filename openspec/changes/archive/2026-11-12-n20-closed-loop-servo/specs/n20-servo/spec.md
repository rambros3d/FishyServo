## ADDED Requirements

### Requirement: Safe Hardware Initialization and Coast State
The system SHALL initialize all DRV8411A H-bridge inputs in high-impedance Coast mode (`0/0`) during system boot to prevent uncontrolled motor movement or current draw.

#### Scenario: System boot state
- **WHEN** the microcontroller completes reset and peripheral initialization
- **THEN** all H-bridge input pins (Channels A, B, C, D) are driven LOW and PWM duty cycles are 0%

### Requirement: Channel D Closed-Loop Angle Control
The system SHALL drive the Channel D N20 motor using `DCDriver2PWM` and `DCMotor`, regulating position to a target angle setpoint using real-time feedback from the `Tmag5273Sensor`.

#### Scenario: Angle tracking response
- **WHEN** a target angle setpoint is commanded
- **THEN** `DCMotor` calculates position error through `P_angle`, regulates velocity through `PID_velocity`, and applies directional voltage to Channel D until target is reached

### Requirement: Integrated Current Mirror Load Telemetry
The system SHALL sample `ISENS_D` on `PA5` using `CurrentMirrorSense` to measure real-time motor current magnitude and detect load stalls.

#### Scenario: Current reading under load
- **WHEN** motor experiences external mechanical torque resistance
- **THEN** `CurrentMirrorSense` reports proportional current magnitude scaled by the 200 µA/A mirror ratio

### Requirement: Host I2C2 Multibus Target Interface
The system SHALL expose a SimpleFOC register map over `I2C2` at target address `0x40`, allowing external host controllers to read telemetry and update angle targets.

#### Scenario: Host target angle write
- **WHEN** host controller writes a 32-bit float angle setpoint to `RegId::Target` (0x02) over I2C2
- **THEN** the active motor target is updated to the new setpoint
