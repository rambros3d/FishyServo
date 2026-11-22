# Spec Delta

## MODIFIED Requirements

### Requirement: Host I2C2 Multibus Target Interface
The system SHALL expose a SimpleFOC register map over `I2C2` at the active dynamic address synchronized with the bootloader via `SoftResetMailbox` or persistent flash storage, allowing external host controllers to read telemetry, update angle targets, and command handsfree bootloader entry.

#### Scenario: Host target angle write
- **WHEN** host controller writes a 32-bit float angle setpoint to `RegId::Target` (0x02) over I2C2
- **THEN** the active motor target is updated to the new setpoint
