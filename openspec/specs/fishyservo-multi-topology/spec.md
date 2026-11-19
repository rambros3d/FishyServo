# fishyservo-multi-topology Specification

## Purpose
TBD - created by archiving change fishyservo-multi-topology-firmware. Update Purpose after archive.
## Requirements
### Requirement: Four-Channel Motor Bridge Control

FishyServo firmware SHALL configure and drive all 4 H-bridge circuits (Bridges A, B, C, D) using TIM1, TIM2, and TIM3 PWM channels with independent current sensing.

#### Scenario: Safe boot coast mode
- **WHEN** board boots
- **THEN** all 4 H-bridge outputs are held LOW in high-impedance coast mode.

### Requirement: Runtime Multi-Topology Execution

FishyServo firmware SHALL execute the control loop according to the configured `TopologyPreset` (4x 1-DOF, 2x 2-DOF, 1x 1-DOF + 1x 2-DOF, or 2x 1-DOF + 1x 2-DOF).

#### Scenario: Switching topology live over I2C
- **WHEN** host writes `1` to `TopologyMode` (0x78)
- **THEN** the firmware dynamically reconfigures control loops to 2x 2-DOF differential mode.

