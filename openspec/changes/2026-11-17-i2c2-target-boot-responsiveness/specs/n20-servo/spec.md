# n20-servo

## MODIFIED Requirements

### Requirement: Host bus available during sensor discovery

The firmware SHALL bring up the I2C2 Multibus target at 7-bit address 0x40 (PB3 SCL / PB4 SDA) before I2C1 sensor discovery begins, so a host probe is answered within roughly 100 ms of power-on or reset.

#### Scenario: host probe during discovery

- **WHEN** the firmware is still probing the I2C1 TMAG5273 candidate addresses
- **THEN** the I2C2 target at 0x40 is already configured and its task is spawned
- **AND** a host address probe receives an ACK without waiting for the full boot sequence

### Requirement: Bounded sensor discovery

The firmware SHALL probe only the TMAG5273 factory addresses (0x35, 0x22, 0x78, 0x44) and SHALL preserve the 20 kHz, 50 kHz, 100 kHz fallback order and the A1/A2/B1/B2/C1/C2/D1/D2 variant detection.

#### Scenario: sensor present

- **WHEN** a TMAG5273 ACKs at one of the candidate addresses
- **THEN** discovery stops at the first speed that identifies it
- **AND** `TMAG_DETECTED` / `has_sensor` semantics are unchanged

#### Scenario: no sensor

- **WHEN** no TMAG5273 ACKs at any candidate address at any speed
- **THEN** discovery finishes quickly and the firmware falls back to the dummy sensor