# i2c-device-detection

## ADDED Requirements

### Requirement: Detect I2C devices on sensor bus (I2C1)

The firmware SHALL probe I2C1 (PB7 SDA / PB6 SCL) and report which device addresses ACK.

#### Scenario: new sensor detected

- **WHEN** the sensor bus is powered and the firmware is running
- **THEN** the scanner reports any ACKing addresses over RTT

### Requirement: RTT-only debug output

The scanner SHALL report results via SEGGER RTT, without requiring a UART console.

#### Scenario: RTT capture

- **WHEN** a probe is attached over SWD
- **THEN** scan results are readable via RTT channel 0
