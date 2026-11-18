## ADDED Requirements

### Requirement: Multi-speed I2C scanning
The firmware SHALL scan standard 7-bit I2C addresses (0x03 to 0x77) across multiple clock frequencies including 10 kHz, 50 kHz, 100 kHz, and 400 kHz on pins PB7 (SDA) and PB6 (SCL).

#### Scenario: Successful device address detection
- **WHEN** a TMAG5273 sensor is connected to PB7/PB6 and powered at 3.3V
- **THEN** the scanner SHALL detect an ACK on the sensor's address (e.g. 0x35, 0x22, 0x78, or 0x49) and report the corresponding frequency and device identity

#### Scenario: Unresponsive or missing device
- **WHEN** an address does not acknowledge on a given frequency
- **THEN** the scanner SHALL record a NACK without hanging the microcontroller or locking the I2C bus

### Requirement: Internal pull-up activation
The firmware SHALL configure and maintain internal pull-up resistors on PB6 and PB7 in Alternate Function Open-Drain mode.

#### Scenario: Clock frequency change
- **WHEN** the scanner switches I2C clock frequency
- **THEN** the firmware SHALL re-assert the internal pull-up configuration on PB6 and PB7 after the clock reconfiguration

### Requirement: RTT debug streaming over SWD
The firmware SHALL stream human-readable scan tables and status messages via SEGGER RTT over the standard SWD interface (`SWDIO`/`SWCLK`).

#### Scenario: Real-time scan monitoring
- **WHEN** a host runs `pyocd rtt -t stm32h503kbux` with an ST-Link probe attached to the SWD header
- **THEN** the scanner output SHALL appear in the host terminal without requiring a separate physical UART connection
