## ADDED Requirements

### Requirement: TMAG5273 Initialization and Address Probing
The firmware SHALL initialize communication with the TMAG5273 over I2C1 (PB7 for SDA, PB6 for SCL) and probe valid 7-bit addresses (0x35, 0x22, 0x78, 0x49) to detect the sensor.

#### Scenario: Sensor detected successfully
- **WHEN** the firmware starts up and a TMAG5273 responds on any probed I2C address
- **THEN** the firmware initializes the sensor in continuous measurement mode with X and Y channels enabled, flashes both LEDs briefly once to signal success, and enters the main reading loop.

#### Scenario: Sensor communication failure
- **WHEN** no TMAG5273 device responds on the probed I2C addresses during startup
- **THEN** the firmware SHALL trap into an error indicator mode that blinks LED_R and LED_B in a rapid alternating pattern indefinitely.

### Requirement: Magnetic Field Measurement
The firmware SHALL continuously read the magnetic flux density along the X and Y axes from the TMAG5273.

#### Scenario: Periodic measurement updates
- **WHEN** the main loop executes
- **THEN** the firmware polls the X and Y magnetic field readings from the sensor every loop interval (~20ms).

### Requirement: Visual Magnetic Field LED Feedback
The firmware SHALL map the magnetic field magnitude of the X and Y axes to the brightness of LED_R and LED_B respectively using active-low PWM.

#### Scenario: Low or zero magnetic field
- **WHEN** no significant magnetic field is present (|B| ≈ 0 mT)
- **THEN** the PWM duty cycle for LED_R and LED_B SHALL be set to near-zero brightness (PWM output near 255 due to active-low wiring).

#### Scenario: Strong magnetic field on X-axis
- **WHEN** a magnetic field is applied along the X-axis of the TMAG5273
- **THEN** the brightness of LED_R SHALL increase proportionally with |Bx| up to maximum brightness at full scale (~40 mT).

#### Scenario: Strong magnetic field on Y-axis
- **WHEN** a magnetic field is applied along the Y-axis of the TMAG5273
- **THEN** the brightness of LED_B SHALL increase proportionally with |By| up to maximum brightness at full scale (~40 mT).
