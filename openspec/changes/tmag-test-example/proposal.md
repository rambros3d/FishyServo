## Why

The custom N20-Servo board integrates an on-board TI TMAG5273 3D Hall-effect magnetic sensor wired to STM32H503 I2C1 (PB6/PB7) and two status LEDs (LED_R on PA1, LED_B on PA0). A dedicated test firmware example is needed in `tmag_test` to verify sensor communication, validate magnetic field acquisition along the X and Y axes, and provide immediate visual feedback via PWM brightness on the LEDs.

## What Changes

- Add a new PlatformIO build environment `[env:tmag_test]` configuration with the SparkFun TMAG5273 Arduino library dependency.
- Create an interactive test firmware application in `tmag_test/main.cpp` that:
  - Initializes I2C1 on PB7 (SDA) and PB6 (SCL).
  - Probes for the TMAG5273 sensor (supporting common I2C addresses, defaulting to 0x35).
  - Enters an alternating rapid error blink pattern on both LEDs if the sensor is not found.
  - Configures the TMAG5273 for continuous measurement on the X and Y channels.
  - Continuously polls magnetic readings and scales |Bx| and |By| to active-low PWM brightness on LED_R (X-axis) and LED_B (Y-axis).

## Capabilities

### New Capabilities
- `tmag-test`: TMAG5273 I2C communication, magnetic field acquisition on X and Y axes, and responsive active-low LED PWM feedback.

### Modified Capabilities
<!-- None -->

## Impact

- `platformio.ini`: update `[env:tmag_test]` environment with library dependencies.
- `tmag_test/main.cpp`: new standalone test example.
- No impact on `blink_test` or existing board definitions.
