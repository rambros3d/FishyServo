## 1. Environment & Dependencies Setup

- [x] 1.1 Configure `[env:tmag_test]` in `platformio.ini` with SparkFun TMAG5273 library dependency

## 2. Sensor & LED Implementation

- [x] 2.1 Implement I2C initialization on PB7/PB6 and address auto-probing in `tmag_test/main.cpp`
- [x] 2.2 Implement error visual trap on LEDs when sensor is not found
- [x] 2.3 Implement sensor configuration for continuous X and Y measurement
- [x] 2.4 Implement magnetic field magnitude scaling to active-low PWM for LED_R and LED_B

## 3. Verification & Build

- [x] 3.1 Build firmware with `pio run -e tmag_test` and verify successful compilation
