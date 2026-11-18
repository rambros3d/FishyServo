## Context

The custom STM32H503KBU6 board contains a TI TMAG5273 3D Hall-effect sensor connected to dedicated pins PB6 (I2C1_SCL / `PIN_3D_SCL`) and PB7 (I2C1_SDA / `PIN_3D_SDA`). Two status LEDs are connected to PA1 (Red, `LED_R`) and PA0 (Blue, `LED_B`). The LEDs are wired active-low (common anode to 3V3). The board does not expose a serial monitor UART interface, making visual LED feedback the primary debug interface.

## Goals / Non-Goals

**Goals:**
- Provide a clear, reliable test environment `[env:tmag_test]` in PlatformIO.
- Automatically probe and initialize the TMAG5273 over I2C1 using custom pins PB7/PB6.
- Map the absolute magnitude of the X-axis field to `LED_R` brightness and Y-axis field to `LED_B` brightness.
- Provide immediate visual error feedback (rapid alternating flash) if the sensor fails to respond or is absent.

**Non-Goals:**
- Closed-loop servo position control or motor driving (that will be handled in subsequent firmware changes).
- Z-axis angle calculation or CORDIC processing.
- Persistent configuration saving to flash/EEPROM.

## Decisions

1. **Use SparkFun TMAG5273 Arduino Library**:
   - *Rationale*: It is self-contained with no extra dependencies (unlike Adafruit which pulls in `Adafruit_BusIO` and `Adafruit_Sensor`), has a small memory footprint suitable for the 128KB Flash STM32H503, and natively accepts any `TwoWire` reference in `begin()`.
   - *Alternatives considered*: Adafruit TMAG5273 (heavier dependency graph), Direct bare-metal Wire register driver (more code maintenance).

2. **I2C Port Instantiation & Address Auto-Detection**:
   - *Rationale*: STM32duino allows creating `TwoWire Wire3D(PIN_3D_SDA, PIN_3D_SCL);`. At boot, the code performs an I2C scan over standard addresses (`0x35`, `0x22`, `0x78`, `0x49`).
   - *Alternatives considered*: Hardcoding `0x35` only, which could fail if a different TMAG5273 orderable variant (e.g. B1/B2) is assembled.

3. **Active-Low PWM Brightness Scaling**:
   - *Rationale*: `analogWrite(pin, 255 - brightness)` maps 0 mT to LED off (`PWM 255`) and high field strength (40 mT) to maximum brightness (`PWM 0`).

4. **Visual Error Trap**:
   - *Rationale*: With no serial port, an alternating red/blue flash at 100ms interval cleanly notifies the user of an I2C bus error or missing sensor.

## Risks / Trade-offs

- [I2C bus lockup on unpowered sensor] → Wire3D timeout configured; error trap triggered if `endTransmission()` fails.
- [Sensor saturation beyond 40 mT] → PWM calculation uses `constrain()` to clamp maximum brightness at 255.
