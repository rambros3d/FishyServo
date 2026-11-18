## 1. Project Setup & RTT Integration

- [x] 1.1 Add self-contained SEGGER RTT core files (`SEGGER_RTT.h`, `SEGGER_RTT.c`, `SEGGER_RTT_printf.c`) to `firmware/i2c_scanner/`
- [x] 1.2 Add the `[env:i2c_scanner]` build target to `firmware/platformio.ini` with source filtering for `firmware/i2c_scanner`

## 2. Multi-Speed Scanner Firmware

- [x] 2.1 Implement `firmware/i2c_scanner/main.cpp` with I2C initialization on PB7 (`PIN_3D_SDA`) and PB6 (`PIN_3D_SCL`)
- [x] 2.2 Add internal pull-up enforcement logic using STM32 LL GPIO calls (`LL_GPIO_SetPinPull`) re-applied on every clock frequency switch
- [x] 2.3 Implement multi-speed address scan (0x03 to 0x77) across 10 kHz, 50 kHz, 100 kHz, and 400 kHz with TMAG5273 address decoding (0x35, 0x22, 0x78, 0x49)
- [x] 2.4 Format and stream the scan matrix and device summary to RTT buffer, with onboard LED status blink on each scan pass

## 3. Build Verification

- [x] 3.1 Build the `i2c_scanner` firmware environment with `pio run -e i2c_scanner` and confirm zero compilation errors

## 4. Flashing & Live Testing

- [x] 4.1 Flash the firmware to the custom board using `pio run -e i2c_scanner -t upload` (or pyOCD custom upload)
- [x] 4.2 Launch `pyocd rtt -t stm32h503kbux` to capture live RTT console output over SWD
- [x] 4.3 Verify TMAG5273 address ACK on the RTT monitor and document which I2C frequencies succeed with the internal pull-ups
