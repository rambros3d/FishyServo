## Context

The custom N20-Servo board houses an STM32H503KBU6 microcontroller and a TI TMAG5273 3D Hall-effect sensor connected on PB7 (`PIN_3D_SDA`) and PB6 (`PIN_3D_SCL`). The PCB design omitted external pull-up resistors for the I2C bus. Furthermore, the board does not feature a dedicated USB CDC or UART serial monitor port.

To verify sensor hardware connectivity without soldering rework or risking SWD lockout:
- The STM32H503 internal GPIO pull-ups (~40 kΩ) will be enabled in Alternate Function Open-Drain mode.
- I2C speeds from 10 kHz to 400 kHz will be evaluated to identify reliable operating limits.
- Debug and diagnostic text will be streamed over SWD via SEGGER RTT, captured on the PC host using `pyocd rtt` with an ST-Link probe.

## Goals / Non-Goals

**Goals:**
- Implement an automated multi-speed I2C scanner firmware in `firmware/i2c_scanner/` targeting PB7 (SDA) and PB6 (SCL).
- Re-assert internal pull-ups on PB6 and PB7 across every speed re-configuration.
- Integrate minimal SEGGER RTT sources to provide `rtt_printf` / stream output through the existing SWD header.
- Add an `[env:i2c_scanner]` configuration in `firmware/platformio.ini`.
- Document exact flashing and live testing commands with `pyocd rtt`.

**Non-Goals:**
- Full TMAG5273 continuous measurement or angle calculation (handled in `tmag_test`).
- Reconfiguring PA13/PA14 to hardware UART (which would disrupt SWD and prevent flashing).
- External bus scanning on PB3/PB4 (`PIN_SCL`/`PIN_SDA`).

## Decisions

1. **SEGGER RTT over SWD instead of hardware UART**:
   - *Rationale*: Reconfiguring PA13/PA14 to USART1 disables SWD and requires connect-under-reset to reflash. RTT operates directly through target RAM over standard SWD without CPU halt or pin remapping.
   - *Alternatives considered*: Physical UART on PA13/PA14 (risk of bricking/lockout), LED blink-code Morse (slow and tedious to decode).

2. **Speed Array: 10 kHz, 50 kHz, 100 kHz, 400 kHz**:
   - *Rationale*: Given the weak ~40 kΩ internal pull-ups and typical trace capacitance (~10-20 pF), 100 kHz and 400 kHz may have marginal rise times. Adding 10 kHz and 50 kHz provides a guaranteed baseline to isolate wiring/soldering defects from RC rise-time limits.
   - *Alternatives considered*: Standard 100k/400k/1M (1M is impossible with 40 kΩ pull-ups).

3. **Explicit GPIO Register Pull-up Persistence**:
   - *Rationale*: STM32duino `Wire.begin()` and `Wire.setClock()` re-apply `GPIO_NOPULL`. We must invoke `LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP)` and `LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP)` after every clock speed change.

4. **Self-Contained RTT Implementation**:
   - *Rationale*: Including lightweight `SEGGER_RTT.c` and `SEGGER_RTT.h` directly in `firmware/i2c_scanner/` avoids external library dependencies and works seamlessly with STM32duino and GCC ARM.

## Risks / Trade-offs

- [400 kHz fails due to RC rise time] → Expected; 10 kHz and 50 kHz verify device ACK and silicon presence.
- [pyOCD RTT terminal attach timing] → Scanner runs in a periodic loop (every 3 seconds), so late attachment by `pyocd rtt` will not miss scan results.
- [Noise coupling on high-impedance 40 kΩ pull-up] → Keep motor drivers inactive during scan; scanner strictly probes I2C without pulsing H-bridge outputs.
