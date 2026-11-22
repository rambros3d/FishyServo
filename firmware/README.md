# FishyServo Firmware (Upstream Reference)

The firmware for the FishyServo platform has been upstreamed into the core [BareFOC](https://github.com/simplefoc/Arduino-FOC) ecosystem as an official reference example.

Canonical source location:
**`bareFOC/examples/fishyservo_stm32h5`**

This repository (`FishyServo`) is dedicated strictly to hardware design, CAD assets, schematics, PCB fabrication files, and production test scripts.

---

## Building and Flashing Upstream Firmware

### 1. Prerequisites

Ensure you have Rust and the Cortex-M33 target installed:

```sh
rustup target add thumbv8m.main-none-eabihf
```

### 2. Standalone Build (Direct Flash Origin 0x08000000)

```sh
cd /path/to/bareFOC/examples/fishyservo_stm32h5
cargo build --release
```

To flash via SWD using `probe-rs`:

```sh
cargo run --release
```

### 3. Katapult / BareBoot Relocated Build (Flash Origin 0x08002000)

When operating behind a resident Katapult or BareBoot bootloader:

```sh
cd /path/to/bareFOC/examples/fishyservo_stm32h5
cargo build --release --features bootloader
```

To extract a raw binary for USB/I2C bootloader flashing:

```sh
arm-none-eabi-objcopy -O binary \
  target/thumbv8m.main-none-eabihf/release/fishyservo_stm32h5 \
  target/thumbv8m.main-none-eabihf/release/fishyservo_stm32h5_bootloader.bin
```

---

## Features Implemented in Upstream Example

- **4-Axis DC Motor Drivers**: Full H-bridge control using dual TI DRV8411A drivers.
- **Multi-Topology Kinematics**: Switchable between independent 4x 1-DOF, differential 2-DOF, parallel 2-DOF, and coordinated quad drive.
- **External Angle Sensing**: TI TMAG5273 3D Hall-effect sensor over dedicated internal `I2C1` with multi-frequency address auto-discovery.
- **MultiBus I2C Host Target**: Standardized 384-byte Control Table at `0x40` on `I2C2` with telemetry, parameter persistence, and soft-reset bootloader handoff.
- **Low-Side Current Mirrors**: Real-time current sensing on all 4 channels via ADC1.
- **ASCII Commander Console**: Dual USART1 and SEGGER RTT command line interface.
