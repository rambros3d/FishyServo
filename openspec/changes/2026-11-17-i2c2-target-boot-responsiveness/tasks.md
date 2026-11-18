## 1. Implementation

- [x] 1.1 Create I2C2 target and spawn `i2c_target_task` before I2C1 discovery
- [x] 1.2 Yield once after spawning so the target task starts
- [x] 1.3 Narrow discovery to 0x35 / 0x22 / 0x78 / 0x44 with 20/50/100 kHz fallback
- [x] 1.4 Use read-only address probe
- [x] 1.5 Yield after discovery
- [x] 1.6 Add RTT timing logs

## 2. Build

- [x] 2.1 `cargo build --release` in `firmware/` (thumbv8m.main-none-eabihf)

## 3. Verification

- [ ] 3.1 Measure reset-to-0x40-ACK on hardware (probe warm-boot path was unreliable)
- [ ] 3.2 Confirm TMAG5273 detection on I2C1 (A2 at 0x35) after the change
- [ ] 3.3 Confirm fallback to dummy sensor when no TMAG is present