# Tasks

## 1. Mailbox Integration (`FishyServo/firmware`)

- [x] 1.1 Read `SoftResetMailbox` at `0x2000_7FE0` in `firmware/src/main.rs` and bind `I2C2` target server to valid dynamic address
- [x] 1.2 Update `bootloader_write_hook` in `firmware/src/main.rs` to write `SoftResetMailbox` with active address prior to calling `enter_bareboot`
- [x] 1.3 Verify firmware compiles cleanly under `#![no_std]` (`cargo check --target thumbv8m.main-none-eabihf --bin firmware`)

## 2. Hardware & Bus Verification

- [x] 2.1 Verify handsfree reboot handoff retains address across soft resets on hardware
- [x] 2.2 Verify cold boot falls back to `H5FlashStorage` sectors 14 and 15

## Workflow follow-up

- Sync delta specs to main specs (`openspec spec sync unified-uid-bus-address`).
- Archive the change after verification (`openspec archive unified-uid-bus-address`).
