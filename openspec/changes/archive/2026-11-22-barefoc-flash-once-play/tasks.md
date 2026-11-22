# Tasks

## 1. Specification and Design Validation

- [x] 1.1 Formulate and validate OpenSpec delta spec and design for barefoc-flash-once-play using `openspec validate barefoc-flash-once-play`

## 2. Firmware Implementation

- [x] 2.1 Implement Cold Boot Setpoint Hydration in `firmware/src/main.rs` to read measured shaft angle and seed initial target setpoints before closed-loop engagement
- [x] 2.2 Implement MultiBus to Control Table synchronization in `firmware/src/main.rs` before executing `0x55` flash save to persist live tuning parameters into Bank 2 sectors
- [x] 2.3 Implement autonomous multi-tier address resolution across RAM Mailbox (Tier 1), NVM ControlTable (Tier 2), and Silicon UID hash (Tier 3)
- [x] 2.4 Verify 200-sample ADC baseline nulling calibration and TMAG5273 multi-frequency auto-discovery sequence with dummy fallback
- [x] 2.5 Verify and harden overcurrent emergency stall protection (< 2 ms trip, high-impedance coast, > 600.0 mA threshold)

## 3. Firmware Compilation and Verification

- [x] 3.1 Run `cargo check` and `cargo build --release` on `firmware/` and verify zero errors and `#![no_std]` compliance

## 4. Documentation and Specification Synchronization

- [x] 4.1 Update live documentation in `barefoc.github.io/docs/` detailing the Flash Once and Play architecture, MultiBus-to-NVM sync, cold boot angle hydration, and safety protections
- [x] 4.2 Sync OpenSpec delta specs to main specs and archive change `barefoc-flash-once-play`
- [x] 4.3 Commit changes across `FishyServo` and `bareFOC` repositories with descriptive commit messages
