## 1. Preparation

- [ ] 1.1 Confirm I2C address of newly added sensor and document in proposal/design
- [x] 1.2 Review internal pull-ups on PB6/PB7 in hardware (already added) and note in docs

## 2. Build/flash

- [x] 2.1 Build `i2c_scanner` env with `pio run -e i2c_scanner`
- [x] 2.2 Flash with probe-rs (pyocd fallback): `pyocd flash ...` or probe-rs
- [x] 2.3 Open RTT console and run scan (no ACKs)

## 3. Verification

- [x] 3.1 Verify TMAG addresses ACK if present (none)
- [x] 3.2 Verify new sensor (not detected, no ACK)
- [x] 3.3 Document result (bus idle high, zero ACKs)
