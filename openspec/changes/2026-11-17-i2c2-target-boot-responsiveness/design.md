## Context

The firmware runs on a single-core Cortex-M33 under one embassy executor. `I2c::new_blocking` and the blocking read/write calls are synchronous: while `main` is inside the I2C1 discovery loop the executor cannot poll any other task. The I2C2 target hardware ACKs its address as soon as `Stm32I2cTarget::new` runs, but a full host transaction is clock-stretched until the target task is polled.

## Decisions

### 1. Bring the target up before discovery

Move the I2C2 configuration, `Stm32I2cTarget::new`, and `spawner.spawn(i2c_target_task(...))` to just after `embassy_stm32::init` and the DBGMCU setup. This makes address 0x40 ACK within a few milliseconds of reset, well before the sensor scan. The `Peri` values for `p.I2C2`, `p.PB3`, and `p.PB4` are moved out of `p` before the later `p.I2C1` / `p.PB6` / `p.PB7` use; partial moves are allowed because `p` is never used as a whole.

### 2. Shrink discovery to the TMAG5273 candidates

The full 0x03..=0x77 sweep at three speeds is unnecessary: the TMAG5273 factory addresses are 0x35 (A1/A2), 0x22 (B1/B2), 0x78 (C1/C2), and 0x44 (D1/D2). Discovery probes only these four addresses, in the same 20/50/100 kHz order, and stops at the first speed that identifies a device. The variant decode and `TMAG_DETECTED` / `has_sensor` behaviour are unchanged.

### 3. Use a read-only address probe

`i2c1.blocking_write(addr, &[])` to a NACKing address waits for the configured timeout because the driver's NACK path expects an automatic STOP that software mode does not generate. A one-byte `blocking_read` NACKs immediately (AUTOEND STOP), so it is both faster and sufficient for the TMAG5273, which supports reads.

### 4. Yield around the blocking sections

A one-millisecond `Timer::after_millis(1).await` after spawning the target task lets it enter its service loop before discovery. A second yield after discovery lets it service any transaction that arrived while the executor was blocked.

## Risks / Trade-offs

- A host transaction that starts during discovery is clock-stretched until the post-scan yield. The candidate-only scan is short enough (single-digit milliseconds with a read-only probe) to stay inside the 100 ms budget.
- The full-bus diagnostic sweep is intentionally dropped. It was only useful for bring-up; production only needs the TMAG5273 candidates.
- The 500 ms ADC offset calibration remains blocking and is outside the scope of this change.