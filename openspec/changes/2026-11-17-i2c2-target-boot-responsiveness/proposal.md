## Why

The I2C2 Multibus target at 7-bit address 0x40 is created at the very end of `main`, after a full 0x03..=0x77 I2C1 scan at 20/50/100 kHz. The scan is synchronous and can spend seconds in blocking timeouts, so the single embassy executor never polls the target task and the host sees no ACK at 0x40 for roughly 7.2 s after power-on. Moving the `spawner.spawn(...)` earlier is not sufficient on its own because the blocking scan still prevents the executor from polling the task.

## What Changes

- Create the I2C2 target (`PB3`/`PB4`, address 0x40) and spawn `i2c_target_task` before I2C1 discovery.
- Yield once after spawning so the target task enters its service loop before discovery starts.
- Replace the full-range multi-speed scan with a probe of the four TMAG5273 factory addresses (0x35, 0x22, 0x78, 0x44), preserving the 20/50/100 kHz fallback order and the A1/A2/B1/B2/C1/C2/D1/D2 detection.
- Use a read-only address probe because a blocking write to a NACKing address waits for the full configured timeout.
- Yield again after discovery so the target task services any host transaction that arrived during the scan.
- Add RTT timing logs for "I2C2 target ready" and "Boot complete".

## Capabilities

### Modified Capabilities
- `n20-servo`: Host bus availability during sensor discovery.

## Impact

- Affected code: `firmware/src/main.rs`.
- Pin map, motor control logic, and `TMAG_DETECTED` / `has_sensor` semantics are unchanged.
- No new dependencies.