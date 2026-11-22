# Design

## Context

See `proposal.md` for motivation. `FishyServo` is an Embassy-based `#![no_std]` firmware running on the STM32H503KBU6. It communicates with the host over `I2C2` (PB3/PB4) running `barefoc-multibus`. Flash storage resides in Bank 2 Sectors 14 and 15 (`0x0801_C000..0x0802_0000`). Uninitialized SRAM at `0x2000_7FE0..0x2000_7FFF` survives soft resets.

## Goals / Non-Goals

**Goals:**
- Read `SoftResetMailbox` on boot and bind `I2C2` target server to `dynamic_node_id` if CRC32 is valid.
- Write `SoftResetMailbox` with active dynamic address when receiving in-band reboot strobe `0xF0` before triggering reset.
- Expose silicon UID and 48-bit MIPI I3C Provisional ID matching BareBoot.

**Non-Goals:**
- Modifying PID gains, PWM generation, or motor topology.

## Decisions

### Decision 1: Query SRAM Mailbox before Flash Storage in `main()`
- *Rationale*: If a warm reboot into FishyServo occurred after dynamic address assignment, FishyServo must immediately use the assigned address without requiring a flash write.
- *Alternatives Considered*: Writing address to flash on every dynamic assignment. Rejected to prevent flash cell wear.

### Decision 2: Write Mailbox in `bootloader_write_hook`
- *Rationale*: When the host writes single-byte `0xF0`, `bootloader_write_hook` writes the active address into the mailbox before calling `enter_bareboot`.

## Risks / Trade-offs

- [Mailbox CRC32 invalid on cold boot] -> Mitigation: System falls back to `H5FlashStorage` ping-pong sectors 14 and 15.
