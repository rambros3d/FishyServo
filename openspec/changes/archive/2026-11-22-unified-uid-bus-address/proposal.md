# Proposal

## Why

In multidrop actuator arrays connected via MIPI I3C, I2C, CAN, or RS485, multiple FishyServo nodes share the same physical bus. When a host controller addresses a specific actuator node (e.g. Node ID `0x42`) for firmware upgrade, the node must maintain the exact same hardware identity (96-bit silicon UID, 48-bit MIPI I3C Provisional ID, 6-byte Katapult CAN UUID) and the exact same bus address whether executing the motion application (`FishyServo`) or the bootloader (`BareBoot`).

Without unified identity and dual-tier address synchronization, nodes either collide on default addresses or require flash wear and latency to persist runtime dynamic addresses across warm reboots.

## What Changes

- Update `FishyServo` boot sequence to read `SoftResetMailbox` placed at uninitialized SRAM (`0x2000_7FE0..0x2000_7FFF`) before reading flash sectors 14 and 15, binding the I2C2/I3C MultiBus target server directly to the active dynamic node address.
- Update `bootloader_write_hook` and handsfree reboot trigger (`0xF0` strobe / I3C `RSTACT`) to write `SoftResetMailbox` with the active dynamic node ID and CRC32 prior to `SCB::sys_reset()`.
- Populate `HardwareUid` and `I3C_EPIDR` in the ControlTable from STM32H503 `UID_BASE` registers.

## Capabilities

### Modified Capabilities
- `n20-servo`: Extend `Host I2C2 Multibus Target Interface` to read and write active dynamic node addresses via the `SoftResetMailbox` at `0x2000_7FE0`.

## Impact

- `firmware/src/main.rs`: Updates `main()` bootloader mailbox inspection and `bootloader_write_hook`.
- Upstream compatibility: Seamless handsfree OTA updates with [BareBoot](file:///home/sun/Filelink/Firmwares/bareBoot) on multidrop buses.
