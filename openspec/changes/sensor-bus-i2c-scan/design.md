## Context

Current state: `firmware/i2c_scanner/main.cpp` scans I2C1 (PB7/PB6), reports via SEGGER RTT, with internal pull-ups. A new sensor was added to that bus. We need bring-up confirmation via RTT only. PB3/PB4 (I2C2) no longer in use for testing.

## Goals / Non-Goals

**Goals:**
- Detect all I2C devices (ACK) on PB6/PB7 across speeds.
- Identify known sensors by address.
- No serial console dependency (RTT only).

**Non-Goals:**
- Motor-control firmware.
- Driver library integration for the new sensor.
- Code on PB3/PB4.

## Decisions

- **Reuse existing scanner sketch** rather than new project — it already handles pull-ups and multi-speed scanning. Rationale: proven, minimal new code.
- **RTT only** as agreeing in exploration — avoids pin consumption.
- **Probe-rs primary, pyocd fallback** since WCH-Link available.
- Address table already includes 0x35/0x22/0x78/0x49 (TMAG variants). New sensor address must be added when known.

## Risks / Trade-offs
- [Unknown address] → add once sensor part number confirmed.
- [RTT throughput] → if verbose at all speeds, limit to 4 speeds still fine.
- [Bus shared with TMAG and new sensor] → Increased capacitive load; scan shows ACK truth but not signal integrity. Mitigation: assess single with known device.

## Migration Plan

Reuse `[env:i2c_scanner]` environment and build with `pio run -e i2c_scanner`. Flash via pyocd/probe-rs.

## Open Questions
- What is the I2C address of the newly added sensor?
