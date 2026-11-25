#!/usr/bin/env python3
"""
Full autonomous motor test for FishyServo with connected output joint over MIPI I3C:
  Phase A: Open-loop-equivalent gentle sweep & TMAG5273 position tracking to verify hard-stop limits.
  Phase B: Closed-loop position mode (Kp=16.0) bidirectional step tracking within safe interior limits.
"""

import struct
import sys
import time
import math

sys.path.insert(0, "/home/sun/Filelink/Firmwares/AgentLA-rust/sdk")
from agentla.i3c import AgentLaI3c, I3cException

TARGET_ADDR = 0x40
I3C_FREQ_KHZ = 1000

REG_STATUS = 0x00
REG_TARGET = 0x01
REG_ENABLE = 0x04
REG_ANGLE = 0x09
REG_VELOCITY = 0x11
REG_CURRENT_DC = 0x28
REG_ANG_PID_P = 0x36

HOMING_P_ANGLE = 5.5
TRACKING_P_ANGLE = 16.0
OVERCURRENT_ABORT_MA = 450.0  # Firmware trips at > 600 mA

STEP_RAD = 0.05
STEP_DT = 0.04
MAX_FOLLOW_ERR = 0.60         # caps drive at ~3.3 V during homing
STALL_FOLLOW_ERR = 0.40       # >= 2.2 V push required to count as stall
STALL_MOVE_EPS = 0.012
STALL_CONFIRM_COUNT = 5       # 200 ms no-progress confirmation
MAX_SWEEP_STEPS = 250


def read_u8(c: AgentLaI3c, reg: int) -> int:
    return c.private_write_read(TARGET_ADDR, bytes([0x00, reg]), 1)[0]


def read_f32(c: AgentLaI3c, reg: int) -> float:
    raw = c.private_write_read(TARGET_ADDR, bytes([0x00, reg]), 4)
    return struct.unpack("<f", raw)[0]


def write_u8(c: AgentLaI3c, reg: int, val: int) -> None:
    c.private_write(TARGET_ADDR, bytes([0x00, reg, val & 0xFF]))


def write_f32(c: AgentLaI3c, reg: int, val: float) -> None:
    c.private_write(TARGET_ADDR, bytes([0x00, reg]) + struct.pack("<f", val))


def ensure_connected(c: AgentLaI3c) -> None:
    try:
        if read_u8(c, REG_STATUS) == 4:
            return
    except I3cException:
        pass
    try:
        c.rstdaa()
    except I3cException:
        pass
    c.entdaa(TARGET_ADDR)
    st = read_u8(c, REG_STATUS)
    if st != 4:
        raise RuntimeError(f"Expected FishyServo STATUS=4, got {st}")


def safe_read_angle(c: AgentLaI3c, fallback: float) -> float:
    a = read_f32(c, REG_ANGLE)
    if not math.isfinite(a) or abs(a) < 1e-4:
        return fallback
    return a


def unstick_if_wedged(c: AgentLaI3c, angle0: float) -> float:
    """If resting wedged near + hard stop (> -3.45 rad), gently pulse reverse to free it."""
    if angle0 <= -3.45:
        return angle0
    print(f"\n[UNSTICK] Joint at {angle0:+.4f} rad (near + hard stop); applying brief reverse pulse...")
    write_f32(c, REG_ANG_PID_P, 9.0)
    write_f32(c, REG_TARGET, angle0 - 0.50)  # ~4.5 V reverse pulse
    write_u8(c, REG_ENABLE, 1)
    a = angle0
    for _ in range(10):
        time.sleep(0.03)
        a = safe_read_angle(c, a)
        cur = read_f32(c, REG_CURRENT_DC)
        if cur > OVERCURRENT_ABORT_MA:
            write_u8(c, REG_ENABLE, 0)
            raise RuntimeError(f"Overcurrent during unstick: {cur:.1f} mA")
        if a < angle0 - 0.15:
            break
    write_u8(c, REG_ENABLE, 0)
    time.sleep(0.1)
    a = safe_read_angle(c, a)
    print(f"[UNSTICK] Post-pulse angle: {a:+.4f} rad ({math.degrees(a):+.2f} deg)")
    return a


def find_limit(c: AgentLaI3c, direction: float, start_angle: float, label: str):
    """Sweep gently in `direction` (+1.0 or -1.0) until mechanical hard stop is detected."""
    print(f"\n--- Phase A Homing {label} (dir={direction:+.0f}) from {start_angle:+.4f} rad ---")
    target = start_angle
    write_f32(c, REG_TARGET, target)
    write_u8(c, REG_ENABLE, 1)
    time.sleep(0.05)

    anchor_angle = safe_read_angle(c, start_angle)
    last_angle = anchor_angle
    stall_count = 0
    peak_cur_ma = 0.0

    for step in range(1, MAX_SWEEP_STEPS + 1):
        angle_now = safe_read_angle(c, last_angle)
        desired_next = target + direction * STEP_RAD
        if direction > 0:
            target = max(target, min(desired_next, angle_now + MAX_FOLLOW_ERR))
        else:
            target = min(target, max(desired_next, angle_now - MAX_FOLLOW_ERR))

        write_f32(c, REG_TARGET, target)
        time.sleep(STEP_DT)

        st = read_u8(c, REG_STATUS)
        angle = safe_read_angle(c, last_angle)
        last_angle = angle
        vel = read_f32(c, REG_VELOCITY)
        cur_ma = read_f32(c, REG_CURRENT_DC)
        peak_cur_ma = max(peak_cur_ma, cur_ma)

        if st != 4:
            write_u8(c, REG_ENABLE, 0)
            raise RuntimeError(f"Controller status {st} during {label} homing")
        if cur_ma > OVERCURRENT_ABORT_MA:
            write_u8(c, REG_ENABLE, 0)
            raise RuntimeError(f"Host safety abort: I_DC={cur_ma:.1f} mA > {OVERCURRENT_ABORT_MA} mA")

        follow_err = abs(target - angle)
        moved = abs(angle - anchor_angle)

        if moved < STALL_MOVE_EPS:
            if follow_err >= STALL_FOLLOW_ERR:
                stall_count += 1
        else:
            anchor_angle = angle
            stall_count = 0

        if step % 5 == 0 or stall_count > 0:
            print(
                f"  step {step:3d}: target={target:+.4f} angle={angle:+.4f} "
                f"vel={vel:+.3f} I_DC={cur_ma:5.1f} mA stall={stall_count}"
            )

        if stall_count >= STALL_CONFIRM_COUNT:
            limit_angle = angle
            write_f32(c, REG_TARGET, limit_angle - direction * 0.10)
            time.sleep(0.15)
            print(
                f"  [STOP] {label} limit at {limit_angle:+.4f} rad "
                f"({math.degrees(limit_angle):+.2f} deg), peak I_DC={peak_cur_ma:.1f} mA"
            )
            return limit_angle, peak_cur_ma

    write_u8(c, REG_ENABLE, 0)
    raise RuntimeError(f"Did not hit {label} hard stop within {MAX_SWEEP_STEPS} steps")


def move_and_verify(c: AgentLaI3c, setpoint: float, label: str, settle_s: float = 1.5):
    print(f"\n--- Closed-Loop Move -> {label}: {setpoint:+.4f} rad ({math.degrees(setpoint):+.2f} deg) ---")
    write_f32(c, REG_TARGET, setpoint)
    peak_cur_ma = 0.0
    max_vel = 0.0
    t0 = time.monotonic()
    last_a = safe_read_angle(c, setpoint)
    settled_at = None

    while time.monotonic() - t0 < settle_s:
        st = read_u8(c, REG_STATUS)
        angle = safe_read_angle(c, last_a)
        last_a = angle
        vel = read_f32(c, REG_VELOCITY)
        cur_ma = read_f32(c, REG_CURRENT_DC)
        peak_cur_ma = max(peak_cur_ma, cur_ma)
        max_vel = max(max_vel, abs(vel))
        if st != 4:
            write_u8(c, REG_ENABLE, 0)
            raise RuntimeError(f"Controller status {st} during move to {label}")
        if cur_ma > OVERCURRENT_ABORT_MA:
            write_u8(c, REG_ENABLE, 0)
            raise RuntimeError(f"Host safety abort during move: I_DC={cur_ma:.1f} mA")
        if abs(setpoint - angle) < 0.03 and abs(vel) < 0.2 and settled_at is None:
            settled_at = time.monotonic() - t0
        time.sleep(0.03)

    final_angle = safe_read_angle(c, last_a)
    final_vel = read_f32(c, REG_VELOCITY)
    final_cur = read_f32(c, REG_CURRENT_DC)
    err = setpoint - final_angle
    settle_str = f"{settled_at:.2f}s" if settled_at is not None else f">{settle_s:.1f}s"
    print(
        f"  [SETTLED {settle_str}] angle={final_angle:+.4f} rad ({math.degrees(final_angle):+.2f} deg) | "
        f"err={err:+.4f} rad ({math.degrees(err):+.2f} deg) | "
        f"vel={final_vel:+.3f} rad/s | peak_I={peak_cur_ma:.1f} mA | end_I={final_cur:.1f} mA"
    )
    return final_angle, err, peak_cur_ma, max_vel, settle_str


def main():
    c = AgentLaI3c()
    c.connect()
    c.set_inverted(False)
    c.set_bus_speed(I3C_FREQ_KHZ)

    try:
        ensure_connected(c)
        write_u8(c, REG_ENABLE, 0)
        time.sleep(0.05)

        angle0 = read_f32(c, REG_ANGLE)
        cur0 = read_f32(c, REG_CURRENT_DC)
        print(f"Initial state: STATUS=4, Angle={angle0:+.4f} rad ({math.degrees(angle0):+.2f} deg), I_DC={cur0:.1f} mA")

        angle0 = unstick_if_wedged(c, angle0)

        # Phase A: Open-loop-equivalent gentle sweep & limit discovery
        write_f32(c, REG_ANG_PID_P, HOMING_P_ANGLE)
        min_limit, min_cur = find_limit(c, -1.0, angle0, "REVERSE (-)")
        cur_angle = safe_read_angle(c, min_limit)
        max_limit, max_cur = find_limit(c, +1.0, cur_angle, "FORWARD (+)")

        span = max_limit - min_limit
        center = 0.5 * (min_limit + max_limit)
        print("\n================ PHASE A: HOMING SUMMARY ================")
        print(f"  Min limit (reverse): {min_limit:+.4f} rad ({math.degrees(min_limit):+.2f} deg), peak I={min_cur:.1f} mA")
        print(f"  Max limit (forward): {max_limit:+.4f} rad ({math.degrees(max_limit):+.2f} deg), peak I={max_cur:.1f} mA")
        print(f"  Total travel span  : {span:.4f} rad ({math.degrees(span):.2f} deg)")
        print(f"  Center position    : {center:+.4f} rad ({math.degrees(center):+.2f} deg)")
        print("=========================================================")

        if span < 0.5:
            raise RuntimeError(f"Discovered travel span too small ({span:.4f} rad); check joint or limits")

        home_only = "--home-only" in sys.argv[1:]
        write_f32(c, REG_ANG_PID_P, TRACKING_P_ANGLE)

        if home_only:
            print(f"\n================ CENTERING JOINT (Kp={TRACKING_P_ANGLE}) ================")
            move_and_verify(c, center, "Home Center", settle_s=2.0)
            write_u8(c, REG_ENABLE, 0)
            print("\nAutomatic homing complete. Motor disabled cleanly (REG_ENABLE=0).")
            return

        # Phase B: Closed-loop position tracking across safe interior range
        print(f"\n================ PHASE B: CLOSED-LOOP TESTS (Kp={TRACKING_P_ANGLE}) ================")

        targets = [
            ("Center", center),
            ("Interior +25%", center + 0.25 * span),
            ("Interior -25%", center - 0.25 * span),
            ("Interior +35%", center + 0.35 * span),
            ("Interior -35%", center - 0.35 * span),
            ("Return Center", center),
        ]
        results = []
        for name, sp in targets:
            final_a, err, pk_i, pk_v, st_str = move_and_verify(c, sp, name)
            results.append((name, sp, final_a, err, pk_i, pk_v, st_str))

        write_u8(c, REG_ENABLE, 0)
        print("\nMotor disabled cleanly (REG_ENABLE=0).")
        print("\nClosed-Loop Tracking Summary:")
        for name, sp, fa, err, pk_i, pk_v, st_str in results:
            print(
                f"  {name:14s}: target={sp:+.4f}, actual={fa:+.4f}, "
                f"err={err:+.4f} rad ({math.degrees(err):+.2f} deg), "
                f"settle={st_str}, peak_I={pk_i:.1f} mA, max_vel={pk_v:.2f} rad/s"
            )

    finally:
        try:
            write_u8(c, REG_ENABLE, 0)
        except Exception:
            pass
        c.close()


if __name__ == "__main__":
    main()
