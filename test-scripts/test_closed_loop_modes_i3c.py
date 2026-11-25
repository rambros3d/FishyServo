#!/usr/bin/env python3
"""
Verification of aligned MultiBus registers, SafetySupervisor soft limits,
and all closed-loop modes (Angle, Velocity, Torque/Voltage) on FishyServo over MIPI I3C.
"""

import math
import struct
import sys
import time

sys.path.insert(0, "/home/sun/Filelink/Firmwares/AgentLA-rust/sdk")
from agentla.i3c import AgentLaI3c, I3cException

TARGET_ADDR = 0x40
I3C_FREQ_KHZ = 1000

# 4-byte aligned MultiBus RegisterId offsets (RegisterId::ALIGNED_OFFSETS)
REG_STATUS = 0x000
REG_TARGET = 0x004
REG_ENABLE = 0x00C
REG_CONTROL_MODE = 0x010
REG_ANGLE = 0x01C
REG_VELOCITY = 0x028
REG_CURRENT_DC = 0x088
REG_ANG_PID_P = 0x0A4
REG_VOLTAGE_LIMIT = 0x0EC
REG_CURRENT_LIMIT = 0x0F0
REG_VELOCITY_LIMIT = 0x0F4
REG_MIN_POS_LIMIT = 0x108
REG_MAX_POS_LIMIT = 0x10C
REG_STALL_TIMEOUT_MS = 0x110
REG_HW_ERR_STATUS = 0x114
REG_BUS_VOLTAGE = 0x118
REG_TEMPERATURE = 0x11C

# Soft limits inside discovered mechanical stops ([-4.91, -3.26] rad)
SOFT_MIN = -4.60
SOFT_MAX = -3.60
SAFE_MIN = -4.55
SAFE_MAX = -3.65
CENTER = -4.08
OVERCURRENT_ABORT_MA = 450.0


def _retry(fn):
    for attempt in range(4):
        try:
            return fn()
        except I3cException:
            if attempt == 3:
                raise
            time.sleep(0.003)


def read_u8(c: AgentLaI3c, reg: int) -> int:
    return _retry(lambda: c.private_write_read(TARGET_ADDR, struct.pack(">H", reg), 1)[0])


def read_u16(c: AgentLaI3c, reg: int) -> int:
    return _retry(lambda: struct.unpack("<H", c.private_write_read(TARGET_ADDR, struct.pack(">H", reg), 2))[0])


def read_f32(c: AgentLaI3c, reg: int) -> float:
    return _retry(lambda: struct.unpack("<f", c.private_write_read(TARGET_ADDR, struct.pack(">H", reg), 4))[0])


def write_u8(c: AgentLaI3c, reg: int, val: int) -> None:
    _retry(lambda: c.private_write(TARGET_ADDR, struct.pack(">H", reg) + bytes([val & 0xFF])))


def write_f32(c: AgentLaI3c, reg: int, val: float) -> None:
    _retry(lambda: c.private_write(TARGET_ADDR, struct.pack(">H", reg) + struct.pack("<f", val)))


def safe_angle(c: AgentLaI3c, fallback: float) -> float:
    a = read_f32(c, REG_ANGLE)
    return a if math.isfinite(a) and abs(a) > 1e-4 else fallback


def check_safety(c: AgentLaI3c, last_a: float):
    st = read_u8(c, REG_STATUS)
    a = safe_angle(c, last_a)
    v = read_f32(c, REG_VELOCITY)
    cur = read_f32(c, REG_CURRENT_DC)
    if st != 4:
        write_u8(c, REG_ENABLE, 0)
        raise RuntimeError(f"Controller fault: STATUS={st}")
    if cur > OVERCURRENT_ABORT_MA:
        write_u8(c, REG_ENABLE, 0)
        raise RuntimeError(f"Overcurrent abort: I_DC={cur:.1f} mA")
    return a, v, cur


def main():
    c = AgentLaI3c()
    c.connect()
    c.set_inverted(False)
    c.set_bus_speed(I3C_FREQ_KHZ)

    try:
        try:
            st = read_u8(c, REG_STATUS)
        except I3cException:
            try:
                c.rstdaa()
            except I3cException:
                pass
            c.entdaa(TARGET_ADDR)
            st = read_u8(c, REG_STATUS)
        assert st == 4, f"Expected STATUS=4, got {st}"
        write_u8(c, REG_ENABLE, 0)

        # 1. Verify aligned register R/W
        print("=== 1. Aligned MultiBus Register R/W ===")
        write_f32(c, REG_VOLTAGE_LIMIT, 4.0)
        write_f32(c, REG_CURRENT_LIMIT, 0.5)
        write_f32(c, REG_VELOCITY_LIMIT, 15.0)
        write_f32(c, REG_STALL_TIMEOUT_MS, 400.0)
        assert abs(read_f32(c, REG_VOLTAGE_LIMIT) - 4.0) < 1e-3
        for m in (0, 3, 2, 0):
            write_u8(c, REG_CONTROL_MODE, m)
            assert read_u8(c, REG_CONTROL_MODE) == m
        last_a = safe_angle(c, CENTER)
        print(
            f"Aligned R/W OK: Vlim={read_f32(c, REG_VOLTAGE_LIMIT):.2f} V, "
            f"Vbus={read_f32(c, REG_BUS_VOLTAGE):.2f} V, Start={last_a:+.4f} rad"
        )

        # 2. Pre-position to CENTER before enabling soft limits
        print("\n=== 2. Pre-position & SafetySupervisor Soft-Limit Clamp Test ===")
        write_u8(c, REG_CONTROL_MODE, 0)
        write_f32(c, REG_ANG_PID_P, 8.0)
        write_f32(c, REG_TARGET, last_a)
        write_u8(c, REG_ENABLE, 1)
        cmd = last_a
        while abs(CENTER - cmd) > 0.06:
            cmd += 0.06 if CENTER > cmd else -0.06
            write_f32(c, REG_TARGET, cmd)
            last_a, _, _ = check_safety(c, last_a)
            time.sleep(0.04)
        write_f32(c, REG_TARGET, CENTER)
        time.sleep(0.6)
        last_a, _, _ = check_safety(c, last_a)
        print(f"  Pre-positioned at {last_a:+.4f} rad, Err=0x{read_u16(c, REG_HW_ERR_STATUS):X}")

        # Now apply soft limits [-4.60, -3.60] and test clamping
        write_f32(c, REG_MIN_POS_LIMIT, SOFT_MIN)
        write_f32(c, REG_MAX_POS_LIMIT, SOFT_MAX)
        assert abs(read_f32(c, REG_MIN_POS_LIMIT) - SOFT_MIN) < 1e-3
        assert abs(read_f32(c, REG_MAX_POS_LIMIT) - SOFT_MAX) < 1e-3
        write_f32(c, REG_ANG_PID_P, 10.0)

        # Command -2.50 (beyond SOFT_MAX=-3.60, hard stop at -3.26)
        write_f32(c, REG_TARGET, -2.50)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.2:
            last_a, v, cur = check_safety(c, last_a)
            assert last_a <= -3.35, f"Escaped past soft max! angle={last_a:.4f}"
            time.sleep(0.03)
        print(f"  Cmd -2.50 -> clamped angle={last_a:+.4f} rad, Err=0x{read_u16(c, REG_HW_ERR_STATUS):X}")

        # Command -5.50 (beyond SOFT_MIN=-4.60, hard stop at -4.91)
        write_f32(c, REG_TARGET, -5.50)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.2:
            last_a, v, cur = check_safety(c, last_a)
            assert last_a >= -4.85, f"Escaped past soft min! angle={last_a:.4f}"
            time.sleep(0.03)
        print(f"  Cmd -5.50 -> clamped angle={last_a:+.4f} rad, Err=0x{read_u16(c, REG_HW_ERR_STATUS):X}")

        # 3. Mode 0: Closed-Loop Angle (Steps + Sine Tracking)
        print("\n=== 3. Mode 0: Closed-Loop Position (Angle) ===")
        for label, sp in [
            ("Center", CENTER),
            ("Step +0.35", CENTER + 0.35),
            ("Step -0.35", CENTER - 0.35),
            ("Return Center", CENTER),
        ]:
            write_f32(c, REG_TARGET, sp)
            pk_i = 0.0
            pk_v = 0.0
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.4:
                last_a, v, cur = check_safety(c, last_a)
                pk_i = max(pk_i, cur)
                pk_v = max(pk_v, abs(v))
                time.sleep(0.03)
            err = sp - last_a
            print(
                f"  {label:14s}: target={sp:+.4f} actual={last_a:+.4f} "
                f"err={err:+.4f} rad ({math.degrees(err):+.2f} deg) "
                f"peak_I={pk_i:.1f} mA max_vel={pk_v:.2f} rad/s"
            )

        amp = 0.35
        freq_hz = 0.4
        dur_s = 2.5
        errs = []
        pk_i = 0.0
        t0 = time.monotonic()
        while True:
            dt = time.monotonic() - t0
            if dt >= dur_s:
                break
            sp = CENTER + amp * math.sin(2.0 * math.pi * freq_hz * dt)
            write_f32(c, REG_TARGET, sp)
            last_a, v, cur = check_safety(c, last_a)
            pk_i = max(pk_i, cur)
            errs.append(sp - last_a)
            time.sleep(0.025)
        rms_err = math.sqrt(sum(e * e for e in errs) / len(errs))
        print(
            f"  Sine (A={amp:.2f} rad, f={freq_hz:.1f} Hz, {len(errs)} pts): "
            f"RMS err={rms_err:.4f} rad ({math.degrees(rms_err):.2f} deg), peak_I={pk_i:.1f} mA"
        )

        # 4. Mode 3: Closed-Loop Velocity
        print("\n=== 4. Mode 3: Closed-Loop Velocity ===")
        write_f32(c, REG_TARGET, 0.0)
        write_u8(c, REG_CONTROL_MODE, 3)
        for label, vel_sp, bound_check in [
            ("Forward +2.0", 2.0, lambda a: a < SAFE_MAX),
            ("Reverse -2.0", -2.0, lambda a: a > SAFE_MIN),
        ]:
            write_f32(c, REG_TARGET, vel_sp)
            vels = []
            pk_i = 0.0
            a_start = last_a
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.9:
                last_a, v, cur = check_safety(c, last_a)
                pk_i = max(pk_i, cur)
                vels.append(v)
                if not bound_check(last_a):
                    break
                time.sleep(0.025)
            write_f32(c, REG_TARGET, 0.0)
            time.sleep(0.1)
            avg_v = sum(vels) / len(vels) if vels else 0.0
            print(
                f"  {label:14s}: delta_angle={last_a - a_start:+.4f} rad, "
                f"avg_vel={avg_v:+.2f} rad/s, peak_I={pk_i:.1f} mA"
            )

        # 5. Mode 2: Closed-Loop Torque (Voltage)
        print("\n=== 5. Mode 2: Closed-Loop Torque (Voltage) ===")
        write_f32(c, REG_TARGET, 0.0)
        write_u8(c, REG_CONTROL_MODE, 2)
        for label, v_sp, bound_check in [
            ("Torque +2.2V", 2.2, lambda a: a < SAFE_MAX),
            ("Torque -2.2V", -2.2, lambda a: a > SAFE_MIN),
        ]:
            write_f32(c, REG_TARGET, v_sp)
            pk_i = 0.0
            pk_v = 0.0
            a_start = last_a
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.7:
                last_a, v, cur = check_safety(c, last_a)
                pk_i = max(pk_i, cur)
                pk_v = max(pk_v, abs(v))
                if not bound_check(last_a):
                    break
                time.sleep(0.025)
            write_f32(c, REG_TARGET, 0.0)
            time.sleep(0.1)
            print(
                f"  {label:14s}: delta_angle={last_a - a_start:+.4f} rad, "
                f"max_vel={pk_v:.2f} rad/s, peak_I={pk_i:.1f} mA"
            )

        # Return to center and disable
        write_u8(c, REG_CONTROL_MODE, 0)
        write_f32(c, REG_TARGET, CENTER)
        time.sleep(0.8)
        last_a, _, _ = check_safety(c, last_a)
        write_u8(c, REG_ENABLE, 0)
        print(f"\nFinal centered angle={last_a:+.4f} rad ({math.degrees(last_a):+.2f} deg), En=0, Err=0x{read_u16(c, REG_HW_ERR_STATUS):X}")

    finally:
        try:
            write_u8(c, REG_ENABLE, 0)
            write_u8(c, REG_CONTROL_MODE, 0)
        except Exception:
            pass
        c.close()


if __name__ == "__main__":
    main()
