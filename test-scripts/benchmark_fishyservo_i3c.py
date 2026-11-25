#!/usr/bin/env python3
"""
Comprehensive closed-loop benchmarking suite for FishyServo over MIPI I3C:
  - Mode 0 (Angle): step response, settling time, bidirectional repeatability, backlash, sine tracking
  - Mode 3 (Velocity): closed-loop velocity sweep and peak closed-loop speed
  - Mode 2 (Torque/Voltage): max output joint speed (rad/s & RPM) and current draw
"""

import math
import statistics
import struct
import sys
import time

sys.path.insert(0, "/home/sun/Filelink/Firmwares/AgentLA-rust/sdk")
from agentla.i3c import AgentLaI3c, I3cException

TARGET_ADDR = 0x40
I3C_FREQ_KHZ = 1000

# 4-byte aligned MultiBus RegisterId offsets
REG_STATUS = 0x000
REG_TARGET = 0x004
REG_ENABLE = 0x00C
REG_CONTROL_MODE = 0x010
REG_ANGLE = 0x01C
REG_VELOCITY = 0x028
REG_CURRENT_DC = 0x088
REG_VEL_PID_P = 0x08C
REG_VEL_PID_I = 0x090
REG_VEL_PID_D = 0x094
REG_ANG_PID_P = 0x0A4
REG_VOLTAGE_LIMIT = 0x0EC
REG_CURRENT_LIMIT = 0x0F0
REG_VELOCITY_LIMIT = 0x0F4
REG_MIN_POS_LIMIT = 0x108
REG_MAX_POS_LIMIT = 0x10C
REG_STALL_TIMEOUT_MS = 0x110
REG_HW_ERR_STATUS = 0x114
REG_BUS_VOLTAGE = 0x118
REG_SENSOR_BUS_FREQ = 0x21C

# Safe interior operating region inside mechanical stops ([-4.91, -3.26] rad)
SOFT_MIN = -4.65
SOFT_MAX = -3.45
SAFE_MIN = -4.58
SAFE_MAX = -3.55
CENTER = -4.08
OVERCURRENT_ABORT_MA = 480.0


def _retry(fn):
    for attempt in range(4):
        try:
            return fn()
        except I3cException:
            if attempt == 3:
                raise
            time.sleep(0.002)


def ru8(c, r):
    return _retry(lambda: c.private_write_read(TARGET_ADDR, struct.pack(">H", r), 1)[0])


def ru16(c, r):
    return _retry(lambda: struct.unpack("<H", c.private_write_read(TARGET_ADDR, struct.pack(">H", r), 2))[0])


def rf32(c, r):
    return _retry(lambda: struct.unpack("<f", c.private_write_read(TARGET_ADDR, struct.pack(">H", r), 4))[0])


def wu8(c, r, v):
    _retry(lambda: c.private_write(TARGET_ADDR, struct.pack(">H", r) + bytes([v & 0xFF])))


def wu32(c, r, v):
    _retry(lambda: c.private_write(TARGET_ADDR, struct.pack(">H", r) + struct.pack("<I", v)))


def wf32(c, r, v):
    _retry(lambda: c.private_write(TARGET_ADDR, struct.pack(">H", r) + struct.pack("<f", v)))


def safe_angle(c, fallback):
    a = rf32(c, REG_ANGLE)
    return a if math.isfinite(a) and abs(a) > 1e-4 else fallback


def check_safety(c, last_a, expect_en=True):
    st = ru8(c, REG_STATUS)
    if st == 0:
        time.sleep(0.003)
        st = ru8(c, REG_STATUS)
    a = safe_angle(c, last_a)
    v = rf32(c, REG_VELOCITY)
    cur = rf32(c, REG_CURRENT_DC)
    if st != 4:
        wu8(c, REG_ENABLE, 0)
        raise RuntimeError(f"Controller fault STATUS={st}")
    if expect_en:
        err = ru16(c, REG_HW_ERR_STATUS)
        if err & 0x1800:
            raise RuntimeError(f"SafetySupervisor fault tripped! Err=0x{err:X}, Angle={a:+.4f}")
    if cur > OVERCURRENT_ABORT_MA:
        wu8(c, REG_ENABLE, 0)
        raise RuntimeError(f"Overcurrent abort I_DC={cur:.1f} mA")
    return a, v, cur


def move_to(c, sp, last_a, kp=16.0, dur=1.2):
    wu8(c, REG_CONTROL_MODE, 0)
    wf32(c, REG_ANG_PID_P, kp)
    wf32(c, REG_TARGET, sp)
    wu8(c, REG_ENABLE, 1)
    pk_i = 0.0
    pk_v = 0.0
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        last_a, v, cur = check_safety(c, last_a)
        pk_i = max(pk_i, cur)
        pk_v = max(pk_v, abs(v))
        time.sleep(0.025)
    return last_a, pk_i, pk_v


def main():
    c = AgentLaI3c()
    c.connect()
    c.set_inverted(False)
    c.set_bus_speed(I3C_FREQ_KHZ)

    try:
        try:
            st = ru8(c, REG_STATUS)
        except I3cException:
            try:
                c.rstdaa()
            except I3cException:
                pass
            c.entdaa(TARGET_ADDR)
            st = ru8(c, REG_STATUS)
        assert st == 4, f"Expected STATUS=4, got {st}"
        wu8(c, REG_ENABLE, 0)

        # Use 100 kHz I2C1 sensor bus for high EMI margin on FPC ribbon & full 4.9 V limit
        wu32(c, REG_SENSOR_BUS_FREQ, 100_000)
        wf32(c, REG_VOLTAGE_LIMIT, 4.9)
        wf32(c, REG_CURRENT_LIMIT, 0.55)
        wf32(c, REG_VELOCITY_LIMIT, 30.0)
        wf32(c, REG_STALL_TIMEOUT_MS, 500.0)
        wf32(c, REG_MIN_POS_LIMIT, 0.0)
        wf32(c, REG_MAX_POS_LIMIT, 0.0)

        vbus = rf32(c, REG_BUS_VOLTAGE)
        last_a = safe_angle(c, CENTER)
        print(f"Connected: STATUS=4, Vbus={vbus:.2f} V, StartAngle={last_a:+.4f} rad, Err=0x{ru16(c, REG_HW_ERR_STATUS):X}")

        # Break away cleanly from + hard stop if near -3.20 rad using -3.8V torque pulse
        if last_a > -3.60:
            wu8(c, REG_CONTROL_MODE, 2)
            wf32(c, REG_TARGET, -3.8)
            wu8(c, REG_ENABLE, 1)
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.8 and last_a > -3.70:
                last_a, _, _ = check_safety(c, last_a)
                time.sleep(0.025)
            wf32(c, REG_TARGET, 0.0)
            wu8(c, REG_ENABLE, 0)
            time.sleep(0.05)

        last_a, _, _ = move_to(c, CENTER, last_a, kp=16.0, dur=1.0)
        wf32(c, REG_MIN_POS_LIMIT, SOFT_MIN)
        wf32(c, REG_MAX_POS_LIMIT, SOFT_MAX)
        print(f"Centered at {last_a:+.4f} rad with soft limits [{SOFT_MIN}, {SOFT_MAX}] rad")

        # -------------------------------------------------------------------
        # 1. Mode 0 (Closed-Loop Angle): Step Response, Repeatability, Backlash, Sine
        # -------------------------------------------------------------------
        print("\n=== BENCHMARK 1: Mode 0 (Closed-Loop Position) ===")
        for kp in (16.0, 24.0):
            wf32(c, REG_ANG_PID_P, kp)
            for lbl, sp in [("Step +0.45", CENTER + 0.45), ("Step -0.45", CENTER - 0.45), ("Return Ctr", CENTER)]:
                t0 = time.monotonic()
                wf32(c, REG_TARGET, sp)
                pk_i = 0.0
                pk_v = 0.0
                settled_t = None
                while time.monotonic() - t0 < 1.2:
                    last_a, v, cur = check_safety(c, last_a)
                    pk_i = max(pk_i, cur)
                    pk_v = max(pk_v, abs(v))
                    if abs(sp - last_a) < 0.03 and abs(v) < 0.25 and settled_t is None:
                        settled_t = time.monotonic() - t0
                    time.sleep(0.02)
                err = sp - last_a
                st_str = f"{settled_t:.2f}s" if settled_t is not None else ">1.2s"
                print(
                    f"  Kp={kp:4.1f} {lbl:10s}: actual={last_a:+.4f} err={err:+.4f} rad ({math.degrees(err):+.2f} deg) "
                    f"settle={st_str} pk_I={pk_i:5.1f}mA pk_vel={pk_v:4.2f} rad/s"
                )

        # Bidirectional Repeatability & Backlash (Kp=24.0)
        wf32(c, REG_ANG_PID_P, 24.0)
        from_plus = []
        from_minus = []
        for k in range(5):
            last_a, _, _ = move_to(c, CENTER + 0.35, last_a, kp=24.0, dur=0.7)
            last_a, _, _ = move_to(c, CENTER, last_a, kp=24.0, dur=0.9)
            from_plus.append(last_a)
            last_a, _, _ = move_to(c, CENTER - 0.35, last_a, kp=24.0, dur=0.7)
            last_a, _, _ = move_to(c, CENTER, last_a, kp=24.0, dur=0.9)
            from_minus.append(last_a)

        mean_p = statistics.mean(from_plus)
        mean_m = statistics.mean(from_minus)
        std_p = statistics.stdev(from_plus)
        std_m = statistics.stdev(from_minus)
        backlash = abs(mean_p - mean_m)
        print(
            f"\n  Repeatability (+ approach, n=5): mean={mean_p:+.4f} std={std_p:.5f} rad ({math.degrees(std_p):.3f} deg), "
            f"spread={max(from_plus)-min(from_plus):.5f} rad"
        )
        print(
            f"  Repeatability (- approach, n=5): mean={mean_m:+.4f} std={std_m:.5f} rad ({math.degrees(std_m):.3f} deg), "
            f"spread={max(from_minus)-min(from_minus):.5f} rad"
        )
        print(f"  Bidirectional Backlash/Hysteresis: {backlash:.5f} rad ({math.degrees(backlash):.3f} deg)")

        # Sinusoidal tracking
        for freq_hz in (0.5, 1.0):
            amp = 0.35
            dur = 3.0
            errs = []
            pk_i = 0.0
            pk_v = 0.0
            t0 = time.monotonic()
            while True:
                dt = time.monotonic() - t0
                if dt >= dur:
                    break
                sp = CENTER + amp * math.sin(2.0 * math.pi * freq_hz * dt)
                wf32(c, REG_TARGET, sp)
                last_a, v, cur = check_safety(c, last_a)
                pk_i = max(pk_i, cur)
                pk_v = max(pk_v, abs(v))
                errs.append(sp - last_a)
                time.sleep(0.015)
            rms = math.sqrt(sum(e * e for e in errs) / len(errs))
            print(
                f"  Sine (A={amp:.2f} rad, f={freq_hz:.1f} Hz, {len(errs)} pts): "
                f"RMS err={rms:.4f} rad ({math.degrees(rms):.2f} deg), pk_I={pk_i:.1f}mA, pk_vel={pk_v:.2f} rad/s"
            )

        # -------------------------------------------------------------------
        # 2. Mode 3 (Closed-Loop Velocity) & Speed Sweep
        # -------------------------------------------------------------------
        print("\n=== BENCHMARK 2: Mode 3 (Closed-Loop Velocity) ===")
        # Move near SAFE_MIN first so + sweep has full stroke
        last_a, _, _ = move_to(c, SAFE_MIN + 0.10, last_a, kp=20.0, dur=0.9)
        wf32(c, REG_VEL_PID_P, 2.0)
        wf32(c, REG_VEL_PID_I, 0.5)
        wf32(c, REG_TARGET, 0.0)
        wu8(c, REG_CONTROL_MODE, 3)

        max_vel_cl = 0.0
        for vel_sp in (2.0, -2.0, 4.0, -4.0, 6.0, -6.0, 10.0, -10.0):
            # Position appropriately before each sweep
            if vel_sp > 0 and last_a > CENTER:
                last_a, _, _ = move_to(c, SAFE_MIN + 0.10, last_a, kp=20.0, dur=0.8)
                wf32(c, REG_TARGET, 0.0)
                wu8(c, REG_CONTROL_MODE, 3)
            elif vel_sp < 0 and last_a < CENTER:
                last_a, _, _ = move_to(c, SAFE_MAX - 0.10, last_a, kp=20.0, dur=0.8)
                wf32(c, REG_TARGET, 0.0)
                wu8(c, REG_CONTROL_MODE, 3)

            wf32(c, REG_TARGET, vel_sp)
            vels = []
            pk_i = 0.0
            a_start = last_a
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.7:
                last_a, v, cur = check_safety(c, last_a)
                pk_i = max(pk_i, cur)
                vels.append(v)
                if (vel_sp > 0 and last_a >= SAFE_MAX) or (vel_sp < 0 and last_a <= SAFE_MIN):
                    break
                time.sleep(0.015)
            dt_run = time.monotonic() - t0
            wf32(c, REG_TARGET, 0.0)
            time.sleep(0.08)
            avg_v = (last_a - a_start) / max(dt_run, 1e-3)
            pk_v = max(abs(x) for x in vels) if vels else 0.0
            max_vel_cl = max(max_vel_cl, abs(avg_v), pk_v)
            print(
                f"  Target={vel_sp:+5.1f} rad/s: avg_vel={avg_v:+5.2f} rad/s ({avg_v*60/(2*math.pi):+5.1f} RPM), "
                f"pk_vel={pk_v:5.2f} rad/s, dAngle={last_a-a_start:+.3f} rad, pk_I={pk_i:5.1f} mA"
            )

        # -------------------------------------------------------------------
        # 3. Mode 2 (Closed-Loop Torque/Voltage) & Max Unloaded Speed
        # -------------------------------------------------------------------
        print("\n=== BENCHMARK 3: Mode 2 (Closed-Loop Torque / Voltage) & Max Speed ===")
        max_vel_torque = 0.0
        for v_sp in (3.5, -3.5, 4.8, -4.8):
            if v_sp > 0:
                last_a, _, _ = move_to(c, SAFE_MIN + 0.08, last_a, kp=20.0, dur=0.8)
            else:
                last_a, _, _ = move_to(c, SAFE_MAX - 0.08, last_a, kp=20.0, dur=0.8)
            wf32(c, REG_TARGET, 0.0)
            wu8(c, REG_CONTROL_MODE, 2)

            wf32(c, REG_TARGET, v_sp)
            pk_i = 0.0
            pk_v = 0.0
            a_start = last_a
            t0 = time.monotonic()
            while time.monotonic() - t0 < 0.6:
                last_a, v, cur = check_safety(c, last_a)
                pk_i = max(pk_i, cur)
                pk_v = max(pk_v, abs(v))
                if (v_sp > 0 and last_a >= SAFE_MAX) or (v_sp < 0 and last_a <= SAFE_MIN):
                    break
                time.sleep(0.012)
            dt_run = time.monotonic() - t0
            wf32(c, REG_TARGET, 0.0)
            time.sleep(0.08)
            avg_v = (last_a - a_start) / max(dt_run, 1e-3)
            max_vel_torque = max(max_vel_torque, abs(avg_v), pk_v)
            print(
                f"  Voltage={v_sp:+4.1f} V: avg_vel={avg_v:+5.2f} rad/s ({avg_v*60/(2*math.pi):+5.1f} RPM), "
                f"pk_vel={pk_v:5.2f} rad/s, dAngle={last_a-a_start:+.3f} rad, pk_I={pk_i:5.1f} mA"
            )

        # Return to center and disable
        last_a, _, _ = move_to(c, CENTER, last_a, kp=16.0, dur=0.9)
        wu8(c, REG_ENABLE, 0)
        err_final = ru16(c, REG_HW_ERR_STATUS)
        print(f"\n=== SUMMARY ===")
        print(f"  Max Closed-Loop Velocity (Mode 3): {max_vel_cl:.2f} rad/s ({max_vel_cl*60/(2*math.pi):.1f} RPM)")
        print(f"  Max Voltage-Mode Speed   (Mode 2): {max_vel_torque:.2f} rad/s ({max_vel_torque*60/(2*math.pi):.1f} RPM)")
        print(f"  Unidirectional Repeatability (SD): {std_p:.5f} / {std_m:.5f} rad ({math.degrees(std_p):.3f} / {math.degrees(std_m):.3f} deg)")
        print(f"  Bidirectional Backlash           : {backlash:.5f} rad ({math.degrees(backlash):.3f} deg)")
        print(f"  Final Angle={last_a:+.4f} rad, En=0, Err=0x{err_final:X}")

    finally:
        try:
            wu8(c, REG_ENABLE, 0)
            wu8(c, REG_CONTROL_MODE, 0)
        except Exception:
            pass
        c.close()


if __name__ == "__main__":
    main()
