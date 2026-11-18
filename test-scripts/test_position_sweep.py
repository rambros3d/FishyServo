#!/usr/bin/env python3
"""Sweep closed-loop position across 6 equal positions between endpoints."""

import sys
import time
from pyocd.core.helpers import ConnectHelper

PROBE_ID = "A1A48F062680"
TARGET_TYPE = "stm32h503kbux"

def main():
    print("=======================================================")
    print("      N20-SERVO 6-POINT POSITION SWEEP TEST            ")
    print("=======================================================")

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=PROBE_ID,
        target_override=TARGET_TYPE,
        options={"connect_mode": "attach", "frequency": 500000}
    )
    session.open()
    target = session.target
    if target.is_halted():
        target.resume()

    # Find RTT block
    ram = bytes(target.read_memory_block8(0x20000000, 0x8000))
    idx = ram.find(b"SEGGER RTT\0")
    if idx == -1:
        print("ERROR: SEGGER RTT header not found in RAM")
        session.close()
        sys.exit(1)
    rtt_cb_addr = 0x20000000 + idx

    up_hdr = target.read_memory_block32(rtt_cb_addr + 0x18, 6)
    up_buf_ptr, up_buf_size = up_hdr[1], up_hdr[2]
    down_hdr = target.read_memory_block32(rtt_cb_addr + 0x30, 6)
    down_buf_ptr, down_buf_size = down_hdr[1], down_hdr[2]

    def send_cmd(cmd_str):
        nonlocal target
        cmd_bytes = cmd_str.encode("utf-8") + b"\n"
        hdr = target.read_memory_block32(rtt_cb_addr + 0x30, 6)
        wr_off = hdr[3]
        for b in cmd_bytes:
            target.write8(down_buf_ptr + wr_off, b)
            wr_off = (wr_off + 1) % down_buf_size
        target.write32(rtt_cb_addr + 0x30 + 12, wr_off)

    def drain_up():
        nonlocal target
        try:
            hdr = target.read_memory_block32(rtt_cb_addr + 0x18, 6)
            wr_off, rd_off = hdr[3], hdr[4]
            if wr_off == rd_off:
                return ""
            out = bytearray()
            while rd_off != wr_off:
                out.append(target.read8(up_buf_ptr + rd_off))
                rd_off = (rd_off + 1) % up_buf_size
            target.write32(rtt_cb_addr + 0x18 + 16, rd_off)
            return out.decode("utf-8", errors="replace")
        except Exception as e:
            return f"[err: {e}]"

    def read_settled_telemetry(samples=6, sample_delay=0.08):
        nonlocal target
        drain_up()
        angles = []
        currents = []
        velocities = []
        for _ in range(samples):
            time.sleep(sample_delay)
            txt = drain_up()
            for line in txt.splitlines():
                if "ang=" in line:
                    try:
                        # Format: [ANG] tgt=0.00 (0.0°) | ang=0.01 (0.4°) | vel=0.00 rad/s | I=12.3 mA | en=1
                        part_ang = line.split("ang=")[1].split("(")[1].split("°")[0].strip()
                        angles.append(float(part_ang))
                        if "I=" in line:
                            part_i = line.split("I=")[1].split("mA")[0].strip()
                            currents.append(float(part_i))
                        if "vel=" in line:
                            part_v = line.split("vel=")[1].split("rad/s")[0].strip()
                            velocities.append(float(part_v))
                    except Exception:
                        pass
        avg_ang = sum(angles) / len(angles) if angles else None
        avg_i = sum(currents) / len(currents) if currents else None
        avg_v = sum(velocities) / len(velocities) if velocities else None
        return avg_ang, avg_i, avg_v

    def move_and_settle(target_deg, wait_sec=1.3):
        nonlocal session, target
        send_cmd(f"d {target_deg:.2f}")
        time.sleep(0.05)
        # Detach SWD during motion to prevent brush noise disturbance
        session.close()
        time.sleep(wait_sec)
        session = ConnectHelper.session_with_chosen_probe(
            unique_id=PROBE_ID,
            target_override=TARGET_TYPE,
            options={"connect_mode": "attach", "frequency": 500000}
        )
        session.open()
        target = session.target
        if target.is_halted():
            target.resume()
        return read_settled_telemetry()

    print("Centering servo at 0.0° before sweep...")
    move_and_settle(0.0, wait_sec=1.0)
    print("Servo initialized.")

    # 6 equal positions across stroke: -40.0° to +40.0° (step = 16.0°)
    # Safe stroke within physical endpoints (-46.8° to +42.4°)
    positions = [-40.0, -24.0, -8.0, 8.0, 24.0, 40.0]

    print("\n" + "="*68)
    print(" SWEEP PROFILE: 6 EQUAL POSITIONS BETWEEN ENDPOINTS (±40.0°)")
    print(f" Positions: {positions}")
    print("="*68)

    # -------------------------------------------------------------------
    # Pass 1: Forward Sweep (-40° -> +40°)
    # -------------------------------------------------------------------
    print("\n>>> STARTING PASS 1: FORWARD SWEEP (-40° -> +40°)")
    fwd_records = []
    print(f"{'Idx':<4} | {'Target':<8} | {'Measured':<10} | {'Error':<8} | {'Current':<10} | {'Velocity'}")
    print("-" * 68)

    for i, tgt in enumerate(positions):
        meas_ang, meas_i, meas_v = move_and_settle(tgt, wait_sec=1.4)
        if meas_ang is not None:
            err = meas_ang - tgt
            fwd_records.append({
                "idx": i + 1,
                "target": tgt,
                "measured": meas_ang,
                "error": err,
                "current": meas_i,
                "velocity": meas_v
            })
            i_str = f"{meas_i:.1f} mA" if meas_i is not None else "N/A"
            v_str = f"{meas_v:.2f} r/s" if meas_v is not None else "N/A"
            print(f"{i+1:<4} | {tgt:+6.1f}°  | {meas_ang:+7.2f}°   | {err:+6.2f}° | {i_str:<10} | {v_str}")
        else:
            print(f"{i+1:<4} | {tgt:+6.1f}°  | [No telemetry read]")

    time.sleep(0.5)

    # -------------------------------------------------------------------
    # Pass 2: Reverse Sweep (+40° -> -40°)
    # -------------------------------------------------------------------
    rev_positions = list(reversed(positions))
    print("\n>>> STARTING PASS 2: REVERSE SWEEP (+40° -> -40°)")
    rev_records = []
    print(f"{'Idx':<4} | {'Target':<8} | {'Measured':<10} | {'Error':<8} | {'Current':<10} | {'Velocity'}")
    print("-" * 68)

    for i, tgt in enumerate(rev_positions):
        meas_ang, meas_i, meas_v = move_and_settle(tgt, wait_sec=1.4)
        if meas_ang is not None:
            err = meas_ang - tgt
            rev_records.append({
                "idx": i + 1,
                "target": tgt,
                "measured": meas_ang,
                "error": err,
                "current": meas_i,
                "velocity": meas_v
            })
            i_str = f"{meas_i:.1f} mA" if meas_i is not None else "N/A"
            v_str = f"{meas_v:.2f} r/s" if meas_v is not None else "N/A"
            print(f"{i+1:<4} | {tgt:+6.1f}°  | {meas_ang:+7.2f}°   | {err:+6.2f}° | {i_str:<10} | {v_str}")
        else:
            print(f"{i+1:<4} | {tgt:+6.1f}°  | [No telemetry read]")

    # Return to center
    print("\nReturning servo to center (0.0°)...")
    center_ang, center_i, _ = move_and_settle(0.0, wait_sec=1.2)
    if center_ang is not None:
        print(f"Center position reached: {center_ang:+.2f}° (Current: {center_i:.1f} mA)")

    # -------------------------------------------------------------------
    # Statistical Summary
    # -------------------------------------------------------------------
    all_errors = [abs(r["error"]) for r in fwd_records + rev_records]
    all_currents = [r["current"] for r in fwd_records + rev_records if r["current"] is not None]

    print("\n" + "="*68)
    print("                    POSITION SWEEP SUMMARY                     ")
    print("===============================================================")
    if all_errors:
        max_err = max(all_errors)
        mean_err = sum(all_errors) / len(all_errors)
        print(f"  Total test points evaluated:   {len(all_errors)} (6 forward + 6 reverse)")
        print(f"  Mean Absolute Error (MAE):     {mean_err:.2f}°")
        print(f"  Max Absolute Position Error:   {max_err:.2f}°")
    if all_currents:
        avg_current = sum(all_currents) / len(all_currents)
        max_current = max(all_currents)
        print(f"  Average Holding Current:       {avg_current:.1f} mA")
        print(f"  Peak Holding Current:          {max_current:.1f} mA")

    print("\n  Hysteresis / Directional Repeatability at Matching Targets:")
    for f in fwd_records:
        match_rev = next((r for r in rev_records if abs(r["target"] - f["target"]) < 0.1), None)
        if match_rev:
            diff = abs(f["measured"] - match_rev["measured"])
            print(f"    Target {f['target']:+5.1f}°: Fwd={f['measured']:+6.2f}°, Rev={match_rev['measured']:+6.2f}° -> Delta = {diff:.2f}°")
    print("===============================================================")

    session.close()

if __name__ == "__main__":
    main()
