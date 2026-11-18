#!/usr/bin/env python3
"""Measure gearbox backlash and positional hysteresis on the N20 servo."""

import sys
import time
from pyocd.core.helpers import ConnectHelper

PROBE_ID = "A1A48F062680"
TARGET_TYPE = "stm32h503kbux"

def main():
    print("=======================================================")
    print("      N20-SERVO GEARBOX BACKLASH & HYSTERESIS TEST     ")
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

    def read_settled_angle(samples=5, sample_delay=0.1):
        nonlocal session, target
        drain_up()
        angles = []
        for _ in range(samples):
            time.sleep(sample_delay)
            txt = drain_up()
            for line in txt.splitlines():
                if "ang=" in line:
                    try:
                        # Format: [ANG] tgt=0.00 (0.0°) | ang=0.01 (0.4°) ...
                        part = line.split("ang=")[1].split("(")[1].split("°")[0].strip()
                        angles.append(float(part))
                    except Exception:
                        pass
        if angles:
            return sum(angles) / len(angles)
        return None

    def move_and_settle(target_deg, wait_sec=1.2):
        nonlocal session, target
        send_cmd(f"d {target_deg:.2f}")
        time.sleep(0.05)
        # Quiet SWD during travel
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
        ang = read_settled_angle(samples=6, sample_delay=0.08)
        return ang

    print("Checking initial connection & centering servo at 0.0°...")
    move_and_settle(0.0, wait_sec=1.0)
    print("Servo initialized.")

    # -----------------------------------------------------------------------
    # Part 1: Directional Hysteresis (Dual-Approach Backlash) Test
    # -----------------------------------------------------------------------
    test_points = [-20.0, -10.0, 0.0, 10.0, 20.0]
    approach_delta = 8.0 # Move 8° away to ensure full gear tooth disengagement and re-engagement
    results = []

    print("\n" + "="*55)
    print(" PART 1: DUAL-DIRECTION APPROACH HYSTERESIS TEST")
    print(f" Approach offset: ±{approach_delta:.1f}°")
    print("="*55)

    for pt in test_points:
        print(f"\n--- Testing Target Position: {pt:+.1f}° ---")
        
        # 1. Approach from Negative (driving Forward/Increasing)
        move_and_settle(pt - approach_delta, wait_sec=1.0)
        time.sleep(0.1)
        ang_from_neg = move_and_settle(pt, wait_sec=1.3)

        # 2. Approach from Positive (driving Reverse/Decreasing)
        move_and_settle(pt + approach_delta, wait_sec=1.0)
        time.sleep(0.1)
        ang_from_pos = move_and_settle(pt, wait_sec=1.3)

        if ang_from_neg is not None and ang_from_pos is not None:
            backlash_deg = abs(ang_from_neg - ang_from_pos)
            backlash_rad = backlash_deg * 3.14159265 / 180.0
            results.append({
                "target": pt,
                "from_neg": ang_from_neg,
                "from_pos": ang_from_pos,
                "backlash_deg": backlash_deg,
                "backlash_rad": backlash_rad,
            })
            print(f"  Target:            {pt:+.1f}°")
            print(f"  Approach from -:   {ang_from_neg:+.2f}° (err: {ang_from_neg - pt:+.2f}°)")
            print(f"  Approach from +:   {ang_from_pos:+.2f}° (err: {ang_from_pos - pt:+.2f}°)")
            print(f"  Backlash / Delta:  {backlash_deg:.2f}° ({backlash_rad:.4f} rad)")
        else:
            print("  WARN: Could not read settled angles reliably.")

    # -----------------------------------------------------------------------
    # Part 2: Reversal Deadband (Breakaway Sensitivity) Test
    # -----------------------------------------------------------------------
    print("\n" + "="*55)
    print(" PART 2: DIRECTION REVERSAL DEADBAND TEST AT CENTER")
    print("="*55)
    # Pre-load in negative direction
    move_and_settle(-5.0, wait_sec=0.8)
    # Move to 0.0° from negative direction
    base_ang = move_and_settle(0.0, wait_sec=1.2)
    print(f"Baseline settled angle (approached from -5°): {base_ang:+.2f}°")

    steps = [0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0]
    breakaway_rev_cmd = None
    breakaway_rev_meas = None

    print("\nApplying reverse micro-steps from settled forward position:")
    for s in steps:
        target_s = -s
        ang = move_and_settle(target_s, wait_sec=1.0)
        delta = abs(ang - base_ang) if ang is not None else 0.0
        print(f"  Cmd: {target_s:+.1f}° | Measured: {ang:+.2f}° | Displaced from baseline: {delta:.2f}°")
        if delta > 0.4 and breakaway_rev_cmd is None:
            breakaway_rev_cmd = s
            breakaway_rev_meas = delta

    # Return to center
    move_and_settle(0.0, wait_sec=1.0)

    # -----------------------------------------------------------------------
    # Summary Report
    # -----------------------------------------------------------------------
    print("\n" + "="*55)
    print("            N20 GEARBOX BACKLASH SUMMARY REPORT        ")
    print("=======================================================")
    if results:
        avg_bl_deg = sum(r["backlash_deg"] for r in results) / len(results)
        max_bl_deg = max(r["backlash_deg"] for r in results)
        min_bl_deg = min(r["backlash_deg"] for r in results)
        avg_bl_rad = avg_bl_deg * 3.14159265 / 180.0
        print(f"  Average Mechanical Backlash: {avg_bl_deg:.2f}° ({avg_bl_rad:.4f} rad)")
        print(f"  Min Backlash observed:       {min_bl_deg:.2f}°")
        print(f"  Max Backlash observed:       {max_bl_deg:.2f}°")
        print("  Per-Position Breakdown:")
        for r in results:
            print(f"    @ {r['target']:+5.1f}°: Backlash = {r['backlash_deg']:.2f}° ({r['backlash_deg']*60.0:.0f} arcmin)")
    if breakaway_rev_cmd:
        print(f"  Reversal Deadband Threshold: ~{breakaway_rev_cmd:.1f}° commanded displacement needed to overcome gear play")
    print("=======================================================")

    session.close()

if __name__ == "__main__":
    main()
