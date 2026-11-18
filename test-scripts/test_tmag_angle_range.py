#!/usr/bin/env python3
"""Run motor in open loop and measure TMAG5273 angle range between endpoints."""

import subprocess
import sys
import time
from pyocd.core.helpers import ConnectHelper

PROBE_ID = "A1A48F062680"
TARGET_TYPE = "stm32h503kbux"
ELF_PATH = "/home/sun/Filelink/Firmwares/N20-Servo/firmware/target/thumbv8m.main-none-eabihf/release/n20-servo"

def main():
    print("=======================================================")
    print("   TMAG5273 OPEN-LOOP ANGLE RANGE VERIFICATION TEST   ")
    print("=======================================================")

    print("Flashing firmware...")
    res = subprocess.run([
        "pyocd", "flash",
        "-u", PROBE_ID,
        "-t", TARGET_TYPE,
        "-f", "500000",
        "--format", "elf",
        ELF_PATH
    ], capture_output=True, text=True)
    if res.returncode != 0:
        print("Flash error:", res.stderr)
        sys.exit(1)
    print("Flash OK.")

    print("Resetting target (hardware reset)...")
    subprocess.run(["pyocd", "reset", "-u", PROBE_ID, "-t", TARGET_TYPE, "-f", "500000", "-m", "hw"], capture_output=True)
    time.sleep(0.5)

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=PROBE_ID,
        target_override=TARGET_TYPE,
        options={"connect_mode": "attach", "frequency": 500000}
    )
    session.open()
    target = session.target
    if target.is_halted():
        target.resume()

    # Locate RTT
    rtt_cb_addr = None
    for _ in range(25):
        try:
            ram = bytes(target.read_memory_block8(0x20000000, 0x8000))
            idx = ram.find(b"SEGGER RTT\0")
            if idx != -1:
                rtt_cb_addr = 0x20000000 + idx
                break
        except Exception:
            pass
        time.sleep(0.1)

    if not rtt_cb_addr:
        print("ERROR: SEGGER RTT header not found in RAM")
        session.close()
        sys.exit(1)

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
        print(f">>> Sent: '{cmd_str}'")

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

    # Drain boot logs until console is ready
    print("Waiting for target initialization and RTT console ready...")
    start_boot = time.time()
    boot_log = ""
    while time.time() - start_boot < 6.0:
        txt = drain_up()
        if txt:
            sys.stdout.write(txt)
            sys.stdout.flush()
            boot_log += txt
            if "RTT Interactive Console Ready" in boot_log or "System running" in boot_log:
                break
        time.sleep(0.1)

    time.sleep(0.3)
    drain_up()

    print("\n" + "="*55)
    print(" STEP 1: Running Open-Loop Full-Stroke Stall Homing Test")
    print(" (V = 4.0V, Threshold = 100mA)")
    print("="*55)
    send_cmd("home 4.0 100")
    time.sleep(0.1)

    # Disconnect probe during autonomous travel to eliminate brush noise interference on SWD
    session.close()
    print("-> Motor homing sequence running autonomously in firmware...")
    print("-> Waiting 4.5 seconds for Forward -> Hard Stop -> Relax -> Reverse -> Hard Stop...")
    time.sleep(4.5)

    # Reconnect to read the complete telemetry and angle range summary
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=PROBE_ID,
        target_override=TARGET_TYPE,
        options={"connect_mode": "attach", "frequency": 500000}
    )
    session.open()
    target = session.target
    if target.is_halted():
        target.resume()

    txt = drain_up()
    if txt:
        print("\n--- Homing Telemetry & Summary Log ---")
        print(txt)

    angles_seen = []
    if txt:
        for line in txt.splitlines():
            if "ang=" in line:
                try:
                    ang_str = line.split("ang=")[1].split(" ")[0].strip()
                    angles_seen.append(float(ang_str))
                except Exception:
                    pass

    if angles_seen:
        min_ang = min(angles_seen)
        max_ang = max(angles_seen)
        span_rad = max_ang - min_ang
        span_deg = span_rad * 180.0 / 3.14159265
        print("\n" + "="*55)
        print("          TMAG5273 LIVE ANGLE RANGE ANALYSIS          ")
        print("=======================================================")
        print(f"  Total angle samples captured: {len(angles_seen)}")
        print(f"  Minimum observed angle:       {min_ang:.3f} rad ({min_ang * 180.0 / 3.14159265:.1f}°)")
        print(f"  Maximum observed angle:       {max_ang:.3f} rad ({max_ang * 180.0 / 3.14159265:.1f}°)")
        print(f"  Observed Angular Sweep:       {span_rad:.3f} rad ({span_deg:.1f}°)")
        print("=======================================================")

    session.close()

if __name__ == "__main__":
    main()
