#!/usr/bin/env python3
"""Test refined current sensing and sensorless stall detection up to 500mA."""

import subprocess
import sys
import time
from pyocd.core.helpers import ConnectHelper

PROBE_ID = "A1A48F062680"
TARGET_TYPE = "stm32h503kbux"
ELF_PATH = "/home/sun/Filelink/Firmwares/N20-Servo/firmware/target/thumbv8m.main-none-eabihf/release/n20-servo"

def main():
    print("=======================================================")
    print("   N20-SERVO REFINED SENSORLESS STALL TEST (UP TO 500mA)")
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

    print("Resetting target...")
    subprocess.run(["pyocd", "reset", "-u", PROBE_ID, "-t", TARGET_TYPE, "-f", "500000"], capture_output=True)
    time.sleep(0.3)

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
        print("ERROR: SEGGER RTT block not found in RAM")
        session.close()
        sys.exit(1)

    print(f"SEGGER RTT block found at 0x{rtt_cb_addr:08X}\n")

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

    # Drain initial boot logs
    time.sleep(0.5)
    boot_log = drain_up()
    if boot_log:
        print(boot_log)

    # Test 1: Baseline stall homing at 4.0V with 100mA threshold
    print("\n" + "="*55)
    print(" TEST 1: Automated Endpoint Stall Homing @ 4.0V, 100mA")
    print("="*55)
    send_cmd("home 4.0 100")
    t0 = time.time()
    while time.time() - t0 < 6.0:
        time.sleep(0.1)
        txt = drain_up()
        if txt:
            print(txt, end="", flush=True)
            if "=== ENDPOINT HOMING SUMMARY" in txt:
                break
    time.sleep(0.5)
    print(drain_up(), end="", flush=True)

    # Test 2: Higher Voltage & Stall Current Testing at 4.5V and 5.0V (Up to 500mA)
    test_cases = [
        {"volts": 4.5, "thresh": 200.0},
        {"volts": 5.0, "thresh": 350.0},
        {"volts": 5.0, "thresh": 500.0},
    ]

    for tc in test_cases:
        v = tc["volts"]
        th = tc["thresh"]
        print("\n" + "="*55)
        print(f" TEST: Stall Homing @ {v:.1f}V (Threshold: {th:.0f}mA)")
        print("="*55)
        send_cmd(f"home {v} {th}")
        t0 = time.time()
        while time.time() - t0 < 7.0:
            time.sleep(0.1)
            txt = drain_up()
            if txt:
                print(txt, end="", flush=True)
                if "=== ENDPOINT HOMING SUMMARY" in txt:
                    break
        time.sleep(0.5)
        print(drain_up(), end="", flush=True)

    # Safe Disarm
    print("\n--- Final Safety Disarm ---")
    send_cmd("u 0.0")
    send_cmd("e 0")
    time.sleep(0.2)
    print(drain_up())

    session.close()
    print("=======================================================")
    print("All tests completed successfully.")
    print("=======================================================")

if __name__ == "__main__":
    main()
