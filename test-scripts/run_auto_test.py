#!/usr/bin/env python3
"""Automated endpoint stall-detection & current spike test for N20-Servo."""

import sys
import time
from pyocd.core.helpers import ConnectHelper

def main():
    probe_id = "A1A48F062680"
    target_type = "stm32h503kbux"

    voltage = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    stall_ma = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0

    print("=======================================================")
    print("      N20-SERVO AUTOMATED ENDPOINT STALL TEST")
    print(f"      Voltage: {voltage:.2f} V | Stall Threshold: {stall_ma:.1f} mA")
    print("=======================================================")
    print(f"Connecting to target via SWD (Probe: {probe_id}, 1 MHz)...")

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id,
        target_override=target_type,
        options={"connect_mode": "attach", "frequency": 1000000}
    )
    session.open()
    target = session.target
    if target.is_halted():
        target.resume()

    # Locate _SEGGER_RTT dynamically
    ram = bytes(target.read_memory_block8(0x20000000, 0x8000))
    idx = ram.find(b"SEGGER RTT\0")
    if idx == -1:
        print("ERROR: SEGGER RTT header not found in RAM")
        session.close()
        return
    rtt_cb_addr = 0x20000000 + idx
    print(f"Connected! SEGGER RTT block located at 0x{rtt_cb_addr:08X}")

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
            return f"[SWD read err: {e}]"

    drain_up()
    time.sleep(0.1)

    print("\n--- Triggering Automated Endpoint Homing ---")
    send_cmd(f"home {voltage} {stall_ma}")

    start_home = time.time()
    homing_finished = False
    while time.time() - start_home < 35.0:
        time.sleep(0.1)
        txt = drain_up()
        if txt:
            print(txt, end="", flush=True)
            if "ENDPOINT HOMING SUMMARY" in txt or "timeout" in txt:
                homing_finished = True
                time.sleep(0.5)
                print(drain_up(), end="", flush=True)
                break

    # Ensure disarmed
    send_cmd("u 0.0")
    send_cmd("e 0")
    time.sleep(0.2)
    print(drain_up())

    session.close()
    print("\n=======================================================")
    print("Test finished.")
    print("=======================================================")

if __name__ == "__main__":
    main()
