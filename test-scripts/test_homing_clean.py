#!/usr/bin/env python3
"""Reliable automated stall homing with quiet SWD during travel."""

import sys
import time
from pyocd.core.helpers import ConnectHelper

def run_homing(voltage=4.0, threshold=80.0):
    probe_id = "A1A48F062680"
    target_type = "stm32h503kbux"

    print("=======================================================")
    print("      N20-SERVO AUTOMATED ENDPOINT STALL TEST")
    print(f"      Voltage: {voltage:.2f} V | Stall Threshold: {threshold:.1f} mA")
    print("=======================================================")

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id,
        target_override=target_type,
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
        return
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
        print(f">>> Sent: '{cmd_str}'")

    def drain_up():
        nonlocal target
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

    drain_up()
    time.sleep(0.1)

    # Trigger Homing
    send_cmd(f"home {voltage} {threshold}")

    # Close session during autonomous motor motion to keep SWD line quiet & immune to brush noise
    session.close()
    print("-> Motor homing sequence running autonomously in firmware...")
    print("-> Waiting 4.0 seconds for Forward -> Hard Stop -> Relax -> Reverse -> Hard Stop...")
    time.sleep(4.0)

    # Reconnect to read the complete RTT log
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id,
        target_override=target_type,
        options={"connect_mode": "attach", "frequency": 500000}
    )
    session.open()
    target = session.target
    if target.is_halted():
        target.resume()

    print("\n--- Telemetry & Homing Summary Log ---")
    log = drain_up()
    print(log.strip())

    session.close()
    print("\n=======================================================")
    print("Test complete.")
    print("=======================================================")

if __name__ == "__main__":
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
    th = float(sys.argv[2]) if len(sys.argv) > 2 else 80.0
    run_homing(v, th)
