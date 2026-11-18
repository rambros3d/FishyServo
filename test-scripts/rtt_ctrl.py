#!/usr/bin/env python3
"""Interactive RTT controller and monitor for N20-Servo with endpoint homing."""

import sys
import time
import struct
import pyocd
from pyocd.core.helpers import ConnectHelper

def main():
    probe_id = "A1A48F062680"
    target_type = "stm32h503kbux"

    print(f"Connecting to N20-Servo (Probe: {probe_id})...")
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id,
        target_override=target_type,
        options={"connect_mode": "attach"}
    )
    session.open()
    target = session.target
    if target.is_halted():
        target.resume()

    # Find _SEGGER_RTT
    rtt_cb_addr = 0x20001be8
    header = bytes(target.read_memory_block8(rtt_cb_addr, 16))
    if not header.startswith(b"SEGGER RTT"):
        print("Scanning SRAM for SEGGER RTT header...")
        ram = bytes(target.read_memory_block8(0x20000000, 0x8000))
        idx = ram.find(b"SEGGER RTT\0")
        if idx == -1:
            print("ERROR: SEGGER RTT not found in RAM")
            return
        rtt_cb_addr = 0x20000000 + idx

    print(f"Found SEGGER RTT at 0x{rtt_cb_addr:08X}")

    up_hdr = target.read_memory_block32(rtt_cb_addr + 0x18, 6)
    up_buf_ptr = up_hdr[1]
    up_buf_size = up_hdr[2]

    down_hdr = target.read_memory_block32(rtt_cb_addr + 0x30, 6)
    down_buf_ptr = down_hdr[1]
    down_buf_size = down_hdr[2]

    def send_cmd(cmd_str):
        nonlocal target
        cmd_bytes = cmd_str.encode("utf-8") + b"\n"
        hdr = target.read_memory_block32(rtt_cb_addr + 0x30, 6)
        wr_off = hdr[3]
        rd_off = hdr[4]
        for b in cmd_bytes:
            target.write8(down_buf_ptr + wr_off, b)
            wr_off = (wr_off + 1) % down_buf_size
        target.write32(rtt_cb_addr + 0x30 + 12, wr_off)
        print(f"\n--> Sent command: '{cmd_str}'")

    def read_up():
        nonlocal target
        hdr = target.read_memory_block32(rtt_cb_addr + 0x18, 6)
        wr_off = hdr[3]
        rd_off = hdr[4]
        if wr_off == rd_off:
            return ""
        out = bytearray()
        while rd_off != wr_off:
            out.append(target.read8(up_buf_ptr + rd_off))
            rd_off = (rd_off + 1) % up_buf_size
        target.write32(rtt_cb_addr + 0x18 + 16, rd_off)
        return out.decode("utf-8", errors="replace")

    # Read initial stream
    time.sleep(0.2)
    initial_text = read_up()
    if initial_text:
        print(initial_text, end="")

    # Send status
    send_cmd("s")
    time.sleep(0.3)
    print(read_up(), end="")

    # Send home command
    send_cmd("home 1.5 80")
    for _ in range(25):
        time.sleep(0.3)
        chunk = read_up()
        if chunk:
            print(chunk, end="", flush=True)

    session.close()
    print("\nEndpoint test script completed.")

if __name__ == "__main__":
    main()
