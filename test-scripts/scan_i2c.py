#!/usr/bin/env python3
"""Flash, reset target, and read I2C Bus Scan report via SWD RTT."""

import subprocess
import sys
import time
from pyocd.core.helpers import ConnectHelper

PROBE_ID = "A1A48F062680"
TARGET_TYPE = "stm32h503kbux"
ELF_PATH = "/home/sun/Filelink/Firmwares/N20-Servo/firmware/target/thumbv8m.main-none-eabihf/release/n20-servo"

def main():
    print("Flashing firmware with pyocd...")
    cmd = [
        "pyocd", "flash",
        "-u", PROBE_ID,
        "-t", TARGET_TYPE,
        "-f", "500000",
        "--format", "elf",
        ELF_PATH
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print("Flash error:", res.stderr)
        sys.exit(1)
    print("Flash successful.")

    print("Resetting target and attaching RTT...")
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

    # Search for SEGGER RTT header
    rtt_cb_addr = None
    for _ in range(20):
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
        print("Failed to locate SEGGER RTT in RAM.")
        session.close()
        sys.exit(1)

    up_hdr = target.read_memory_block32(rtt_cb_addr + 0x18, 6)
    up_buf_ptr, up_buf_size = up_hdr[1], up_hdr[2]

    def read_rtt():
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

    print("\nReading target output & periodic 5s hot-plug monitor...\n" + "="*55)
    start_t = time.time()
    all_output = ""
    while time.time() - start_t < 13.0:
        txt = read_rtt()
        if txt:
            sys.stdout.write(txt)
            sys.stdout.flush()
            all_output += txt
        time.sleep(0.05)

    print("="*55)
    session.close()

if __name__ == "__main__":
    main()
