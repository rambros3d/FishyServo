#!/usr/bin/env python3
"""Progressive higher-current testing up to 200mA with quiet SWD during travel."""

import time
from pyocd.core.helpers import ConnectHelper

def main():
    probe_id = "A1A48F062680"
    target_type = "stm32h503kbux"

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id,
        target_override=target_type,
        options={"connect_mode": "attach", "frequency": 500000}
    )
    session.open()
    target = session.target
    if target.is_halted():
        target.resume()

    rtt_cb_addr = 0x20001c40
    up_hdr = target.read_memory_block32(rtt_cb_addr + 0x18, 6)
    up_ptr, up_sz = up_hdr[1], up_hdr[2]
    down_hdr = target.read_memory_block32(rtt_cb_addr + 0x30, 6)
    down_ptr, down_sz = down_hdr[1], down_hdr[2]

    def send_cmd(cmd_str):
        nonlocal target
        cmd_bytes = cmd_str.encode("utf-8") + b"\n"
        hdr = target.read_memory_block32(rtt_cb_addr + 0x30, 6)
        wr_off = hdr[3]
        for b in cmd_bytes:
            target.write8(down_ptr + wr_off, b)
            wr_off = (wr_off + 1) % down_sz
        target.write32(rtt_cb_addr + 0x30 + 12, wr_off)

    def drain_up():
        nonlocal target
        hdr = target.read_memory_block32(rtt_cb_addr + 0x18, 6)
        wr_off, rd_off = hdr[3], hdr[4]
        if wr_off == rd_off:
            return ""
        out = bytearray()
        while rd_off != wr_off:
            out.append(target.read8(up_ptr + rd_off))
            rd_off = (rd_off + 1) % up_sz
        target.write32(rtt_cb_addr + 0x18 + 16, rd_off)
        return out.decode("utf-8", errors="replace")

    drain_up()

    results = []

    for v in [3.5, 4.0, 4.5, 5.0]:
        print(f"\n=======================================================")
        print(f"             TESTING ±{v:.1f} V")
        print(f"=======================================================")

        # 1. Forward run for 2.2 seconds (hits FWD stop)
        print(f">>> Driving Forward +{v:.1f}V...")
        send_cmd(f"u {v}")
        time.sleep(2.2) # Quiet on SWD during motor travel and stall!
        send_cmd("u 0.0")
        time.sleep(0.3)
        fwd_log = drain_up()
        print("FWD Run Log:")
        print(fwd_log.strip())

        # Extract peak FWD current
        peak_fwd = 0.0
        for line in fwd_log.splitlines():
            if "I=" in line:
                try:
                    val = float(line.split("I=")[1].split(" mA")[0].strip())
                    if val > peak_fwd:
                        peak_fwd = val
                except Exception:
                    pass

        time.sleep(0.5)

        # 2. Reverse run for 2.2 seconds (hits REV stop)
        print(f">>> Driving Reverse -{v:.1f}V...")
        send_cmd(f"u -{v}")
        time.sleep(2.2) # Quiet on SWD
        send_cmd("u 0.0")
        time.sleep(0.3)
        rev_log = drain_up()
        print("REV Run Log:")
        print(rev_log.strip())

        # Extract peak REV current
        peak_rev = 0.0
        for line in rev_log.splitlines():
            if "I=" in line:
                try:
                    val = float(line.split("I=")[1].split(" mA")[0].strip())
                    if val > peak_rev:
                        peak_rev = val
                except Exception:
                    pass

        results.append((v, peak_fwd, peak_rev))
        print(f"-> At ±{v:.1f}V: Peak FWD = {peak_fwd:.1f} mA | Peak REV = {peak_rev:.1f} mA")
        time.sleep(0.5)

    # Final Disarm
    send_cmd("u 0.0")
    send_cmd("e 0")
    time.sleep(0.2)
    drain_up()
    session.close()

    print("\n=======================================================")
    print("               FULL TEST SUMMARY")
    print("=======================================================")
    print(" Voltage | Peak FWD Current | Peak REV Current")
    print("---------+------------------+------------------")
    for v, fwd, rev in results:
        print(f"  {v:.1f} V   |     {fwd:5.1f} mA     |     {rev:5.1f} mA")
    print("=======================================================")

if __name__ == "__main__":
    main()
