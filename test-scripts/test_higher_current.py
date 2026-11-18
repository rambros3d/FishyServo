#!/usr/bin/env python3
"""Progressive higher-current testing up to 200mA for N20-Servo."""

import time
from pyocd.core.helpers import ConnectHelper

def main():
    probe_id = "A1A48F062680"
    target_type = "stm32h503kbux"

    print("=======================================================")
    print("   N20-SERVO PROGRESSIVE HIGHER-CURRENT TEST (UP TO 200mA)")
    print("=======================================================")

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
    print(f"SEGGER RTT block at 0x{rtt_cb_addr:08X}\n")

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

    drain_up()
    time.sleep(0.1)

    voltages = [3.5, 4.0, 4.5, 5.0]

    for v in voltages:
        print(f"\n=======================================================")
        print(f"   STEP: Testing ±{v:.1f} V (Targeting higher current)")
        print(f"=======================================================")

        # Forward
        print(f"\n--- Driving Forward +{v:.1f}V for 3.0s ---")
        send_cmd(f"u {v}")
        max_i_fwd = 0.0
        start = time.time()
        while time.time() - start < 3.0:
            time.sleep(0.1)
            txt = drain_up()
            if txt:
                print(txt, end="", flush=True)
                for line in txt.splitlines():
                    if "I=" in line:
                        try:
                            val = float(line.split("I=")[1].split(" mA")[0].strip())
                            if val > max_i_fwd:
                                max_i_fwd = val
                        except Exception:
                            pass

        # Coast neutral
        send_cmd("u 0.0")
        time.sleep(0.3)
        drain_up()
        print(f"-> Peak FWD current at +{v:.1f}V: {max_i_fwd:.1f} mA")

        # Reverse
        print(f"\n--- Driving Reverse -{v:.1f}V for 3.0s ---")
        send_cmd(f"u -{v}")
        max_i_rev = 0.0
        start = time.time()
        while time.time() - start < 3.0:
            time.sleep(0.1)
            txt = drain_up()
            if txt:
                print(txt, end="", flush=True)
                for line in txt.splitlines():
                    if "I=" in line:
                        try:
                            val = float(line.split("I=")[1].split(" mA")[0].strip())
                            if val > max_i_rev:
                                max_i_rev = val
                        except Exception:
                            pass

        # Coast neutral
        send_cmd("u 0.0")
        time.sleep(0.3)
        drain_up()
        print(f"-> Peak REV current at -{v:.1f}V: {max_i_rev:.1f} mA")

    # Safe Disarm
    print("\n--- Final Safety Disarm ---")
    send_cmd("u 0.0")
    send_cmd("e 0")
    time.sleep(0.2)
    print(drain_up())

    session.close()
    print("=======================================================")
    print("Progressive test completed.")
    print("=======================================================")

if __name__ == "__main__":
    main()
