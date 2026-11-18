#!/usr/bin/env python3
import time
from pyocd.core.helpers import ConnectHelper

def main():
    probe_id = "A1A48F062680"
    target_type = "stm32h503kbux"

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id,
        target_override=target_type,
        options={"connect_mode": "attach"}
    )
    session.open()
    target = session.target

    rtt_cb_addr = 0x20001c30
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

    for v in [3.0, 3.5, 4.0]:
        print(f"\n================ DRIVE TEST: +{v}V ================")
        send_cmd(f"u {v}")
        time.sleep(0.8)
        ccr2 = target.read32(0x40012c00 + 0x38)
        ccr3 = target.read32(0x40000400 + 0x3c)
        print(f"HW Regs at +{v}V: TIM1_CCR2={ccr2}, TIM3_CCR3={ccr3}")
        print("RTT:", drain_up().strip())

        print(f"\n================ DRIVE TEST: -{v}V ================")
        send_cmd(f"u -{v}")
        time.sleep(0.8)
        ccr2 = target.read32(0x40012c00 + 0x38)
        ccr3 = target.read32(0x40000400 + 0x3c)
        print(f"HW Regs at -{v}V: TIM1_CCR2={ccr2}, TIM3_CCR3={ccr3}")
        print("RTT:", drain_up().strip())

    print("\n================ STOP & DISARM ================")
    send_cmd("u 0.0")
    send_cmd("e 0")
    time.sleep(0.2)
    print("RTT:", drain_up().strip())

    session.close()

if __name__ == "__main__":
    main()
