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
    if target.is_halted():
        target.resume()

    # RTT block
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

    def print_regs(tag):
        print(f"\n--- {tag} ---")
        gpioa_moder = target.read32(0x42020000)
        gpioa_afrh  = target.read32(0x42020000 + 0x24)
        pa9_mode = (gpioa_moder >> (9 * 2)) & 3
        pa9_af   = (gpioa_afrh >> ((9 - 8) * 4)) & 0xF
        print(f"PA9  (IN_D1): MODER={pa9_mode} (2=AF), AF={pa9_af} (should be 1)")

        gpiob_moder = target.read32(0x42020400)
        gpiob_afrl  = target.read32(0x42020400 + 0x20)
        pb0_mode = (gpiob_moder >> (0 * 2)) & 3
        pb0_af   = (gpiob_afrl >> (0 * 4)) & 0xF
        print(f"PB0  (IN_D2): MODER={pb0_mode} (2=AF), AF={pb0_af} (should be 2)")

        tim1_cr1  = target.read32(0x40012c00 + 0x00)
        tim1_bdtr = target.read32(0x40012c00 + 0x44)
        tim1_ccer = target.read32(0x40012c00 + 0x20)
        tim1_arr  = target.read32(0x40012c00 + 0x2c)
        tim1_ccr2 = target.read32(0x40012c00 + 0x38)
        tim1_cnt  = target.read32(0x40012c00 + 0x24)
        print(f"TIM1: CR1=0x{tim1_cr1:04X} (CEN={tim1_cr1&1}), BDTR=0x{tim1_bdtr:08X} (MOE={(tim1_bdtr>>15)&1}), CCER=0x{tim1_ccer:04X} (CC2E={(tim1_ccer>>4)&1}), ARR={tim1_arr}, CCR2={tim1_ccr2}, CNT={tim1_cnt}")

        tim3_cr1  = target.read32(0x40000400 + 0x00)
        tim3_ccer = target.read32(0x40000400 + 0x20)
        tim3_arr  = target.read32(0x40000400 + 0x2c)
        tim3_ccr3 = target.read32(0x40000400 + 0x3c)
        tim3_cnt  = target.read32(0x40000400 + 0x24)
        print(f"TIM3: CR1=0x{tim3_cr1:04X} (CEN={tim3_cr1&1}), CCER=0x{tim3_ccer:04X} (CC3E={(tim3_ccer>>8)&1}), ARR={tim3_arr}, CCR3={tim3_ccr3}, CNT={tim3_cnt}")

    print_regs("INITIAL IDLE STATE")

    print("\n>>> Setting u 3.0 (Forward 3.0V)...")
    send_cmd("u 3.0")
    time.sleep(0.5)
    print_regs("DURING u 3.0 (FWD)")
    print("RTT output during FWD:")
    print(drain_up())

    print("\n>>> Setting u -3.0 (Reverse 3.0V)...")
    send_cmd("u -3.0")
    time.sleep(0.5)
    print_regs("DURING u -3.0 (REV)")
    print("RTT output during REV:")
    print(drain_up())

    print("\n>>> Setting u 0.0 & e 0 (STOP)...")
    send_cmd("u 0.0")
    send_cmd("e 0")
    time.sleep(0.2)
    print_regs("AFTER STOP")

    session.close()

if __name__ == "__main__":
    main()
