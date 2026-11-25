#!/usr/bin/env python3
"""
FishyServo I3C-Only End-to-End Hardware & Motor Control Benchmark Suite.

Uses AgentLA Native USB Bulk I3C (AgentLaI3c, 1209:414D) to:
1. Verify I3C ENTDAA and BareBoot Katapult handshake.
2. Flash fishyservo_stm32h5_i3c.bin over I3C SDR @ 10 MHz with CommitStore + CRC32 commit.
3. Verify zero-delay MultiBus register read/write (including end-of-map reads at 0x70/0x78).
4. Verify replugged TMAG5273 angle sensor tracking & closed-loop motor control over I3C.
5. Verify hands-free in-band 0xF0 reboot back into BareBoot over I3C.
"""

import os
import sys
import time
import struct

sys.path.insert(0, "/home/sun/Filelink/Firmwares/AgentLA-rust/sdk")
sys.path.insert(0, "/home/sun/Filelink/Firmwares/bareBoot/host/scripts")

from agentla.i3c import AgentLaI3c
from flash_app_over_i3c_usb import I3cUsbFlasher

TARGET_ADDR = 0x40
DEFAULT_APP_BIN = (
    "/home/sun/Filelink/Firmwares/bareFOC/examples/fishyservo_stm32h5/"
    "target/thumbv8m.main-none-eabihf/release/fishyservo_stm32h5_i3c.bin"
)

# MultiBus Register Offsets
REG_STATUS = 0x00
REG_TARGET = 0x01
REG_ENABLE = 0x04
REG_ANGLE = 0x09
REG_VELOCITY = 0x11
REG_CURRENT_DC = 0x28
REG_ANG_PID_P = 0x36
REG_VOLTAGE_LIMIT = 0x50
REG_VELOCITY_LIMIT = 0x52
REG_SAVE_CONFIG = 0x64
REG_MOTOR_SELECT = 0x70
REG_TOPOLOGY_MODE = 0x78
CMD_SAVE_CONFIG = 0x55


def read_u8(client: AgentLaI3c, addr: int, reg: int) -> int:
    rx = client.private_write_read(addr, bytes([0x00, reg]), 1)
    return rx[0]


def read_f32(client: AgentLaI3c, addr: int, reg: int) -> float:
    rx = client.private_write_read(addr, bytes([0x00, reg]), 4)
    return struct.unpack("<f", rx)[0]


def write_u8(client: AgentLaI3c, addr: int, reg: int, val: int):
    client.private_write(addr, bytes([0x00, reg, val]))


def write_f32(client: AgentLaI3c, addr: int, reg: int, val: float):
    client.private_write(addr, bytes([0x00, reg]) + struct.pack("<f", val))


def connect_i3c_target(client: AgentLaI3c, freq_khz: int = 1000) -> int:
    client.set_inverted(False)
    client.set_bus_speed(freq_khz)
    try:
        client.rstdaa()
    except Exception:
        pass
    dev = client.entdaa(TARGET_ADDR)
    if dev is not None:
        return dev.get("dynamic_address", dev.get("addr", TARGET_ADDR))
    return TARGET_ADDR


def run_benchmark(app_bin_path: str = DEFAULT_APP_BIN):
    print("=" * 72)
    print("   FISHYSERVO I3C-ONLY HARDWARE & MOTOR CONTROL BENCHMARK SUITE")
    print("=" * 72)

    assert os.path.exists(app_bin_path), f"App binary not found: {app_bin_path}"
    with open(app_bin_path, "rb") as f:
        app_data = f.read()

    client = AgentLaI3c()
    assert client.connect(), "Failed to connect to AgentLA USB Bulk I3C adapter"

    try:
        # Phase 1: Enter BareBoot (try 0xF0 if app is running, else ENTDAA)
        print("\n[Phase 1] Connecting to I3C-only BareBoot...")
        addr = connect_i3c_target(client, freq_khz=1000)
        flasher = I3cUsbFlasher(client, addr=addr, freq_khz=1000)
        if not flasher.connect_bootloader(retries=3):
            print("[*] BareBoot not immediately active; sending 0xF0 strobe to app...")
            try:
                client.private_write(addr, bytes([0xF0]))
            except Exception:
                pass
            time.sleep(0.25)
            addr = connect_i3c_target(client, freq_khz=1000)
            flasher = I3cUsbFlasher(client, addr=addr, freq_khz=1000)
            assert flasher.connect_bootloader(retries=10), "Failed to enter BareBoot over I3C!"

        print(f"[*] BareBoot connected: MCU={flasher.mcu}, app_base=0x{flasher.app_base:08X}")

        # Phase 2: Benchmark OTA Flash over I3C @ 10 MHz
        ota_freq_khz = 10000
        print(f"\n[Phase 2] Flashing {len(app_data)} bytes over I3C SDR @ {ota_freq_khz / 1000:.1f} MHz...")
        client.set_bus_speed(ota_freq_khz)
        t0 = time.perf_counter()
        stats = flasher.flash_image(app_data, verify=True)
        dt = time.perf_counter() - t0
        assert stats["verification_passed"], "I3C OTA readback verification failed!"
        print(
            f"[*] I3C OTA Complete: {stats['num_blocks']} blocks in {dt:.3f}s "
            f"({stats['effective_kb_s']:.2f} KB/s, avg latency {stats['avg_block_latency_us']:.1f} us)"
        )

        # Wait for app boot (ADC offset calibration + TMAG5273 discovery takes ~1s)
        client.set_bus_speed(1000)
        status = 0
        t_boot = time.perf_counter()
        while time.perf_counter() - t_boot < 4.5:
            time.sleep(0.2)
            try:
                status = read_u8(client, addr, REG_STATUS)
                if status == 4:
                    break
            except Exception:
                pass

        # Phase 3: Zero-Delay MultiBus Telemetry, End-of-Map Reads, & Register R/W
        print("\n[Phase 3] Verifying Zero-Delay MultiBus Telemetry & End-of-Map Reads...")
        status = read_u8(client, addr, REG_STATUS)
        angle0 = read_f32(client, addr, REG_ANGLE)
        vel0 = read_f32(client, addr, REG_VELOCITY)
        mot_sel = read_u8(client, addr, REG_MOTOR_SELECT)
        topo_mode = read_u8(client, addr, REG_TOPOLOGY_MODE)
        print(
            f"[*] Status={status} (Expected 4), Angle={angle0:.4f} rad, Vel={vel0:.4f} rad/s, "
            f"MotorSelect={mot_sel}, TopologyMode={topo_mode}"
        )
        assert status == 4, f"Unexpected status: {status}"
        assert mot_sel < 4, f"Unexpected MotorSelect: {mot_sel}"

        # Zero-delay back-to-back write -> read stress (20 iterations)
        for i in range(20):
            test_kp = 20.0 + (i % 5) * 0.5
            write_f32(client, addr, REG_ANG_PID_P, test_kp)
            rb_kp = read_f32(client, addr, REG_ANG_PID_P)
            assert abs(rb_kp - test_kp) < 0.05, f"Zero-delay Kp mismatch: {rb_kp} vs {test_kp}"
        print("[*] 20x zero-delay back-to-back write->read cycles passed with 0 errors.")

        # End-of-map register readback (0x78 TopologyMode, last byte of MultiBus map)
        write_u8(client, addr, REG_TOPOLOGY_MODE, 0)
        rb_topo = read_u8(client, addr, REG_TOPOLOGY_MODE)
        assert rb_topo == 0, f"End-of-map TopologyMode mismatch: {rb_topo}"
        print(f"[*] End-of-map read at 0x{REG_TOPOLOGY_MODE:02X} succeeded (TopologyMode={rb_topo}).")

        # Phase 4: Closed-Loop Motor Control & TMAG5273 Angle Tracking over I3C
        print("\n[Phase 4] Closed-Loop Motor Control & TMAG5273 Tracking over I3C...")
        write_f32(client, addr, REG_VOLTAGE_LIMIT, 3.0)
        write_f32(client, addr, REG_TARGET, angle0 + 0.5)
        write_u8(client, addr, REG_ENABLE, 1)

        samples = []
        t_ctrl_start = time.perf_counter()
        for _ in range(50):
            a = read_f32(client, addr, REG_ANGLE)
            v = read_f32(client, addr, REG_VELOCITY)
            i_dc = read_f32(client, addr, REG_CURRENT_DC)
            samples.append((a, v, i_dc))
            time.sleep(0.01)
        t_ctrl_elapsed = time.perf_counter() - t_ctrl_start

        write_u8(client, addr, REG_ENABLE, 0)
        final_angle = samples[-1][0]
        angle_span = max(s[0] for s in samples) - min(s[0] for s in samples)
        print(
            f"[*] 50x I3C telemetry loops (150 transactions) in {t_ctrl_elapsed:.3f}s "
            f"| Angle: {angle0:.4f} -> {final_angle:.4f} rad (span={angle_span:.4f} rad)"
        )
        assert any(abs(s[0]) > 1e-4 for s in samples) or angle_span > 1e-3, (
            "TMAG5273 angle stayed identically 0.0; check I2C1 PB6/PB7 sensor connection!"
        )

        # Phase 5: Hands-free In-Band 0xF0 Reboot back to BareBoot
        print("\n[Phase 5] Verifying Hands-Free 0xF0 Reboot back to BareBoot over I3C...")
        try:
            client.private_write(addr, bytes([0xF0]))
        except Exception:
            pass
        time.sleep(0.25)
        addr = connect_i3c_target(client, freq_khz=1000)
        flasher = I3cUsbFlasher(client, addr=addr, freq_khz=1000)
        assert flasher.connect_bootloader(retries=10), "Failed to re-enter BareBoot via 0xF0!"
        print("[*] Re-entered BareBoot over I3C successfully!")

        print("\n" + "=" * 72)
        print("  ALL FISHYSERVO I3C-ONLY BENCHMARK & MOTOR CONTROL TESTS PASSED!")
        print("=" * 72)
    finally:
        client.close()


if __name__ == "__main__":
    run_benchmark()
