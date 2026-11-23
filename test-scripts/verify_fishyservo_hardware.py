#!/usr/bin/env python3
"""
Hardware Verification Suite for FishyServo (STM32H503 + TMAG5273)
Communicates via AgentLA I3C/I2C Bus Bridge (10.10.10.10).
Pulsing nRST via CMSIS-DAP HID (/dev/hidraw5).
"""

import sys
import time
import json
import struct
import urllib.request

AGENTLA_HOST = "http://10.10.10.10"
PROBE_PATH = "/dev/hidraw5"
TARGET_ADDR = 0x40

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


def xfer(hex_data, read_len=0, timeout=2.0):
    url = f"{AGENTLA_HOST}/api/v1/bus/i2c/transfer"
    payload = json.dumps({
        "addr": TARGET_ADDR,
        "data_hex": hex_data,
        "read_len": read_len
    }).encode("utf-8")
    req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def read_u8(reg):
    res = xfer(f"{reg:02x}", 1)
    if res.get("status") == "ok" and res.get("rx_hex"):
        return int(res["rx_hex"], 16)
    return None


def read_f32(reg):
    res = xfer(f"{reg:02x}", 4)
    if res.get("status") == "ok" and res.get("rx_hex"):
        b = bytes.fromhex(res["rx_hex"])
        if len(b) == 4:
            return struct.unpack("<f", b)[0]
    return None


def write_u8(reg, val):
    data_hex = f"{reg:02x}{val:02x}"
    res = xfer(data_hex, 0)
    return res.get("status") == "ok"


def write_f32(reg, val):
    b = struct.pack("<f", val)
    data_hex = f"{reg:02x}{b.hex()}"
    res = xfer(data_hex, 0)
    return res.get("status") == "ok"


def pulse_hardware_reset():
    """Pulse nRST via CMSIS-DAP HID (/dev/hidraw5)."""
    with open(PROBE_PATH, "r+b", buffering=0) as f:
        def send_dap(cmd_bytes):
            p = bytes([0x00]) + cmd_bytes
            p = p + bytes(65 - len(p))
            f.write(p[:65])
            return f.read(64)

        send_dap(bytes([0x10, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00]))
        time.sleep(0.05)
        send_dap(bytes([0x10, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00]))
        time.sleep(0.1)


def check_i3c_availability():
    """Check if AgentLA I3C controller is ready and probe target."""
    try:
        req = urllib.request.Request(f"{AGENTLA_HOST}/api/v1/bus/i3c/init", data=b"{}", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            if data.get("status") == "ok":
                print(f"[*] AgentLA I3C Controller active at {data.get('freq_khz', 1000)} kHz.")
                # Probe DAA / CCC
                daa_req = urllib.request.Request(f"{AGENTLA_HOST}/api/v1/bus/i3c/entdaa", data=b"{}", headers={"Content-Type": "application/json"})
                try:
                    with urllib.request.urlopen(daa_req, timeout=1.0) as daa_resp:
                        print(f"[*] I3C ENTDAA: {daa_resp.read().decode('utf-8')}")
                        return "i3c"
                except Exception:
                    print("[*] Target is operating in legacy I2C fallback mode (I2C2 @ 0x40).")
    except Exception as e:
        print(f"[*] I3C controller probe: {e}. Falling back to I2C.")

    # Re-arm I2C bridge
    try:
        req = urllib.request.Request(f"{AGENTLA_HOST}/api/v1/bridge/i2c/on", data=b"{}", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5):
            pass
    except Exception:
        pass
    return "i2c"


def scan_i2c_bus():
    req = urllib.request.Request(f"{AGENTLA_HOST}/api/v1/bus/i2c/scan")
    with urllib.request.urlopen(req, timeout=2.0) as resp:
        data = json.loads(resp.read().decode("utf-8"))
        return data.get("addresses", [])


def run_test_suite():
    print("===============================================================")
    print("        FISHYSERVO HARDWARE VERIFICATION TEST SUITE            ")
    print("===============================================================")

    # 1. Bus Capability Check (I3C with I2C fallback)
    bus_mode = check_i3c_availability()
    print(f"[*] Active Bus Protocol: {bus_mode.upper()} (Target Address: 0x{TARGET_ADDR:02X})")

    # 2. Bus Scan
    addrs = scan_i2c_bus()
    print(f"[*] I2C Bus Scan: found {len(addrs)} devices: {addrs}")
    assert TARGET_ADDR in addrs, f"Target 0x{TARGET_ADDR:02X} not detected on I2C bus!"

    # 3. Initial Telemetry & Sensor Check
    status = read_u8(REG_STATUS)
    init_angle = read_f32(REG_ANGLE)
    init_vel = read_f32(REG_VELOCITY)
    print(f"[*] Initial Status: {status} (Expected 4 = Ready)")
    print(f"[*] Initial Shaft Angle: {init_angle:.4f} rad ({init_angle * 180.0 / 3.14159265:.2f} deg)")
    print(f"[*] Initial Shaft Velocity: {init_vel:.4f} rad/s")
    assert status == 4, f"Target status unexpected: {status}"

    # 4. Closed-Loop Position / MultiBus Read-Write Verification
    print("\n---------------------------------------------------------------")
    print(" TEST PHASE 1: Closed-Loop MultiBus Gain Regulation")
    print("---------------------------------------------------------------")
    test_kp = 28.50
    print(f"[*] Writing AngPidP = {test_kp:.2f} to register 0x{REG_ANG_PID_P:02X}...")
    write_f32(REG_ANG_PID_P, test_kp)
    rb_kp = read_f32(REG_ANG_PID_P)
    print(f"[*] AngPidP readback: {rb_kp:.2f}")
    assert abs(rb_kp - test_kp) < 0.05, f"AngPidP write failed: got {rb_kp}, expected {test_kp}"
    print("[*] MultiBus register read/write verified.")

    # 5. "Flash Once and Use" Cold-Boot Flash Persistence Verification
    print("\n---------------------------------------------------------------")
    print(" TEST PHASE 2: 'Flash Once and Use' Cold-Boot Flash Persistence")
    print("---------------------------------------------------------------")
    persist_kp = 33.25
    print(f"[*] Staging test parameter AngPidP = {persist_kp:.2f} into MultiBus...")
    write_f32(REG_ANG_PID_P, persist_kp)
    assert abs(read_f32(REG_ANG_PID_P) - persist_kp) < 0.05

    print(f"[*] Triggering Flash commit (0x{CMD_SAVE_CONFIG:02X} -> reg 0x{REG_SAVE_CONFIG:02X}) to Bank 2 Flash Sectors 14 & 15...")
    write_u8(REG_SAVE_CONFIG, CMD_SAVE_CONFIG)
    time.sleep(0.20)  # Allow Bank 2 flash page erase and quad-word write to complete

    print("[*] Asserting hardware nRST via CMSIS-DAP HID to simulate hard cold-boot power cycle...")
    pulse_hardware_reset()

    print("[*] Waiting 0.6s for cold-boot startup, TMAG5273 auto-discovery, and setpoint hydration...")
    time.sleep(0.6)

    # Reconnect and verify persistence
    addrs_after = scan_i2c_bus()
    print(f"[*] Post-reset I2C Bus Scan: {addrs_after}")
    assert TARGET_ADDR in addrs_after, "Target did not respond at 0x40 after cold boot!"

    boot_status = read_u8(REG_STATUS)
    boot_angle = read_f32(REG_ANGLE)
    boot_kp = read_f32(REG_ANG_PID_P)

    print(f"\n[*] Cold-Boot Hydration Report:")
    print(f"    - Status: {boot_status} (Expected 4 = Ready)")
    print(f"    - Shaft Angle: {boot_angle:.4f} rad ({boot_angle * 180.0 / 3.14159265:.2f} deg)")
    print(f"    - Hydrated AngPidP from Flash: {boot_kp:.2f} (Expected {persist_kp:.2f})")

    assert boot_status == 4, f"Boot status {boot_status} is not Ready!"
    assert abs(boot_kp - persist_kp) < 0.05, f"AngPidP did not persist across reboot! Got {boot_kp}, expected {persist_kp}"

    print("\n===============================================================")
    print("  VERIFICATION RESULT: ALL HARDWARE TESTS PASSED (100% OK)")
    print("  BareFOC is confirmed: True 'Flash Once and Use' Autonomy.")
    print("===============================================================")


if __name__ == "__main__":
    run_test_suite()
