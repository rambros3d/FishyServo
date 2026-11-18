#!/usr/bin/env python3
"""
TMAG5273 3D Hall Magnetic Sensor Real-Time Test & Monitor
=========================================================
Monitors angle, vector magnitude, and 3-axis magnetic flux (X, Y, Z in mT, Gauss)
for manual magnet rotation & position testing.

Viewing Modes:
  - Default: Clean autoscrolling stream with color badges, arrow, magnitude gauge.
  - --live: Single-line in-place live update (no scrolling, no flicker).
  - --tui: Multi-line fixed dashboard (using flicker-free in-place redraw).
  - --raw: Plain unformatted text stream.

Usage:
  python3 tmag-test.py               # Auto-scrolling stream (default)
  python3 tmag-test.py --live        # Single-line in-place live update
  python3 tmag-test.py --tui         # Full in-place terminal dashboard
  python3 tmag-test.py --port /dev/ttyACM1  # Connect via UART serial
  python3 tmag-test.py --ch341       # Direct I2C via CH341 USB adapter
"""

import argparse
import atexit
import glob
import math
import os
import re
import signal
import struct
import sys
import time

# ANSI color codes
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
CLEAR_LINE = "\033[2K\r"
CURSOR_HOME = "\033[H"
CURSOR_HIDE = "\033[?25l"
CURSOR_SHOW = "\033[?25h"

COMPASS_ARROWS = ["↑ N ", "↗ NE", "→ E ", "↘ SE", "↓ S ", "↙ SW", "← W ", "↖ NW"]

def compass_direction(angle_deg: float) -> str:
    """Returns 8-point compass arrow and abbreviation."""
    idx = int((angle_deg + 22.5) % 360 // 45)
    return COMPASS_ARROWS[idx]

def make_gauge(val: float, min_val: float, max_val: float, width: int = 16, center_zero: bool = False) -> str:
    """Creates a visual ASCII gauge bar."""
    if center_zero:
        half = width // 2
        clamped = max(-max_val, min(max_val, val))
        pos = int((clamped / max_val) * half)
        if pos < 0:
            bar = " " * (half + pos) + "◀" + "█" * (-pos - 1) + "|" + " " * half
        elif pos > 0:
            bar = " " * half + "|" + "█" * (pos - 1) + "▶" + " " * (half - pos)
        else:
            bar = " " * half + "|" + " " * half
        return f"[{bar}]"
    else:
        clamped = max(min_val, min(max_val, val))
        frac = (clamped - min_val) / (max_val - min_val) if max_val > min_val else 0.0
        filled = int(frac * width)
        bar = "█" * filled + "░" * (width - filled)
        return f"[{bar}]"

class RttReader:
    """Reads live TMAG output directly from target RAM via pyOCD SWD."""
    def __init__(self, probe_id: str = "810F8F0605AA", target: str = "stm32h503kbux"):
        import pyocd
        from pyocd.core.helpers import ConnectHelper
        print(f"{CYAN}Connecting to STM32 target via SWD (Probe: {probe_id})...{RESET}")
        self.session = ConnectHelper.session_with_chosen_probe(unique_id=probe_id, target_override=target)
        self.session.open()
        self.target = self.session.target
        self.target.resume()

        # Locate _SEGGER_RTT structure
        self.rtt_addr = 0x200018d4
        header = self.target.read_memory_block32(self.rtt_addr, 11)
        acid = bytes([b for w in header[0:4] for b in struct.pack("<I", w)])
        if not acid.startswith(b"SEGGER RTT"):
            # Scan RAM for SEGGER RTT signature
            ram = self.target.read_memory_block8(0x20000000, 0x4000)
            ram_bytes = bytes(ram)
            idx = ram_bytes.find(b"SEGGER RTT\0")
            if idx != -1:
                self.rtt_addr = 0x20000000 + idx
                header = self.target.read_memory_block32(self.rtt_addr, 11)
            else:
                raise RuntimeError("SEGGER RTT buffer not found in RAM")

        self.p_buf = header[7]
        self.buf_size = header[8]
        # Align read pointer with current write pointer to discard stale logs
        wr_off = header[9]
        self.target.write32(self.rtt_addr + 40, wr_off)
        self.line_buf = ""

    def read_lines(self):
        try:
            header = self.target.read_memory_block32(self.rtt_addr, 11)
            wr_off = header[9]
            rd_off = header[10]
            if wr_off == rd_off:
                return []

            if wr_off > rd_off:
                data = self.target.read_memory_block8(self.p_buf + rd_off, wr_off - rd_off)
            else:
                data = self.target.read_memory_block8(self.p_buf + rd_off, self.buf_size - rd_off)
                data += self.target.read_memory_block8(self.p_buf, wr_off)

            self.target.write32(self.rtt_addr + 40, wr_off)
            text = bytes(data).decode("utf-8", errors="replace")
            self.line_buf += text
            lines = self.line_buf.split("\n")
            self.line_buf = lines[-1]
            return [l.strip() for l in lines[:-1] if l.strip()]
        except Exception:
            return []

    def close(self):
        try:
            self.target.resume()
            self.session.close()
        except Exception:
            pass

class SerialReader:
    """Reads live TMAG output from UART / CDC Serial Port."""
    def __init__(self, port: str = None, baud: int = 115200):
        import serial
        if port is None:
            ports = sorted(glob.glob("/dev/serial/by-id/*WCH-Link*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if not ports:
                raise RuntimeError("No serial port found. Plug in USB-UART adapter or specify --port")
            port = ports[0]

        print(f"{CYAN}Opening Serial Port: {port} at {baud} baud...{RESET}")
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)

    def read_lines(self):
        lines = []
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    lines.append(line)
        except Exception:
            pass
        return lines

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

class Ch341I2CReader:
    """Direct I2C reading from TMAG5273 using CH341 USB adapter."""
    def __init__(self, addr: int = 0x35):
        import usb.core
        import usb.util
        self.addr = addr
        self.dev = usb.core.find(idVendor=0x1a86, idProduct=0x5512)
        if self.dev is None:
            raise RuntimeError("CH341 USB device (1a86:5512) not found")
        try:
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except Exception:
            pass
        self.dev.set_configuration()

        # Probe bus before claiming success
        cmd = bytearray([0xAA, 0x74, 0x82, (self.addr << 1) | 0, 0x12, 0x74, 0x81, (self.addr << 1) | 1, 0xC0, 0x75, 0x00])
        self.dev.write(0x02, bytes(cmd), timeout=100)
        res = self.dev.read(0x82, 32, timeout=100)
        if len(res) == 0 or all(b == 0xFF for b in res):
            raise RuntimeError(
                f"No I2C device responding at 0x{addr:02X} via CH341 (bus floating high 0xFF).\n"
                f"NOTE: The TMAG5273 is wired to the STM32 microcontroller (PB6 SCL / PB7 SDA).\n"
                f"Run 'python3 tmag-test.py' directly without --ch341 to read from STM32 SWD RTT!"
            )

        # Step 1: Standby mode
        self.write_reg(0x01, 0x00)
        # Step 2: 1x average, standard read
        self.write_reg(0x00, 0x00)
        # Step 3: Enable X, Y, Z
        self.write_reg(0x02, 0x70)
        # Step 4: Enable X-Y angle (40 mT range)
        self.write_reg(0x03, 0x04)
        # Step 5: Continuous mode
        self.write_reg(0x01, 0x02)
        print(f"{GREEN}TMAG5273 configured via CH341 I2C at 0x{addr:02X}!{RESET}")

    def write_reg(self, reg: int, val: int):
        cmd = bytearray([0xAA, 0x74, 0x82, (self.addr << 1) | 0, reg, val, 0x75, 0x00])
        self.dev.write(0x02, bytes(cmd), timeout=100)

    def read_data(self):
        cmd = bytearray([0xAA, 0x74, 0x82, (self.addr << 1) | 0, 0x12, 0x74, 0x81, (self.addr << 1) | 1])
        for _ in range(9):
            cmd.append(0xC1)
        cmd.extend([0xC0, 0x75, 0x00])
        self.dev.write(0x02, bytes(cmd), timeout=100)
        res = self.dev.read(0x82, 32, timeout=100)
        if len(res) < 10 or all(b == 0xFF for b in res[:10]):
            # 0xFF indicates floating lines (no device acknowledging)
            return None
        raw_x = struct.unpack(">h", bytes(res[0:2]))[0]
        raw_y = struct.unpack(">h", bytes(res[2:4]))[0]
        raw_z = struct.unpack(">h", bytes(res[4:6]))[0]
        status = res[6]
        raw_ang = (res[7] << 8) | res[8]
        mag = res[9]
        if mag == 0xFF and raw_x == -1 and raw_y == -1 and raw_z == -1:
            return None
        angle_deg = (raw_ang >> 4) + (raw_ang & 0x0F) * 0.0625
        x_mt = raw_x * (40.0 / 32768.0)
        y_mt = raw_y * (40.0 / 32768.0)
        z_mt = raw_z * (40.0 / 32768.0)
        return {
            "angle": angle_deg,
            "mag": mag,
            "x": x_mt,
            "y": y_mt,
            "z": z_mt,
            "rx": raw_x,
            "ry": raw_y,
            "rz": raw_z,
            "stat": status,
        }

    def read_lines(self):
        d = self.read_data()
        if d:
            line = f"TMAG: ang={d['angle']:.1f} mag={d['mag']} x={d['x']:.3f} y={d['y']:.3f} z={d['z']:.3f} rx={d['rx']} ry={d['ry']} rz={d['rz']} stat=0x{d['stat']:02x}"
            return [line]
        return []

    def close(self):
        pass

def parse_tmag_line(line: str):
    """Parses standard TMAG streaming line into a dictionary."""
    if "TMAG" not in line:
        return None
    try:
        ang_m = re.search(r"ang=([-\d\.]+)", line)
        mag_m = re.search(r"mag=(\d+)", line)
        x_m = re.search(r"x=([-\d\.]+)", line)
        y_m = re.search(r"y=([-\d\.]+)", line)
        z_m = re.search(r"z=([-\d\.]+)", line)
        rx_m = re.search(r"rx=([-\d]+)", line)
        ry_m = re.search(r"ry=([-\d]+)", line)
        rz_m = re.search(r"rz=([-\d]+)", line)
        stat_m = re.search(r"stat=0x([0-9a-fA-F]+)", line)

        if ang_m and mag_m and x_m and y_m and z_m:
            mag_val = int(mag_m.group(1))
            x_val = float(x_m.group(1))
            y_val = float(y_m.group(1))
            z_val = float(z_m.group(1))

            # Detect floating I2C bus (all 0xFF bytes)
            if mag_val == 255 and abs(x_val) < 0.005 and abs(y_val) < 0.005 and abs(z_val) < 0.005:
                return None

            return {
                "angle": float(ang_m.group(1)) % 360.0,
                "mag": mag_val,
                "x": x_val,
                "y": y_val,
                "z": z_val,
                "rx": int(rx_m.group(1)) if rx_m else int(x_val * 819.2),
                "ry": int(ry_m.group(1)) if ry_m else int(y_val * 819.2),
                "rz": int(rz_m.group(1)) if rz_m else int(z_val * 819.2),
                "stat": int(stat_m.group(1), 16) if stat_m else 0,
            }
    except Exception:
        pass
    return None

def format_stream_line(d, count: int) -> str:
    """Formats one compact, beautiful scrolling line."""
    ang = d["angle"]
    mag = d["mag"]
    x = d["x"]
    y = d["y"]
    z = d["z"]
    direction = compass_direction(ang)

    # 8-segment magnitude bar
    filled = min(8, int((mag / 40.0) * 8))
    mag_bar = "█" * filled + "░" * (8 - filled)

    # Proportional LED brightness (12 mT full scale)
    x_mag = max(0.0, abs(x) - 0.25)
    y_mag = max(0.0, abs(y) - 0.25)
    blue_pct = int(min(1.0, x_mag / 12.0) * 100)
    red_pct = int(min(1.0, y_mag / 12.0) * 100)
    led_info = f"{BLUE}LED_B(X):{blue_pct:3d}%{RESET} {RED}LED_R(Y):{red_pct:3d}%{RESET}"

    total_field = math.sqrt(x*x + y*y + z*z)
    if total_field > 5.0 or mag > 10:
        badge = f"{GREEN}{BOLD}★ MAGNET DETECTED{RESET}"
    elif total_field > 1.2 or mag > 2:
        badge = f"{YELLOW}▲ WEAK FIELD{RESET}"
    else:
        badge = f"{DIM}· ambient{RESET}"

    return (
        f"[{count:5d}]  "
        f"{BOLD}{YELLOW}{ang:5.1f}°{RESET} {MAGENTA}{direction}{RESET}  "
        f"Mag:{mag:2d} [{CYAN}{mag_bar}{RESET}]  "
        f"X:{x:+6.2f} Y:{y:+6.2f} Z:{z:+6.2f} mT  "
        f"|  {led_info}  "
        f"|  {badge}"
    )

def print_stream_header():
    """Prints column header for auto-scrolling stream."""
    print(f"\n{BOLD}{BLUE}===================================================================================================={RESET}")
    print(f"{BOLD}Sample   Angle & Compass   Magnitude    3-Axis Field (X, Y, Z in mT)     Proportional LEDs     Status{RESET}")
    print(f"{BOLD}{BLUE}===================================================================================================={RESET}")

def render_tui_dashboard(data, sample_count: int, fps: float):
    """Draws a fixed in-place dashboard without screen-clearing flicker."""
    ang = data["angle"]
    mag = data["mag"]
    x = data["x"]
    y = data["y"]
    z = data["z"]
    rx = data["rx"]
    ry = data["ry"]
    rz = data["rz"]
    direction = compass_direction(ang)

    dial_pos = int((ang / 360.0) * 36)
    dial = ["─"] * 36
    dial[min(35, dial_pos)] = f"{YELLOW}◆{RESET}{CYAN}"
    dial_str = "".join(dial)

    mag_bar = make_gauge(mag, 0, 60, width=24)
    x_bar = make_gauge(x, -20.0, 20.0, width=16, center_zero=True)
    y_bar = make_gauge(y, -20.0, 20.0, width=16, center_zero=True)
    z_bar = make_gauge(z, -20.0, 20.0, width=16, center_zero=True)

    total_field = math.sqrt(x*x + y*y + z*z)
    if total_field > 5.0 or mag > 10:
        prox_badge = f"{GREEN}{BOLD}STRONG MAGNET DETECTED!{RESET}"
    elif total_field > 1.2 or mag > 2:
        prox_badge = f"{YELLOW}{BOLD}WEAK / DISTANT MAGNET DETECTED{RESET}"
    else:
        prox_badge = f"{DIM}NO MAGNET IN RANGE (Ambient: {total_field*10.0:4.1f} Gauss){RESET}"

    lines = [
        f"{BOLD}{BLUE}========================================================================{RESET}\033[K",
        f"{BOLD}{WHITE}   TMAG5273 3D Magnetic Sensor Live Monitor  {RESET}{DIM}(N20-Servo / STM32H503){RESET}\033[K",
        f"{BOLD}{BLUE}========================================================================{RESET}\033[K",
        f"  Rate: {CYAN}{fps:4.1f} Hz{RESET}  |  Samples: {WHITE}{sample_count}{RESET}  |  Status: {prox_badge}\033[K",
        f"────────────────────────────────────────────────────────────────────────\033[K",
        f"{BOLD}  1. ROTATIONAL ANGLE (X-Y Plane Tracking):{RESET}\033[K",
        f"    Angle:       {BOLD}{YELLOW}{ang:6.1f}°{RESET}    Compass: {BOLD}{MAGENTA}{direction}{RESET}\033[K",
        f"    Dial:   0° [{dial_str}] 360°\033[K",
        f"\033[K",
        f"{BOLD}  2. MAGNETIC VECTOR MAGNITUDE (CORDIC Vector Length):{RESET}\033[K",
        f"    Magnitude:   {BOLD}{WHITE}{mag:3d}{RESET} (CORDIC)   {mag_bar}\033[K",
        f"    Total |B|:   {BOLD}{WHITE}{total_field:6.3f} mT{RESET}   ({total_field * 10.0:6.1f} Gauss)\033[K",
        f"\033[K",
        f"{BOLD}  3. 3-AXIS MAGNETIC FLUX DENSITY (Range: ±40 mT | 820 LSB/mT):{RESET}\033[K",
        f"    X-Axis:     {BOLD}{x:+7.3f} mT{RESET}  {x_bar}  {DIM}({x*10:+6.1f} G | {rx:+6d} LSB){RESET}\033[K",
        f"    Y-Axis:     {BOLD}{y:+7.3f} mT{RESET}  {y_bar}  {DIM}({y*10:+6.1f} G | {ry:+6d} LSB){RESET}",
        f"    Z-Axis:     {BOLD}{z:+7.3f} mT{RESET}  {z_bar}  {DIM}({z*10:+6.1f} G | {rz:+6d} LSB){RESET}\033[K",
        f"{BOLD}{BLUE}========================================================================{RESET}\033[K",
        f"{DIM}  Tip: Hold magnet within 2-5mm of TMAG IC to see angle track.{RESET}\033[K",
        f"{DIM}  Press Ctrl+C to exit.{RESET}\033[K",
    ]

    # Move cursor to home and write lines without full screen clear (flicker-free)
    sys.stdout.write(CURSOR_HOME + "\n".join(lines) + "\n")
    sys.stdout.flush()

def main():
    parser = argparse.ArgumentParser(description="TMAG5273 3D Magnetic Sensor Real-Time Monitor")
    parser.add_argument("--rtt", action="store_true", help="Use SWD RTT over WCH-Link (recommended)")
    parser.add_argument("--stream", action="store_true", help="Auto-scrolling stream (default)")
    parser.add_argument("--live", action="store_true", help="Single-line in-place live update (no scrolling)")
    parser.add_argument("--tui", action="store_true", help="Fixed in-place multi-line dashboard")
    parser.add_argument("--port", type=str, default=None, help="Serial port to read (e.g. /dev/ttyACM1)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--ch341", action="store_true", help="Direct I2C via CH341 USB adapter")
    parser.add_argument("--raw", action="store_true", help="Plain text unformatted stream")
    args = parser.parse_args()

    reader = None

    def cleanup():
        sys.stdout.write(CURSOR_SHOW + "\n")
        sys.stdout.flush()
        if reader:
            reader.close()

    atexit.register(cleanup)

    try:
        if args.ch341:
            reader = Ch341I2CReader()
        elif args.port:
            reader = SerialReader(port=args.port, baud=args.baud)
        else:
            try:
                reader = RttReader()
            except Exception as e:
                print(f"{RED}SWD RTT connection failed: {e}{RESET}")
                print(f"{YELLOW}Hint: Ensure WCH-Link probe is plugged in. To use UART serial instead, use --port <DEV>.{RESET}")
                sys.exit(1)

        # Default mode is autoscrolling stream unless --live or --tui or --raw is chosen
        mode = "stream"
        if args.live:
            mode = "live"
        elif args.tui:
            mode = "tui"
            sys.stdout.write("\033[2J" + CURSOR_HIDE) # Clear once at start
            sys.stdout.flush()
        elif args.raw:
            mode = "raw"
        else:
            mode = "stream"
            print_stream_header()

        sample_count = 0
        last_tui_render = 0.0
        last_header_count = 0
        last_time = time.time()
        fps = 0.0
        latest_data = None

        while True:
            lines = reader.read_lines()
            if not lines:
                time.sleep(0.01)
                continue

            now = time.time()
            dt = now - last_time
            if dt >= 0.5:
                fps = len(lines) / dt
                last_time = now

            for line in lines:
                data = parse_tmag_line(line)
                if data:
                    sample_count += 1
                    latest_data = data

                    if mode == "stream":
                        # Print header periodically every 40 samples
                        if sample_count - last_header_count >= 40:
                            print_stream_header()
                            last_header_count = sample_count
                        print(format_stream_line(data, sample_count))
                    elif mode == "live":
                        # Single-line in-place update using \r
                        s = format_stream_line(data, sample_count)
                        sys.stdout.write(f"{CLEAR_LINE}{s}")
                        sys.stdout.flush()
                    elif mode == "raw":
                        print(f"[{sample_count}] ANG={data['angle']:5.1f}° MAG={data['mag']:2d} X={data['x']:+7.3f} Y={data['y']:+7.3f} Z={data['z']:+7.3f}")
                elif mode == "raw":
                    print(line)

            # In TUI mode, render once per frame (15 FPS max) using the latest sample
            if mode == "tui" and latest_data and (now - last_tui_render) >= 0.06:
                render_tui_dashboard(latest_data, sample_count, fps)
                last_tui_render = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print(f"\n{GREEN}Monitoring stopped by user.{RESET}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
