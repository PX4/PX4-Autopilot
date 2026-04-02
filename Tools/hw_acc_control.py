#!/usr/bin/env python3
"""
hw_acc_control.py
=================
Điều khiển gia tốc ngoài (acc_sp_external) trên drone thật.

Kiến trúc phần cứng:
  Jetson /dev/ttyACM0 (USB) ──MAVLink──► Pixhawk   (acc commands)
  Jetson /dev/ttyUSB0 (USB) ──0xAA55──► STM32 ──SBUS──► Pixhawk RC input

Flow an toàn (Option A — phi công làm chủ):
  1. Phi công: arm + takeoff bằng MK15
  2. Phi công: flip trigger trên MK15
       → STM32 chuyển SBUS sang Jetson mode
  3. Script detect POSCTL/ALTCTL → bắt đầu gửi acc commands
  4. Phi công cần lấy lại: flip trigger lại
       → STM32 chuyển về AirUnit SBUS
       → watchdog firmware 500ms fired → drone brake tự động

Chạy demo:
  python3 hw_acc_control.py
  python3 hw_acc_control.py --drone serial:///dev/ttyACM0:57600 --rc-port /dev/ttyUSB0
"""

import csv
import math
import os
import time
import threading
import argparse
from datetime import datetime
from typing import Optional, Tuple

try:
    from pymavlink import mavutil
except ImportError:
    raise ImportError("pymavlink required: pip install pymavlink")

try:
    import serial as _serial_mod
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("[WARN] pyserial not found — SBUS keep-alive sẽ bị tắt")

# CRC16-Modbus (lookup table) — khớp với crc.py trên Jetson
_CRC16_HI = [
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
]
_CRC16_LO = [
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,0x04,
    0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,0x08,0xC8,
    0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,
    0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,0x11,0xD1,0xD0,0x10,
    0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
    0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,0x3B,0xFB,0x39,0xF9,0xF8,0x38,
    0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,
    0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,
    0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,
    0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
    0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,
    0xB4,0x74,0x75,0xB5,0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,
    0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,
    0x9C,0x5C,0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,
    0x88,0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40,
]

def _crc16(data: bytes) -> int:
    hi, lo = 0xFF, 0xFF
    for b in data:
        idx = lo ^ b
        lo  = hi ^ _CRC16_HI[idx]
        hi  = _CRC16_LO[idx]
    return (hi << 8) | lo


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_ACC_SP_EXTERNAL_FLAG = 1 << 12

# type_mask: ignore pos + vel, dùng acc fields, giữ heading
_TYPE_MASK_ACC_ONLY = (
    (1 << 0) | (1 << 1) | (1 << 2)   # ignore x, y, z
    | (1 << 3) | (1 << 4) | (1 << 5)  # ignore vx, vy, vz
    # bits 6-8 = 0 → dùng afx, afy, afz
    # bit 9 (FORCE_SET) phải = 0 — nếu set sẽ bị reject
    | (1 << 10)                        # YAW_IGNORE
    | (1 << 11)                        # YAW_RATE_IGNORE
    | _ACC_SP_EXTERNAL_FLAG
)

# type_mask: như trên nhưng có yaw setpoint
_TYPE_MASK_ACC_WITH_YAW = (
    (1 << 0) | (1 << 1) | (1 << 2)
    | (1 << 3) | (1 << 4) | (1 << 5)
    # bit 10 = 0 → dùng yaw field
    | (1 << 11)
    | _ACC_SP_EXTERNAL_FLAG
)

_MAV_FRAME_LOCAL_NED = 1

# PX4 custom mode encoding
_MODE_MAP = {
    (2, 0): "ALTCTL",
    (3, 0): "POSCTL",
    (4, 2): "TAKEOFF",
    (4, 3): "LOITER",
    (4, 4): "MISSION",
    (4, 6): "RTL",
    (4, 7): "LAND",
    (6, 0): "ACRO",
    (7, 0): "STABILIZED",
}

SBUS_RATE_HZ  = 20
SBUS_INTERVAL = 1.0 / SBUS_RATE_HZ


# ---------------------------------------------------------------------------
# SerialRC — gửi centered sticks qua STM32 (SBUS keep-alive)
# ---------------------------------------------------------------------------
# FlightLogger — ghi CSV để debug
# ---------------------------------------------------------------------------

class FlightLogger:
    """
    Ghi log CSV mỗi lần gọi write().

    Columns:
      timestamp, event,
      mode, alt_m, hdg_deg,
      vx_ned, vy_ned, vz_ned, spd_h,
      ax_cmd, ay_cmd, az_cmd
    """

    HEADER = [
        "timestamp", "event",
        "mode", "alt_m", "hdg_deg",
        "vx_ned", "vy_ned", "vz_ned", "spd_h",
        "ax_cmd", "ay_cmd", "az_cmd",
    ]

    def __init__(self, path: Optional[str] = None):
        if path is None:
            ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = f"hw_acc_{ts}.csv"
        self.path = path
        self._f   = open(path, "w", newline="")
        self._w   = csv.writer(self._f)
        self._w.writerow(self.HEADER)
        self._f.flush()
        print(f"[Logger] Log → {os.path.abspath(path)}")

    def write(self,
              mode: str, alt: float, hdg: float,
              vx: float, vy: float, vz: float,
              ax: float, ay: float, az: float,
              event: str = "") -> None:
        spd = math.sqrt(vx**2 + vy**2)
        self._w.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            event,
            mode,
            round(alt, 2), round(hdg, 1),
            round(vx, 3), round(vy, 3), round(vz, 3), round(spd, 3),
            round(ax, 3), round(ay, 3), round(az, 3),
        ])
        self._f.flush()

    def close(self):
        self._f.close()
        print(f"[Logger] Đã lưu: {os.path.abspath(self.path)}")


# ---------------------------------------------------------------------------

class SerialRC:
    """
    Gửi lệnh RC đến STM32 qua custom protocol 0xAA55.
    STM32 convert sang SBUS và forward sang Pixhawk.

    Packet (10 bytes):
      [0xAA, 0x55, roll, pitch, throttle, yaw, 0x7F, 0x02, crc_lo, crc_hi]
    Scaling:
      roll/pitch/yaw : -1..+1 → 0..254
      throttle       :  0..1  → 0..254
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.port     = port
        self.baudrate = baudrate
        self._ser     = None
        self._buf     = bytearray(10)

    def connect(self) -> bool:
        if not HAS_SERIAL:
            print("[SerialRC] pyserial không có — SBUS disabled")
            return False
        try:
            self._ser = _serial_mod.Serial(self.port, self.baudrate, timeout=1)
            print(f"[SerialRC] OK: {self.port} @ {self.baudrate}")
            return True
        except Exception as e:
            print(f"[SerialRC] Lỗi kết nối: {e}")
            return False

    def close(self):
        if self._ser:
            self._ser.close()
            self._ser = None

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def send(self, pitch: float, roll: float, throttle: float, yaw: float):
        """
        pitch, roll, yaw : -1.0 .. +1.0
        throttle         :  0.0 .. +1.0
        """
        if not self.connected:
            return
        b = self._buf
        b[0] = 0xAA
        b[1] = 0x55
        b[2] = int(((roll    + 1.0) / 2.0) * 254)
        b[3] = int(((pitch   + 1.0) / 2.0) * 254)
        b[4] = int(throttle * 254)
        b[5] = int(((yaw     + 1.0) / 2.0) * 254)
        b[6] = 0x7F
        b[7] = 0x02
        crc  = _crc16(bytes(b[:8]))
        b[8] = crc & 0xFF
        b[9] = (crc >> 8) & 0xFF
        try:
            self._ser.write(b)
        except Exception as e:
            print(f"[SerialRC] Write lỗi: {e}")

    def send_centered(self):
        """Hover sticks: tất cả = 0, throttle = 0.5."""
        self.send(0.0, 0.0, 0.5, 0.0)


# ---------------------------------------------------------------------------
# HardwareAccControl — lớp chính
# ---------------------------------------------------------------------------

class HardwareAccControl:
    """
    Điều khiển acc_sp_external trên drone thật.

    Parameters
    ----------
    drone_port  : pymavlink connection string tới Pixhawk
    rc_port     : serial port của STM32
    rc_baudrate : baudrate STM32 (mặc định 115200)
    source_system : MAVLink system ID của Jetson (mặc định 255)
    """

    def __init__(self,
                 drone_port:   str = "serial:///dev/ttyACM0:57600",
                 rc_port:      str = "/dev/ttyUSB0",
                 rc_baudrate:  int = 115200,
                 source_system: int = 255):
        self._drone_port    = drone_port
        self._source_system = source_system
        self._mav: Optional[mavutil.mavfile] = None

        self._rc = SerialRC(rc_port, rc_baudrate)

        # SBUS keep-alive thread
        self._sbus_running = False
        self._sbus_thread: Optional[threading.Thread] = None
        self._sbus_lock = threading.Lock()
        self._sbus_cmd  = (0.0, 0.0, 0.5, 0.0)  # (pitch, roll, thr, yaw)

        # Telemetry thread
        self._telem_running = False
        self._telem_thread: Optional[threading.Thread] = None
        self._telem = {
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "alt": 0.0, "hdg": 0.0,
        }
        self._flight_mode = "UNKNOWN"
        self._telem_lock  = threading.Lock()

        # Logger (None = tắt, gán bằng start_logging())
        self._logger: Optional[FlightLogger] = None
        self._last_acc = (0.0, 0.0, 0.0)  # ax, ay, az — để logger thread dùng

    # ------------------------------------------------------------------
    # Kết nối / ngắt kết nối
    # ------------------------------------------------------------------

    def connect(self, heartbeat_timeout: float = 15.0) -> None:
        """Kết nối MAVLink và STM32 serial, bắt đầu các background threads."""
        print(f"[HwAcc] Kết nối MAVLink: {self._drone_port}")
        self._mav = mavutil.mavlink_connection(
            self._drone_port,
            source_system=self._source_system
        )
        self._mav.wait_heartbeat(timeout=heartbeat_timeout)
        print(f"[HwAcc] Heartbeat: sysid={self._mav.target_system} "
              f"compid={self._mav.target_component}")

        self._rc.connect()
        self._start_telem_thread()
        self._start_sbus_thread()
        print("[HwAcc] Sẵn sàng\n")

    def disconnect(self) -> None:
        """Dừng tất cả threads và đóng kết nối."""
        print("[HwAcc] Ngắt kết nối...")
        self._stop_sbus_thread()
        self._stop_telem_thread()
        self._rc.close()
        if self._mav:
            self._mav.close()
            self._mav = None

    # ------------------------------------------------------------------
    # Chờ phi công trigger (POSCTL / ALTCTL)
    # ------------------------------------------------------------------

    def wait_for_autonomous_mode(self,
                                  accepted: Tuple[str, ...] = ("POSCTL", "ALTCTL"),
                                  timeout_s: float = 120.0) -> str:
        """
        Block cho đến khi phi công chuyển mode sang POSCTL hoặc ALTCTL.
        Trả về tên mode đã detect.

        Raises TimeoutError nếu quá timeout_s giây.
        """
        print("[HwAcc] Chờ phi công trigger mode tự động...")
        print(f"         Mode chấp nhận: {accepted}")
        print("         Phi công: arm → takeoff → flip trigger MK15\n")

        deadline  = time.time() + timeout_s
        last_mode = ""
        while time.time() < deadline:
            with self._telem_lock:
                mode = self._flight_mode
            if mode != last_mode:
                print(f"         [mode] {mode}")
                last_mode = mode
            if any(m in mode.upper() for m in accepted):
                print(f"\n[HwAcc] Autonomous mode: {mode}")
                return mode
            time.sleep(0.2)

        raise TimeoutError(
            f"Timeout sau {timeout_s}s chờ mode {accepted}. "
            f"Mode hiện tại: {self._flight_mode}"
        )

    # ------------------------------------------------------------------
    # Gửi acc command
    # ------------------------------------------------------------------

    def send_acceleration_ned(self,
                               ax: float, ay: float, az: float,
                               yaw_rad: Optional[float] = None) -> None:
        """
        Gửi NED acceleration setpoint lên Pixhawk.

        ax       : gia tốc North (m/s²)
        ay       : gia tốc East  (m/s²)
        az       : gia tốc Down  (m/s², dương = xuống)
        yaw_rad  : heading mong muốn (rad). None = giữ nguyên heading.

        Phải gọi ≥ 2 Hz để tránh watchdog 500ms fired.
        Khuyến nghị: 50 Hz.
        """
        if self._mav is None:
            raise RuntimeError("Chưa kết nối. Gọi connect() trước.")

        if yaw_rad is not None:
            type_mask = _TYPE_MASK_ACC_WITH_YAW
            yaw = float(yaw_rad)
        else:
            type_mask = _TYPE_MASK_ACC_ONLY
            yaw = 0.0

        self._mav.mav.set_position_target_local_ned_send(
            time_boot_ms=0,
            target_system=self._mav.target_system,
            target_component=self._mav.target_component,
            coordinate_frame=_MAV_FRAME_LOCAL_NED,
            type_mask=type_mask,
            x=0.0, y=0.0, z=0.0,
            vx=0.0, vy=0.0, vz=0.0,
            afx=float(ax), afy=float(ay), afz=float(az),
            yaw=yaw,
            yaw_rate=0.0
        )
        self._last_acc = (float(ax), float(ay), float(az))
        self._log_event()

    def hover(self) -> None:
        """Gửi zero acceleration — drone giảm tốc dần về stop."""
        self.send_acceleration_ned(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def start_logging(self, path: Optional[str] = None) -> None:
        """Bắt đầu ghi log CSV. path=None → tự tạo tên theo timestamp."""
        self._logger = FlightLogger(path)
        self._log_event("START")

    def stop_logging(self) -> None:
        """Dừng và lưu log."""
        if self._logger:
            self._log_event("STOP")
            self._logger.close()
            self._logger = None

    def log_event(self, label: str) -> None:
        """Ghi event marker vào log (ví dụ: 'LEG_NORTH', 'BRAKE')."""
        self._log_event(label)

    def _log_event(self, event: str = "") -> None:
        if self._logger is None:
            return
        ax, ay, az = self._last_acc
        with self._telem_lock:
            t = self._telem
            mode = self._flight_mode
        self._logger.write(
            mode, t["alt"], t["hdg"],
            t["vx"], t["vy"], t["vz"],
            ax, ay, az,
            event=event,
        )

    # ------------------------------------------------------------------
    # Telemetry (read-only properties)
    # ------------------------------------------------------------------

    @property
    def velocity_ned(self) -> Tuple[float, float, float]:
        """(vx_north, vy_east, vz_down) m/s."""
        with self._telem_lock:
            t = self._telem
            return t["vx"], t["vy"], t["vz"]

    @property
    def speed_horizontal(self) -> float:
        """Tốc độ ngang (m/s)."""
        vx, vy, _ = self.velocity_ned
        return math.sqrt(vx**2 + vy**2)

    @property
    def altitude_m(self) -> float:
        with self._telem_lock:
            return self._telem["alt"]

    @property
    def heading_deg(self) -> float:
        with self._telem_lock:
            return self._telem["hdg"]

    @property
    def flight_mode(self) -> str:
        with self._telem_lock:
            return self._flight_mode

    def print_telemetry(self) -> None:
        vx, vy, vz = self.velocity_ned
        print(f"  [telem] mode={self.flight_mode:10s} "
              f"alt={self.altitude_m:+6.1f}m "
              f"hdg={self.heading_deg:5.1f}° "
              f"vN={vx:+5.2f} vE={vy:+5.2f} vD={vz:+5.2f} "
              f"spd={self.speed_horizontal:.2f} m/s")

    # ------------------------------------------------------------------
    # Param helper
    # ------------------------------------------------------------------

    def set_param(self, name: str, value: float) -> None:
        if self._mav is None:
            raise RuntimeError("Chưa kết nối.")
        self._mav.mav.param_set_send(
            self._mav.target_system,
            self._mav.target_component,
            name.encode(),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        print(f"[HwAcc] Param: {name} = {value}")

    # ------------------------------------------------------------------
    # SBUS keep-alive thread
    # ------------------------------------------------------------------

    def _start_sbus_thread(self):
        self._sbus_running = True
        self._sbus_thread = threading.Thread(
            target=self._sbus_loop, daemon=True, name="sbus-keepalive"
        )
        self._sbus_thread.start()
        print(f"[HwAcc] SBUS keep-alive @ {SBUS_RATE_HZ} Hz")

    def _stop_sbus_thread(self):
        self._sbus_running = False
        if self._sbus_thread:
            self._sbus_thread.join(timeout=2.0)

    def _sbus_loop(self):
        """Gửi centered sticks liên tục để FlightTask luôn có valid RC input."""
        while self._sbus_running:
            with self._sbus_lock:
                pitch, roll, thr, yaw = self._sbus_cmd
            self._rc.send(pitch, roll, thr, yaw)
            time.sleep(SBUS_INTERVAL)

    # ------------------------------------------------------------------
    # Telemetry thread (pymavlink receive loop)
    # ------------------------------------------------------------------

    def _start_telem_thread(self):
        self._telem_running = True
        self._telem_thread = threading.Thread(
            target=self._telem_loop, daemon=True, name="telem"
        )
        self._telem_thread.start()

    def _stop_telem_thread(self):
        self._telem_running = False
        if self._telem_thread:
            self._telem_thread.join(timeout=2.0)

    def _telem_loop(self):
        _TYPES = ["LOCAL_POSITION_NED", "VFR_HUD", "HEARTBEAT"]
        while self._telem_running:
            if self._mav is None:
                time.sleep(0.1)
                continue
            msg = self._mav.recv_match(type=_TYPES, blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()

            if t == "LOCAL_POSITION_NED":
                with self._telem_lock:
                    self._telem["vx"] = msg.vx
                    self._telem["vy"] = msg.vy
                    self._telem["vz"] = msg.vz

            elif t == "VFR_HUD":
                with self._telem_lock:
                    self._telem["alt"] = msg.alt
                    self._telem["hdg"] = float(msg.heading)

            elif t == "HEARTBEAT":
                if msg.get_srcSystem() != self._mav.target_system:
                    continue
                main = (msg.custom_mode >> 16) & 0xFF
                sub  = (msg.custom_mode >> 24) & 0xFF
                mode = _MODE_MAP.get((main, sub), f"MODE({main},{sub})")
                with self._telem_lock:
                    self._flight_mode = mode


# ---------------------------------------------------------------------------
# CLI — bench test (trên bàn, không cần bay) + flight test
# ---------------------------------------------------------------------------

def _parse_args():
    p = argparse.ArgumentParser(
        description="hw_acc_control — bench test + flight test")
    p.add_argument("--drone",    "-d", default="serial:///dev/ttyACM0:57600",
                   help="MAVLink connection string")
    p.add_argument("--rc-port",        default="/dev/ttyUSB0",
                   help="STM32 serial port")
    p.add_argument("--rc-baud",  type=int, default=115200)
    p.add_argument("--acc",      type=float, default=1.0,
                   help="Gia tốc test (m/s²) — chỉ dùng khi bay")
    p.add_argument("--duration", type=float, default=3.0,
                   help="Thời gian gửi lệnh (s) — chỉ dùng khi bay")
    p.add_argument("--mode-timeout", type=float, default=120.0,
                   help="Timeout chờ POSCTL/ALTCTL (s)")
    p.add_argument("--bench", action="store_true",
                   help="Chạy bench test (không cần bay, không cần arm)")
    p.add_argument("--log", metavar="FILE", default=None,
                   help="Ghi log CSV vào file (mặc định: tự tạo tên theo timestamp)")
    p.add_argument("--no-log", action="store_true",
                   help="Tắt logging hoàn toàn")
    return p.parse_args()


def run_bench_test(ctrl: HardwareAccControl) -> bool:
    """
    Bench test — chạy trên bàn, không cần cánh, không cần arm.

    Các bước:
      B1. Verify MAVLink heartbeat + đọc flight mode
      B2. Verify SBUS keep-alive gửi không lỗi
      B3. Gửi 20 acc commands → kiểm tra [ext_acc] trong Pixhawk console
      B4. Dừng 1.5s → verify watchdog log (nếu drone đang arm)

    Để xem [ext_acc] debug prints:
      QGroundControl → MAVLink Console → gõ: listener acc_sp_external
      hoặc nối debug UART → minicom /dev/ttyUSB_DEBUG
    """
    passed = 0
    failed = 0

    def ok(msg):
        nonlocal passed
        passed += 1
        print(f"  [PASS] {msg}")

    def fail(msg):
        nonlocal failed
        failed += 1
        print(f"  [FAIL] {msg}")

    print("\n" + "=" * 55)
    print("BENCH TEST — không cần arm, không cần bay")
    print("=" * 55)

    # B1 — MAVLink heartbeat
    print("\nB1. MAVLink heartbeat + flight mode")
    time.sleep(1.0)  # chờ telem thread đọc heartbeat đầu tiên
    mode = ctrl.flight_mode
    if mode != "UNKNOWN":
        ok(f"Heartbeat OK, mode = {mode}")
    else:
        fail("Chưa nhận heartbeat hoặc mode = UNKNOWN")

    # B2 — SBUS keep-alive
    print("\nB2. SBUS keep-alive (STM32 serial)")
    if ctrl._rc.connected:
        ok(f"SerialRC connected: {ctrl._rc.port}")
        # Gửi 20 packets thử, kiểm tra không exception
        errors = 0
        for _ in range(20):
            try:
                ctrl._rc.send_centered()
            except Exception:
                errors += 1
            time.sleep(0.05)
        if errors == 0:
            ok("20 SBUS packets gửi OK, không lỗi")
        else:
            fail(f"{errors}/20 packets lỗi")
    else:
        fail("SerialRC không kết nối được — kiểm tra /dev/ttyUSB0")

    # B3 — Acc commands (gửi không cần arm)
    print("\nB3. Gửi acc commands → kiểm tra [ext_acc] trên Pixhawk console")
    print("     >> Mở QGC MAVLink Console, gõ: listener acc_sp_external")
    print("     >> Hoặc: nsh> listener acc_sp_external")
    acc_errors = 0
    for i in range(20):
        try:
            ctrl.send_acceleration_ned(1.0, 0.0, 0.0)
        except Exception as e:
            acc_errors += 1
            print(f"     Lỗi gửi acc: {e}")
        time.sleep(0.02)
    if acc_errors == 0:
        ok("20 acc commands gửi OK (kiểm tra [ext_acc] count trên Pixhawk)")
    else:
        fail(f"{acc_errors}/20 acc commands lỗi")

    # B4 — Watchdog (thông báo thôi, không tự verify vì cần drone arm)
    print("\nB4. Watchdog timeout")
    print("     >> Dừng gửi 1.5s — nếu drone đang arm:")
    print("        sẽ thấy [ext_acc] WARN trên Pixhawk console")
    time.sleep(1.5)
    ok("Watchdog test: xem log Pixhawk để confirm (cần arm để verify đầy đủ)")

    # Kết quả
    print("\n" + "=" * 55)
    print(f"BENCH TEST KẾT QUẢ: {passed} PASS / {failed} FAIL")
    print("=" * 55)
    if failed == 0:
        print(">> Sẵn sàng test bay!\n")
    else:
        print(">> Cần fix các lỗi trên trước khi bay.\n")
    return failed == 0


def main():
    args = _parse_args()

    print("=" * 55)
    print("hw_acc_control.py")
    print(f"  drone  : {args.drone}")
    print(f"  rc     : {args.rc_port} @ {args.rc_baud}")
    if not args.bench:
        print(f"  acc    : {args.acc} m/s²  duration: {args.duration}s")
    print("=" * 55 + "\n")

    ctrl = HardwareAccControl(
        drone_port=args.drone,
        rc_port=args.rc_port,
        rc_baudrate=args.rc_baud,
    )

    try:
        ctrl.connect()

        if not args.no_log:
            ctrl.start_logging(args.log)

        # ── Bench mode: test trên bàn, không cần bay ──────────────────
        if args.bench:
            run_bench_test(ctrl)
            return

        # ── Flight mode: chờ phi công trigger ─────────────────────────
        ctrl.wait_for_autonomous_mode(timeout_s=args.mode_timeout)

        # Hover 3s để ổn định
        print("\n[Test] Hover 3s...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            ctrl.hover()
            ctrl.print_telemetry()
            time.sleep(0.1)

        # Fly North
        print(f"\n[Test] Bay North ax={args.acc} m/s² trong {args.duration}s")
        t0 = time.time()
        while time.time() - t0 < args.duration:
            ctrl.send_acceleration_ned(args.acc, 0.0, 0.0)
            time.sleep(0.02)  # 50 Hz

        # Brake
        print("\n[Test] Brake 3s (zero acc)...")
        t0 = time.time()
        while time.time() - t0 < 3.0:
            ctrl.hover()
            ctrl.print_telemetry()
            time.sleep(0.1)

        print(f"\n[Test] Xong. vx={ctrl.velocity_ned[0]:+.2f} m/s")
        print("         Phi công: flip trigger MK15 để lấy lại điều khiển.")

    except KeyboardInterrupt:
        print("\n[Test] Dừng bởi người dùng")
    except TimeoutError as e:
        print(f"\n[Test] TIMEOUT: {e}")
    finally:
        ctrl.stop_logging()
        ctrl.disconnect()


if __name__ == "__main__":
    main()
