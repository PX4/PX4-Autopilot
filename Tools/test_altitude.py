#!/usr/bin/env python3
"""
test_altitude.py — Kiểm chứng px4_inverse_rc ở ALTITUDE mode
=============================================================
Kịch bản:
  1. Dùng QGC arm + bay lên + chuyển ALTITUDE mode bằng tay / joystick ảo
  2. Script này chờ phát hiện drone vào ALTITUDE mode
  3. Tự động chạy toàn bộ test cases và in kết quả

Yêu cầu: pip install mavsdk numpy
px4_inverse_rc.py phải cùng thư mục.
"""

import asyncio
import sys
import os
import math
import argparse
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_inverse_rc import altitude_mode_to_mavlink

from mavsdk import System
from mavsdk.manual_control import ManualControlError
# FlightMode enum không dùng trực tiếp — so sánh qua str() để tránh AttributeError


# ─── Drone params ─────────────────────────────────────────────────────────────

@dataclass
class DroneParams:
    """Param PX4 ảnh hưởng đến px4_inverse_rc.
    Giá trị default khớp với PX4 firmware default.
    Được đọc từ drone qua MAVSDK ngay sau khi kết nối.
    """
    vel_max_up: float = 3.0    # MPC_Z_VEL_MAX_UP  (m/s)
    vel_max_dn: float = 1.5    # MPC_Z_VEL_MAX_DN  (m/s)
    yaw_max:    float = 150.0  # MPC_MAN_Y_MAX     (°/s)
    tilt_max:   float = 35.0   # MPC_MAN_TILT_MAX  (°)
    deadzone:   float = 0.0    # MAN_DEADZONE
    source:     str   = "default"


async def read_params_via_mavsdk(drone: System) -> DroneParams:
    """Đọc param PX4 qua MAVSDK — cùng connection, không xung đột port.
    Param nào lỗi → dùng default và cảnh báo.
    """
    PARAM_MAP = {
        # px4_name: (field_name, default, unit)
        'MPC_Z_VEL_MAX_UP': ('vel_max_up',  3.0, 'm/s'),
        'MPC_Z_VEL_MAX_DN': ('vel_max_dn',  1.5, 'm/s'),
        'MPC_MAN_Y_MAX':    ('yaw_max',   150.0, '°/s'),
        'MPC_MAN_TILT_MAX': ('tilt_max',   35.0, '°'),
        'MAN_DEADZONE':     ('deadzone',    0.0,  ''),
    }

    print("\nĐọc param từ drone...")
    print(f"  {'Param':<20} {'Giá trị':>8}  {'Unit':<5} {'Ghi chú'}")
    print(f"  {'─'*52}")

    values = {}
    warnings = []

    for px4_name, (field, default, unit) in PARAM_MAP.items():
        try:
            val = await drone.param.get_param_float(px4_name)
            diff = abs(val - default) / max(abs(default), 0.001) * 100
            flag = f" ← KHÁC DEFAULT ({default})" if diff > 1.0 else ""
            print(f"  {px4_name:<20} {val:>8.3f}  {unit:<5}{flag}")
            values[field] = val
        except Exception as e:
            print(f"  {'⚠ '+px4_name:<20} {'N/A':>8}         lỗi: {e} → dùng default={default}")
            values[field] = default
            warnings.append(px4_name)

    if warnings:
        print(f"  ⚠ Không đọc được: {', '.join(warnings)} — dùng default")

    p = DroneParams(**values, source="drone")
    print(f"  {'─'*52}")
    print(f"  ✓ Xong\n")
    return p

# ─── Cấu hình ────────────────────────────────────────────────────────────────
SITL_ADDRESS    = "udp://:14540"
SEND_RATE_HZ    = 20
INTERVAL        = 1.0 / SEND_RATE_HZ

# Settle/measure time — ALTITUDE mode cần lâu hơn vì không giữ vị trí
SETTLE_TIME     = 5.0   # s — chờ drone đạt trạng thái ổn định
MEASURE_TIME    = 3.0   # s — đo thực tế

# Tolerance
PITCH_TOLERANCE = 3.0   # ° — sai số góc pitch/roll chấp nhận được
VEL_TOLERANCE   = 0.8   # m/s — sai số vz
YAW_TOLERANCE   = 20.0  # °/s

# Giới hạn an toàn khi test — không nghiêng quá mạnh
MAX_TEST_TILT   = 10.0  # ° — góc nghiêng tối đa trong test
MIN_ALT_M       = 5.0   # m — ngưỡng độ cao tối thiểu để bắt đầu test


# ─── Data classes ────────────────────────────────────────────────────────────

@dataclass
class Sample:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    heading_deg: float = 0.0
    pitch_deg: float = 0.0   # attitude pitch thực tế
    roll_deg: float = 0.0    # attitude roll thực tế
    alt_m: float = 0.0


@dataclass
class TestResult:
    name: str
    passed: bool
    expected: str
    actual: str
    error_val: str


# ─── Telemetry Collector ─────────────────────────────────────────────────────

class TelemetryCollector:
    def __init__(self):
        self.latest = Sample()
        self.samples: list[Sample] = []
        self.collecting = False
        self.current_mode = "UNKNOWN"
        self._tasks = []

    async def start(self, drone: System):
        self._tasks += [
            asyncio.ensure_future(self._collect_vel(drone)),
            asyncio.ensure_future(self._collect_att(drone)),
            asyncio.ensure_future(self._collect_angular(drone)),
            asyncio.ensure_future(self._collect_pos(drone)),
            asyncio.ensure_future(self._watch_mode(drone)),
        ]

    async def _collect_vel(self, drone):
        async for v in drone.telemetry.velocity_ned():
            self.latest.vx = v.north_m_s
            self.latest.vy = v.east_m_s
            self.latest.vz = v.down_m_s
            if self.collecting:
                self.samples.append(Sample(
                    vx=self.latest.vx, vy=self.latest.vy, vz=self.latest.vz,
                    yaw_rate=self.latest.yaw_rate,
                    heading_deg=self.latest.heading_deg,
                    pitch_deg=self.latest.pitch_deg,
                    roll_deg=self.latest.roll_deg,
                    alt_m=self.latest.alt_m,
                ))

    async def _collect_att(self, drone):
        async for att in drone.telemetry.attitude_euler():
            self.latest.heading_deg = att.yaw_deg
            self.latest.pitch_deg   = att.pitch_deg
            self.latest.roll_deg    = att.roll_deg

    async def _collect_angular(self, drone):
        async for ang in drone.telemetry.attitude_angular_velocity_body():
            self.latest.yaw_rate = math.degrees(ang.yaw_rad_s)

    async def _collect_pos(self, drone):
        async for pos in drone.telemetry.position():
            self.latest.alt_m = pos.relative_altitude_m

    async def _watch_mode(self, drone):
        async for mode in drone.telemetry.flight_mode():
            mode_str = str(mode)
            if mode_str != self.current_mode:
                print(f"  [Mode] {self.current_mode} → {mode_str}")
                self.current_mode = mode_str

    def begin_collect(self):
        self.samples.clear()
        self.collecting = True

    def end_collect(self) -> list[Sample]:
        self.collecting = False
        return list(self.samples)

    def stop(self):
        for t in self._tasks:
            t.cancel()


def avg_samples(samples: list[Sample]) -> Sample:
    if not samples:
        return Sample()
    n = len(samples)
    sin_h = sum(math.sin(math.radians(s.heading_deg)) for s in samples)
    cos_h = sum(math.cos(math.radians(s.heading_deg)) for s in samples)
    return Sample(
        vx=sum(s.vx for s in samples) / n,
        vy=sum(s.vy for s in samples) / n,
        vz=sum(s.vz for s in samples) / n,
        yaw_rate=sum(s.yaw_rate for s in samples) / n,
        heading_deg=math.degrees(math.atan2(sin_h, cos_h)),
        pitch_deg=sum(s.pitch_deg for s in samples) / n,
        roll_deg=sum(s.roll_deg for s in samples) / n,
        alt_m=sum(s.alt_m for s in samples) / n,
    )


# ─── Tiện ích điều khiển ─────────────────────────────────────────────────────

def ned_to_body(vx_ned, vy_ned, heading_deg):
    yaw = math.radians(heading_deg)
    bx =  vx_ned * math.cos(yaw) + vy_ned * math.sin(yaw)
    by = -vx_ned * math.sin(yaw) + vy_ned * math.cos(yaw)
    return bx, by


def to_mavsdk_alt(pitch_deg, roll_deg, vz_ned=0.0, yaw_dps=0.0,
                  params: DroneParams = None):
    """Altitude mode: góc nghiêng (°) + vz (m/s) → MAVSDK inputs.
    Truyền params để dùng giá trị thực từ drone thay vì default.
    """
    p = params or DroneParams()
    x, y, z, r = altitude_mode_to_mavlink(
        pitch_deg=pitch_deg, roll_deg=roll_deg,
        vz=vz_ned, yaw_rate_dps=yaw_dps,
        tilt_max=p.tilt_max,
        vel_max_up=p.vel_max_up, vel_max_dn=p.vel_max_dn,
        yaw_max=p.yaw_max, deadzone=p.deadzone,
    )
    # Từ thực nghiệm ALTITUDE mode (SITL + QGC):
    #   pitch_deg=+10 → đo pitch=-9.9° → pitch bị đảo → dùng -x
    #   roll_deg=+10  → đo roll=+9.9°  → roll đúng    → dùng +y
    return -x/1000.0, +y/1000.0, z/1000.0, r/1000.0


async def send_input(drone, pitch, roll, throttle, yaw):
    await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)


async def hover_alt(drone, duration=2.0):
    """Hover trong altitude mode: stick giữa."""
    steps = int(duration * SEND_RATE_HZ)
    for _ in range(steps):
        await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)
        await asyncio.sleep(INTERVAL)


async def send_and_measure(drone, tel, pitch, roll, throttle, yaw) -> list[Sample]:
    steps_settle  = int(SETTLE_TIME  * SEND_RATE_HZ)
    steps_measure = int(MEASURE_TIME * SEND_RATE_HZ)
    for _ in range(steps_settle):
        await send_input(drone, pitch, roll, throttle, yaw)
        await asyncio.sleep(INTERVAL)
    tel.begin_collect()
    for _ in range(steps_measure):
        await send_input(drone, pitch, roll, throttle, yaw)
        await asyncio.sleep(INTERVAL)
    return tel.end_collect()


# ─── Test cases ALTITUDE mode ────────────────────────────────────────────────
# Altitude mode: pitch/roll → góc nghiêng → gia tốc ngang (không giữ vị trí)
# Đo: góc attitude thực tế + hướng di chuyển (body frame)
# vz và yaw vẫn đo trực tiếp như POSCTL

async def tc_hover(drone, tel, params: DroneParams) -> TestResult:
    """TC1: Hover — stick giữa, tất cả về 0."""
    print("\n[TC1] Hover — stick giữa")
    samples = await send_and_measure(drone, tel, 0.0, 0.0, 0.5, 0.0)
    avg = avg_samples(samples)
    mag = math.sqrt(avg.vx**2 + avg.vy**2 + avg.vz**2)
    tilt = math.sqrt(avg.pitch_deg**2 + avg.roll_deg**2)
    passed = mag < 0.5 and tilt < 2.0
    print(f"  pitch={avg.pitch_deg:+.1f}° roll={avg.roll_deg:+.1f}° |v|={mag:.2f}m/s")
    return TestResult("TC1 Hover",
        passed=passed,
        expected="|v|<0.5 m/s, tilt<2°",
        actual=f"|v|={mag:.3f} pitch={avg.pitch_deg:+.1f}° roll={avg.roll_deg:+.1f}°",
        error_val=f"|v|={mag:.3f} tilt={tilt:.1f}°")


async def tc_pitch_forward(drone, tel, params: DroneParams) -> TestResult:
    """TC2: Pitch tới trước — kiểm tra góc pitch thực tế."""
    target_pitch = MAX_TEST_TILT  # 10°
    print(f"\n[TC2] Pitch tiến {target_pitch}°")
    # Extra hover 2s — TC2 là test đầu tiên có input khác 0,
    # cần thêm thời gian để PX4 chấp nhận RC source sau hover
    await hover_alt(drone, 2.0)
    p, r, t, y = to_mavsdk_alt(pitch_deg=target_pitch, roll_deg=0.0, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    # Pre-warm 1s với nửa giá trị để PX4 thoát trạng thái hover lock
    for _ in range(int(1.0 * SEND_RATE_HZ)):
        await drone.manual_control.set_manual_control_input(p * 0.5, 0.0, 0.5, 0.0)
        await asyncio.sleep(INTERVAL)
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    bx, _ = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err_pitch = abs(avg.pitch_deg - target_pitch)
    # ALTITUDE mode: pitch+ = mũi lên = bay lùi — chỉ check góc
    passed = err_pitch < PITCH_TOLERANCE and avg.pitch_deg > 0
    print(f"  [hdg={avg.heading_deg:.0f}°] pitch_actual={avg.pitch_deg:+.1f}° body_fwd={bx:+.2f}m/s")
    return TestResult(f"TC2 Pitch +{target_pitch}°",
        passed=passed,
        expected=f"pitch ≈ +{target_pitch}° (ALTCTL: pitch+→mũi lên→lùi)",
        actual=f"pitch={avg.pitch_deg:+.1f}° body_fwd={bx:+.2f}m/s [hdg={avg.heading_deg:.0f}°]",
        error_val=f"Δpitch={err_pitch:.1f}°")


async def tc_pitch_backward(drone, tel, params: DroneParams) -> TestResult:
    """TC3: Pitch ra sau."""
    target_pitch = -MAX_TEST_TILT
    print(f"\n[TC3] Pitch lùi {target_pitch}°")
    p, r, t, y = to_mavsdk_alt(pitch_deg=target_pitch, roll_deg=0.0, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    bx, _ = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err_pitch = abs(avg.pitch_deg - target_pitch)
    # ALTITUDE mode: pitch- = mũi xuống = bay tiến (body_fwd > 0) — chỉ check góc
    passed = err_pitch < PITCH_TOLERANCE and avg.pitch_deg < 0
    print(f"  [hdg={avg.heading_deg:.0f}°] pitch_actual={avg.pitch_deg:+.1f}° body_fwd={bx:+.2f}m/s")
    return TestResult(f"TC3 Pitch {target_pitch}°",
        passed=passed,
        expected=f"pitch ≈ {target_pitch}° (ALTCTL: pitch-→mũi xuống→tiến)",
        actual=f"pitch={avg.pitch_deg:+.1f}° body_fwd={bx:+.2f}m/s [hdg={avg.heading_deg:.0f}°]",
        error_val=f"Δpitch={err_pitch:.1f}°")


async def tc_roll_right(drone, tel, params: DroneParams) -> TestResult:
    """TC4: Roll sang phải."""
    target_roll = MAX_TEST_TILT
    print(f"\n[TC4] Roll phải {target_roll}°")
    p, r, t, y = to_mavsdk_alt(pitch_deg=0.0, roll_deg=target_roll, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    _, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err_roll = abs(avg.roll_deg - target_roll)
    passed = err_roll < PITCH_TOLERANCE and by > 0.3
    print(f"  [hdg={avg.heading_deg:.0f}°] roll_actual={avg.roll_deg:+.1f}° body_right={by:+.2f}m/s")
    return TestResult(f"TC4 Roll +{target_roll}°",
        passed=passed,
        expected=f"roll ≈ +{target_roll}° và body_right > 0",
        actual=f"roll={avg.roll_deg:+.1f}° body_right={by:+.2f}m/s [hdg={avg.heading_deg:.0f}°]",
        error_val=f"Δroll={err_roll:.1f}°")


async def tc_roll_left(drone, tel, params: DroneParams) -> TestResult:
    """TC5: Roll sang trái."""
    target_roll = -MAX_TEST_TILT
    print(f"\n[TC5] Roll trái {target_roll}°")
    p, r, t, y = to_mavsdk_alt(pitch_deg=0.0, roll_deg=target_roll, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    _, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err_roll = abs(avg.roll_deg - target_roll)
    passed = err_roll < PITCH_TOLERANCE and by < -0.3
    print(f"  [hdg={avg.heading_deg:.0f}°] roll_actual={avg.roll_deg:+.1f}° body_right={by:+.2f}m/s")
    return TestResult(f"TC5 Roll {target_roll}°",
        passed=passed,
        expected=f"roll ≈ {target_roll}° và body_right < 0",
        actual=f"roll={avg.roll_deg:+.1f}° body_right={by:+.2f}m/s [hdg={avg.heading_deg:.0f}°]",
        error_val=f"Δroll={err_roll:.1f}°")


async def tc_climb(drone, tel, params: DroneParams) -> TestResult:
    """TC6: Lên cao vz=-1.0 m/s."""
    vz_t = -1.0
    print(f"\n[TC6] Lên cao vz={vz_t} m/s")
    p, r, t, y = to_mavsdk_alt(pitch_deg=0.0, roll_deg=0.0, vz_ned=vz_t, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.vz - vz_t)
    passed = err < VEL_TOLERANCE
    print(f"  vz_actual={avg.vz:+.2f}m/s alt={avg.alt_m:.1f}m")
    return TestResult("TC6 Lên vz=-1",
        passed=passed,
        expected=f"vz ≈ {vz_t} m/s",
        actual=f"vz={avg.vz:+.2f} m/s (alt={avg.alt_m:.1f}m)",
        error_val=f"{err:.2f} m/s")


async def tc_descend(drone, tel, params: DroneParams) -> TestResult:
    """TC7: Xuống vz=+0.5 m/s (NED dương = xuống)."""
    vz_t = 0.5
    print(f"\n[TC7] Xuống vz=+{vz_t} m/s")
    p, r, t, y = to_mavsdk_alt(pitch_deg=0.0, roll_deg=0.0, vz_ned=vz_t, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.vz - vz_t)
    # Phải đang đi xuống (vz > 0 trong NED) mới pass
    passed = err < VEL_TOLERANCE and avg.vz > 0.0
    print(f"  vz_actual={avg.vz:+.2f}m/s alt={avg.alt_m:.1f}m")
    return TestResult("TC7 Xuống vz=+0.5",
        passed=passed,
        expected=f"vz ≈ +{vz_t} m/s (đi xuống)",
        actual=f"vz={avg.vz:+.2f} m/s (alt={avg.alt_m:.1f}m)",
        error_val=f"{err:.2f} m/s")


async def tc_yaw_right(drone, tel, params: DroneParams) -> TestResult:
    """TC8: Xoay yaw phải 30°/s."""
    yr_t = 30.0
    print(f"\n[TC8] Yaw phải {yr_t}°/s")
    p, r, t, y = to_mavsdk_alt(pitch_deg=0.0, roll_deg=0.0, yaw_dps=yr_t, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.yaw_rate - yr_t)
    passed = err < YAW_TOLERANCE
    print(f"  yaw_rate_actual={avg.yaw_rate:+.1f}°/s")
    return TestResult(f"TC8 Yaw +{yr_t}°/s",
        passed=passed,
        expected=f"yaw_rate ≈ +{yr_t} °/s",
        actual=f"yaw_rate={avg.yaw_rate:+.1f} °/s",
        error_val=f"{err:.1f} °/s")


async def tc_combined(drone, tel, params: DroneParams) -> TestResult:
    """TC9: Kết hợp pitch+roll+vz+yaw cùng lúc."""
    pd, rd, vz_t, yr_t = 8.0, 5.0, -0.5, 15.0
    print(f"\n[TC9] Kết hợp pitch={pd}° roll={rd}° vz={vz_t} yaw={yr_t}°/s")
    p, r, t, y = to_mavsdk_alt(pitch_deg=pd, roll_deg=rd, vz_ned=vz_t, yaw_dps=yr_t, params=params)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    bx, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    # pitch_deg sẽ đúng dấu sau fix -x/1000
    ep = abs(avg.pitch_deg - pd)
    er = abs(avg.roll_deg  - rd)
    ez = abs(avg.vz - vz_t)
    # Thêm check chiều: pitch dương = tiến, roll dương = phải
    pitch_dir_ok = avg.pitch_deg > 0
    roll_dir_ok  = avg.roll_deg  > 0
    passed = ep < PITCH_TOLERANCE and er < PITCH_TOLERANCE and ez < VEL_TOLERANCE              and pitch_dir_ok and roll_dir_ok
    print(f"  [hdg={avg.heading_deg:.0f}°] pitch={avg.pitch_deg:+.1f}° roll={avg.roll_deg:+.1f}° "
          f"vz={avg.vz:+.2f} body(fwd={bx:+.2f} right={by:+.2f})")
    return TestResult("TC9 Kết hợp",
        passed=passed,
        expected=f"pitch≈{pd}° roll≈{rd}° vz≈{vz_t}",
        actual=f"pitch={avg.pitch_deg:+.1f}° roll={avg.roll_deg:+.1f}° vz={avg.vz:+.2f} "
               f"body(fwd={bx:+.2f} right={by:+.2f})",
        error_val=f"Δp={ep:.1f}° Δr={er:.1f}° Δvz={ez:.2f}")


def print_results(results: list[TestResult]):
    print("\n" + "=" * 72)
    print("KẾT QUẢ KIỂM CHỨNG — ALTITUDE MODE + px4_inverse_rc")
    print("=" * 72)
    n_pass = sum(1 for r in results if r.passed)
    for r in results:
        icon = "✅" if r.passed else "❌"
        print(f"\n{icon} {r.name}")
        print(f"   Gửi đi  : {r.expected}")
        print(f"   Đo được : {r.actual}")
        print(f"   Sai số  : {r.error_val}")
    print("\n" + "-" * 72)
    print(f"TỔNG KẾT: {n_pass}/{len(results)} PASS "
          f"{'✅ TẤT CẢ OK' if n_pass == len(results) else '❌ CÓ TEST FAIL'}")
    print("=" * 72)


# ─── Main ────────────────────────────────────────────────────────────────────

async def wait_for_altitude_mode(tel: TelemetryCollector, timeout=0):
    """Chờ drone vào ALTITUDE mode (hoặc đủ cao với mode bất kỳ).
    
    timeout=0 → chờ vô hạn.
    Chấp nhận:
      - ALTCTL / ALTITUDE* mode ở bất kỳ độ cao nào >= MIN_ALT_M
      - Bất kỳ mode nào nếu alt >= MIN_ALT_M (fallback an toàn)
    """
    print("\n" + "=" * 72)
    print("⏳ ĐANG CHỜ — Dùng QGC để arm + bay lên, rồi chuyển ALTITUDE mode")
    print("   (Hoặc chỉ cần bay lên > 5m — script sẽ tự bắt đầu)")
    print("   Ctrl+C để thoát bất cứ lúc nào")
    print("=" * 72)
    t0 = asyncio.get_event_loop().time()
    last_print = 0.0

    while True:
        elapsed = asyncio.get_event_loop().time() - t0

        # Timeout nếu được đặt
        if timeout > 0 and elapsed > timeout:
            print(f"\n✗ Timeout {timeout}s")
            return False

        # In trạng thái mỗi 5s
        if elapsed - last_print >= 5.0:
            alt = tel.latest.alt_m
            mode = tel.current_mode
            armed_str = ""
            print(f"  [{elapsed:.0f}s] Mode={mode}  Alt={alt:.1f}m")
            last_print = elapsed

        mode_str = tel.current_mode.upper()
        alt = tel.latest.alt_m
        is_alt_mode  = any(k in mode_str for k in ("ALT",))
        has_height   = alt >= MIN_ALT_M
        is_any_mode  = mode_str not in ("UNKNOWN", "DISARMED", "LAND", "RTL", "")

        if is_alt_mode and has_height:
            print(f"\n✓ ALTITUDE mode ({tel.current_mode}) ở {alt:.1f}m → bắt đầu test!")
            return True

        # Fallback: nếu đủ cao dù không phải ALTCTL → hỏi người dùng
        if has_height and is_any_mode and not is_alt_mode:
            print(f"\n  ℹ Mode hiện tại: {tel.current_mode} (Alt={alt:.1f}m)")
            print(f"  Chưa vào ALTITUDE mode. Bấm Enter để test ngay với mode này,")
            print(f"  hoặc chuyển sang ALTITUDE mode trong QGC rồi bấm Enter:")
            try:
                # Non-blocking input với timeout ngắn
                loop = asyncio.get_event_loop()
                await asyncio.wait_for(
                    loop.run_in_executor(None, input, "  > "),
                    timeout=15.0
                )
                print(f"  → Bắt đầu test với mode {tel.current_mode}")
                return True
            except asyncio.TimeoutError:
                print(f"  (Timeout 15s — tiếp tục chờ ALTITUDE mode...)")
                last_print = asyncio.get_event_loop().time() - t0  # reset print timer

        await asyncio.sleep(0.2)


async def run():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connection', '-c', default=SITL_ADDRESS,
                        help=f'MAVLink connection (default: {SITL_ADDRESS})')
    parser.add_argument('--wait-timeout', type=float, default=0,
                        help='Thời gian tối đa chờ (s), 0=vô hạn (default)')
    parser.add_argument('--use-defaults', action='store_true',
                        help='Bỏ qua đọc param, dùng PX4 default')
    args = parser.parse_args()

    # Kết nối
    drone = System()
    print(f"Kết nối {args.connection}...")
    await drone.connect(system_address=args.connection)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Kết nối thành công!")
            break

    # Khởi động telemetry
    tel = TelemetryCollector()
    await tel.start(drone)
    await asyncio.sleep(1.0)  # chờ telemetry stream ổn định

    # Đọc param từ drone — cùng connection, không xung đột port
    if getattr(args, 'use_defaults', False):
        params = DroneParams()
        print("ℹ Dùng param default (--use-defaults)")
    else:
        params = await read_params_via_mavsdk(drone)

    # Chờ ALTITUDE mode từ QGC
    if not await wait_for_altitude_mode(tel, timeout=args.wait_timeout):  # 0 = vô hạn
        return

    # ── Bước 1: Warm-up — gửi liên tục để PX4 chấp nhận script là RC source ──
    print("\n[WARMUP] Gửi MANUAL_CONTROL liên tục 3s để PX4 nhận diện RC source...")
    print("  (Hãy nhả joystick QGC ra nếu đang giữ)")
    await hover_alt(drone, 3.0)

    # ── Bước 2: Kích hoạt Altitude Control API ──
    print("\n[INIT] Kích hoạt Altitude Control...")
    activated = False
    for attempt in range(3):
        try:
            await drone.manual_control.start_altitude_control()
            print("✓ start_altitude_control() OK")
            activated = True
            break
        except ManualControlError as e:
            print(f"  Lần {attempt+1}: {e}")
            # Tiếp tục gửi hover để build up RC input history
            await hover_alt(drone, 2.0)

    if not activated:
        print("⚠ Không kích hoạt được API — vẫn tiếp tục bằng set_manual_control_input")
        print("  PX4 vẫn nhận MANUAL_CONTROL trực tiếp nếu đã có RC source")

    # ── Bước 3: Kiểm tra drone có thực sự phản hồi không ──
    print("\n[CHECK] Kiểm tra drone phản hồi RC input...")
    print("  Gửi throttle cao (thr=0.7) trong 1s — quan sát drone có lên không")
    steps = int(1.0 * SEND_RATE_HZ)
    for _ in range(steps):
        await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.7, 0.0)
        await asyncio.sleep(INTERVAL)
    vz_check = tel.latest.vz
    alt_before = tel.latest.alt_m
    await asyncio.sleep(0.3)
    alt_after = tel.latest.alt_m
    alt_change = alt_after - alt_before
    if alt_change > 0.1 or vz_check < -0.1:
        print(f"✓ Drone phản hồi: alt {alt_before:.1f}→{alt_after:.1f}m (Δ={alt_change:+.2f}m)")
    else:
        print(f"⚠ Drone KHÔNG phản hồi: alt={alt_before:.1f}→{alt_after:.1f}m vz={vz_check:+.2f}")
        print("  → Thử gửi tiếp warm-up 5s...")
        await hover_alt(drone, 5.0)
        # Thử lại start_altitude_control
        try:
            await drone.manual_control.start_altitude_control()
            print("  ✓ start_altitude_control() OK lần 2")
        except ManualControlError as e:
            print(f"  ⚠ Vẫn lỗi: {e} — tiếp tục...")

    # ── Bước 4: Hover ổn định trước khi test ──
    print("\nHover ổn định 3s...")
    await hover_alt(drone, 3.0)

    # Chạy test cases
    print(f"\n{'='*72}")
    print(f"BẮT ĐẦU KIỂM CHỨNG ALTITUDE MODE")
    print(f"  Settle={SETTLE_TIME}s | Measure={MEASURE_TIME}s")
    print(f"  Max tilt={MAX_TEST_TILT}° | Alt hiện tại={tel.latest.alt_m:.1f}m")
    print(f"{'='*72}")

    ALL_TESTS = [
        tc_hover,
        tc_pitch_forward,
        tc_pitch_backward,
        tc_roll_right,
        tc_roll_left,
        tc_climb,
        tc_descend,
        tc_yaw_right,
        tc_combined,
    ]

    results: list[TestResult] = []
    for tf in ALL_TESTS:
        # Safety check — dừng nếu độ cao quá thấp
        if tel.latest.alt_m < 3.0:
            print(f"\n⚠ Độ cao {tel.latest.alt_m:.1f}m quá thấp — dừng test!")
            break
        try:
            result = await tf(drone, tel, params)
            results.append(result)
            icon = "✅" if result.passed else "❌"
            print(f"  {icon} {result.actual}  (sai số: {result.error_val})")
        except Exception as ex:
            print(f"  ❌ Exception: {ex}")
            results.append(TestResult(
                name=tf.__name__, passed=False,
                expected="—", actual=f"Exception: {ex}", error_val=str(ex)))
        # Hover ngắn giữa các test để ổn định
        await hover_alt(drone, 2.0)

    print_results(results)

    # Trả quyền điều khiển về QGC
    print("\n✓ Test hoàn tất — Bạn có thể dùng QGC để landing.")
    print("  (Script không tự land để tránh xung đột với joystick QGC)")

    tel.stop()


if __name__ == "__main__":
    asyncio.run(run())
