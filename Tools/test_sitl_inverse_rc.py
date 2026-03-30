#!/usr/bin/env python3
"""
test.py — MAVSDK + px4_inverse_rc + Verification
Kết nối PX4 SITL, arm, takeoff, điều khiển bằng giá trị vật lý,
và XÁC NHẬN bằng cách đo vận tốc/attitude thực tế.

Yêu cầu: pip install mavsdk numpy
px4_inverse_rc.py phải cùng thư mục.
"""

import asyncio
import sys
import os
import math
import time
from dataclasses import dataclass, field
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_inverse_rc import position_mode_to_mavlink, altitude_mode_to_mavlink

from mavsdk import System
from mavsdk.manual_control import ManualControlError
from mavsdk.telemetry import FlightMode

# ─── Cấu hình ────────────────────────────────────────────────────────────────
SITL_ADDRESS   = "udp://:14540"
SEND_RATE_HZ   = 20
INTERVAL       = 1.0 / SEND_RATE_HZ
SETTLE_TIME    = 4.0   # Giây chờ drone ổn định trước khi đo (tăng lên)
MEASURE_TIME   = 3.0   # Giây đo thực tế
VEL_TOLERANCE  = 1.2   # m/s — sai số chấp nhận
YAW_TOLERANCE  = 20.0  # °/s — sai số yaw chấp nhận (SITL có lag)


# ─── Data class lưu kết quả ──────────────────────────────────────────────────

@dataclass
class Sample:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0   # °/s
    heading_deg: float = 0.0  # yaw heading hiện tại (°)


@dataclass
class TestResult:
    name: str
    passed: bool
    expected: str
    actual: str
    error_val: str


# ─── Telemetry collector (chạy song song) ────────────────────────────────────

class TelemetryCollector:
    """Thu thập telemetry liên tục trong background."""
    def __init__(self):
        self.latest: Sample = Sample()
        self.samples: list[Sample] = []
        self.collecting = False
        self._tasks = []

    async def start(self, drone: System):
        self._tasks.append(asyncio.ensure_future(self._collect_vel(drone)))
        self._tasks.append(asyncio.ensure_future(self._collect_att(drone)))
        self._tasks.append(asyncio.ensure_future(self._print_mode(drone)))

    async def _collect_vel(self, drone):
        async for vel in drone.telemetry.velocity_ned():
            self.latest.vx = vel.north_m_s
            self.latest.vy = vel.east_m_s
            self.latest.vz = vel.down_m_s
            if self.collecting:
                s = Sample(vx=self.latest.vx, vy=self.latest.vy,
                           vz=self.latest.vz, yaw_rate=self.latest.yaw_rate,
                           heading_deg=self.latest.heading_deg)
                self.samples.append(s)

    async def _collect_att(self, drone):
        async for att in drone.telemetry.attitude_euler():
            self.latest.heading_deg = att.yaw_deg  # heading hiện tại

    async def _print_mode(self, drone):
        async for mode in drone.telemetry.flight_mode():
            print(f"  [Mode] {mode}")

    async def start_angular(self, drone):
        """Collect angular velocity riêng (cần gọi sau start)."""
        self._tasks.append(asyncio.ensure_future(self._collect_angular(drone)))

    async def _collect_angular(self, drone):
        async for ang in drone.telemetry.attitude_angular_velocity_body():
            self.latest.yaw_rate = math.degrees(ang.yaw_rad_s)

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
    # Heading: tính mean circular để tránh wrap-around 180°/-180°
    sin_sum = sum(math.sin(math.radians(s.heading_deg)) for s in samples)
    cos_sum = sum(math.cos(math.radians(s.heading_deg)) for s in samples)
    avg_heading = math.degrees(math.atan2(sin_sum, cos_sum))
    return Sample(
        vx=sum(s.vx for s in samples) / n,
        vy=sum(s.vy for s in samples) / n,
        vz=sum(s.vz for s in samples) / n,
        yaw_rate=sum(s.yaw_rate for s in samples) / n,
        heading_deg=avg_heading,
    )


# ─── Tiện ích điều khiển ─────────────────────────────────────────────────────

def ned_to_body(vx_ned, vy_ned, heading_deg):
    """Chuyển velocity từ NED frame sang body frame.
    
    Nếu drone không hướng về North, velocity NED cần rotate về body frame
    để so sánh đúng với lệnh gửi (vốn là body frame: tiến/lùi/phải/trái).
    
    body_x (tiến) =  vx_ned * cos(yaw) + vy_ned * sin(yaw)
    body_y (phải) = -vx_ned * sin(yaw) + vy_ned * cos(yaw)
    """
    yaw = math.radians(heading_deg)
    bx =  vx_ned * math.cos(yaw) + vy_ned * math.sin(yaw)
    by = -vx_ned * math.sin(yaw) + vy_ned * math.cos(yaw)
    return bx, by

def to_mavsdk(vx, vy, vz_ned, yaw_dps):
    """Giá trị vật lý → MAVSDK inputs.

    px4_inverse_rc: x=pitch(North) y=roll(East) z=thr[0,1000] r=yaw
    MAVSDK: set_manual_control_input(pitch, roll, throttle[0,1], yaw)

    Từ kết quả thực nghiệm (ảnh):
      TC3 vy=+2.0 → drone bay vy=-0.88 → roll đang bị ĐẢO DẤU
    Fix: đảo dấu roll (y → -y)
    """
    x, y, z, r = position_mode_to_mavlink(vx=vx, vy=vy, vz=vz_ned,
                                           yaw_rate_dps=yaw_dps)
    # Kết quả thực nghiệm từ SITL:
    #   TC2: vx=+3.0 → đo vx=-1.69  → pitch bị đảo  → dùng -x
    #   TC3: vy=+2.0 → đo vy=-0.88  → roll bị đảo   → dùng -y
    #   TC4: vz=-1.0 → đo vz=-0.83  → throttle OK   → giữ nguyên
    #   TC5: yaw=30  → đo 23.1°/s   → yaw OK         → giữ nguyên
    # Kết luận: MAVSDK set_manual_control_input quy ước pitch+/roll+ ngược MAVLink
    # Từ thực nghiệm SITL (terminal log):
    #
    # TC2: pitch=-0.529, hdg=-180° → NED vx=+2.25 → body_fwd=-2.25  FAIL
    #      Drone hướng Nam (-180°), pitch- → bay North (NED vx+) = LÙI trong body
    #      → Muốn TIẾN cần pitch+ → dùng +x
    #
    # TC3: roll=-0.402,  hdg=-180° → NED vy=+1.32 → body_right=-1.32 FAIL
    #      Drone hướng Nam, roll- → bay East (NED vy+) = TRÁI trong body
    #      → Muốn PHẢI cần roll+ → dùng +y
    #
    # TC4: throttle OK (z/1000), TC5: yaw OK (r/1000)
    #
    # KẾT LUẬN: MAVSDK pitch+ = tiến, roll+ = phải (CÙNG chiều MAVLink)
    # Không cần đảo dấu. Lỗi trước do đảo nhầm.
    pitch    = +x / 1000.0   # + = tiến (North khi hdg=0)
    roll     = +y / 1000.0   # + = phải (East  khi hdg=0)
    throttle =  z / 1000.0   # [0,1]: 0.5 = hover
    yaw      =  r / 1000.0   # + = xoay phải
    return pitch, roll, throttle, yaw


async def send_input(drone, pitch, roll, throttle, yaw):
    await drone.manual_control.set_manual_control_input(pitch, roll, throttle, yaw)


async def hover(drone, duration=2.0):
    steps = int(duration * SEND_RATE_HZ)
    for _ in range(steps):
        await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)
        await asyncio.sleep(INTERVAL)


async def send_and_measure(drone, tel: TelemetryCollector,
                           pitch, roll, throttle, yaw) -> list[Sample]:
    """Gửi input + đo: settle rồi collect."""
    steps_settle  = int(SETTLE_TIME  * SEND_RATE_HZ)
    steps_measure = int(MEASURE_TIME * SEND_RATE_HZ)

    # Giai đoạn ổn định — gửi nhưng chưa đo
    for _ in range(steps_settle):
        await send_input(drone, pitch, roll, throttle, yaw)
        await asyncio.sleep(INTERVAL)

    # Giai đoạn đo
    tel.begin_collect()
    for _ in range(steps_measure):
        await send_input(drone, pitch, roll, throttle, yaw)
        await asyncio.sleep(INTERVAL)
    return tel.end_collect()


# ─── Test cases ──────────────────────────────────────────────────────────────

async def tc_hover(drone, tel) -> TestResult:
    print("\n[TC1] Hover — tất cả về 0")
    samples = await send_and_measure(drone, tel, 0.0, 0.0, 0.5, 0.0)
    avg = avg_samples(samples)
    mag = math.sqrt(avg.vx**2 + avg.vy**2 + avg.vz**2)
    passed = mag < 0.5
    return TestResult("TC1 Hover",
        passed=passed,
        expected="|v| < 0.5 m/s",
        actual=f"|v|={mag:.3f} (vx={avg.vx:+.2f} vy={avg.vy:+.2f} vz={avg.vz:+.2f})",
        error_val=f"{mag:.3f} m/s")


async def tc_forward(drone, tel) -> TestResult:
    vx_t = 3.0
    print(f"\n[TC2] Tiến body_x={vx_t} m/s")
    p, r, t, y = to_mavsdk(vx_t, 0, 0, 0)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    # Chuyển NED → body frame theo heading thực tế
    bx, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err = abs(bx - vx_t)
    print(f"  [heading={avg.heading_deg:.1f}°] NED(vx={avg.vx:+.2f} vy={avg.vy:+.2f}) → body(fwd={bx:+.2f} right={by:+.2f})")
    return TestResult(f"TC2 Tiến {vx_t}m/s",
        passed=err < VEL_TOLERANCE,
        expected=f"body_x ≈ {vx_t} m/s (tiến)",
        actual=f"body_x = {bx:+.2f} m/s  [NED: vx={avg.vx:+.2f} vy={avg.vy:+.2f} hdg={avg.heading_deg:.0f}°]",
        error_val=f"{err:.2f} m/s")


async def tc_lateral(drone, tel) -> TestResult:
    vy_t = 2.0
    print(f"\n[TC3] Bay ngang body_y={vy_t} m/s (phải)")
    p, r, t, y = to_mavsdk(0, vy_t, 0, 0)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    bx, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err = abs(by - vy_t)
    print(f"  [heading={avg.heading_deg:.1f}°] NED(vx={avg.vx:+.2f} vy={avg.vy:+.2f}) → body(fwd={bx:+.2f} right={by:+.2f})")
    return TestResult(f"TC3 Ngang {vy_t}m/s",
        passed=err < VEL_TOLERANCE,
        expected=f"body_y ≈ +{vy_t} m/s (phải)",
        actual=f"body_y = {by:+.2f} m/s  [NED: vx={avg.vx:+.2f} vy={avg.vy:+.2f} hdg={avg.heading_deg:.0f}°]",
        error_val=f"{err:.2f} m/s")


async def tc_climb(drone, tel) -> TestResult:
    vz_t = -1.0  # NED âm = lên
    print(f"\n[TC4] Lên cao vz={vz_t} m/s (NED)")
    p, r, t, y = to_mavsdk(0, 0, vz_t, 0)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.vz - vz_t)
    return TestResult("TC4 Lên vz=-1",
        passed=err < VEL_TOLERANCE,
        expected=f"vz ≈ {vz_t} m/s",
        actual=f"vz = {avg.vz:+.2f} m/s",
        error_val=f"{err:.2f} m/s")


async def tc_yaw(drone, tel) -> TestResult:
    yr_t = 30.0
    print(f"\n[TC5] Xoay yaw={yr_t} °/s")
    p, r, t, y = to_mavsdk(0, 0, 0, yr_t)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.yaw_rate - yr_t)
    return TestResult(f"TC5 Yaw {yr_t}°/s",
        passed=err < YAW_TOLERANCE,
        expected=f"yaw_rate ≈ {yr_t} °/s",
        actual=f"yaw_rate = {avg.yaw_rate:+.1f} °/s",
        error_val=f"{err:.1f} °/s")


async def tc_combined(drone, tel) -> TestResult:
    vx_t, vy_t, vz_t, yr_t = 2.0, 1.0, -0.5, 15.0
    print(f"\n[TC6] Kết hợp vx={vx_t} vy={vy_t} vz={vz_t} yaw={yr_t}°/s")
    p, r, t, y = to_mavsdk(vx_t, vy_t, vz_t, yr_t)
    print(f"  → pitch={p:+.3f} roll={r:+.3f} thr={t:.3f} yaw={y:+.3f}")
    samples = await send_and_measure(drone, tel, p, r, t, y)
    avg = avg_samples(samples)
    bx, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    vh = math.sqrt(bx**2 + by**2)
    vh_t = math.sqrt(vx_t**2 + vy_t**2)
    err_bx = abs(bx - vx_t)
    err_by = abs(by - vy_t)
    err_z  = abs(avg.vz - vz_t)
    print(f"  [heading={avg.heading_deg:.1f}°] body(fwd={bx:+.2f} right={by:+.2f})")
    passed = err_bx < VEL_TOLERANCE and err_by < VEL_TOLERANCE and err_z < VEL_TOLERANCE
    return TestResult("TC6 Kết hợp",
        passed=passed,
        expected=f"body fwd≈{vx_t} right≈{vy_t} vz≈{vz_t}",
        actual=f"body fwd={bx:+.2f} right={by:+.2f} vz={avg.vz:+.2f} [hdg={avg.heading_deg:.0f}°]",
        error_val=f"fwd:{err_bx:.2f} right:{err_by:.2f} z:{err_z:.2f} m/s")


def print_results(results: list[TestResult]):
    print("\n" + "=" * 70)
    print("KẾT QUẢ KIỂM CHỨNG — px4_inverse_rc + MAVSDK")
    print("=" * 70)
    n_pass = sum(1 for r in results if r.passed)
    for r in results:
        icon = "✅" if r.passed else "❌"
        print(f"\n{icon} {r.name}")
        print(f"   Gửi đi  : {r.expected}")
        print(f"   Đo được : {r.actual}")
        print(f"   Sai số  : {r.error_val}")
    print("\n" + "-" * 70)
    print(f"TỔNG KẾT: {n_pass}/{len(results)} PASS "
          f"{'✅ TẤT CẢ OK' if n_pass == len(results) else '❌ CÓ TEST FAIL'}")
    print("=" * 70)


# ─── Main ────────────────────────────────────────────────────────────────────

async def run():
    drone = System()
    print(f"Kết nối {SITL_ADDRESS}...")
    await drone.connect(system_address=SITL_ADDRESS)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✓ Kết nối thành công!")
            break

    # Khởi động telemetry collector
    tel = TelemetryCollector()
    await tel.start(drone)
    await tel.start_angular(drone)

    # Chờ health
    print("Chờ GPS + EKF...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✓ Health OK")
            break

    # RC init
    print("Gửi RC init...")
    await hover(drone, 1.5)

    # ARM
    print("Arming...")
    try:
        await drone.action.arm()
        print("✓ Armed!")
    except Exception as e:
        print(f"✗ Arm thất bại: {e}")
        return

    # Takeoff
    print("Takeoff 12m...")
    await drone.action.set_takeoff_altitude(12.0)
    await drone.action.takeoff()

    # Chờ đạt độ cao — vừa chờ vừa theo dõi
    print("Chờ đạt độ cao...")
    t0 = asyncio.get_event_loop().time()
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        if asyncio.get_event_loop().time() - t0 > 1:
            print(f"  Alt: {alt:.1f}m", end="\r")
        if alt >= 10.0:
            print(f"\n✓ Đạt {alt:.1f}m")
            break
        if asyncio.get_event_loop().time() - t0 > 30:
            print(f"\nTimeout takeoff, alt={alt:.1f}m — tiếp tục...")
            break

    await asyncio.sleep(1.0)

    # Kích hoạt Position Control
    print("\nKích hoạt Position Control...")
    try:
        # Thử bật position control — nếu fail thì thử altitude control
        await drone.manual_control.start_position_control()
        print("✓ Position Control OK")
        mode_label = "POSCTL"
    except ManualControlError as e:
        print(f"  Position control thất bại ({e}), thử Altitude Control...")
        try:
            await drone.manual_control.start_altitude_control()
            print("✓ Altitude Control OK")
            mode_label = "ALTCTL"
        except ManualControlError as e2:
            print(f"✗ Cả hai đều thất bại: {e2}")
            return

    await hover(drone, 2.0)  # Ổn định

    # ── Chạy test cases ────────────────────────────────────────────────────
    print(f"\n{'='*70}")
    print(f"BẮT ĐẦU KIỂM CHỨNG ({mode_label}) — so sánh target vs telemetry thực tế")
    print(f"  Settle={SETTLE_TIME}s | Measure={MEASURE_TIME}s | Tolerance vel={VEL_TOLERANCE}m/s")
    print(f"{'='*70}")

    results: list[TestResult] = []
    test_funcs = [tc_hover, tc_forward, tc_lateral, tc_climb, tc_yaw, tc_combined]

    for tf in test_funcs:
        try:
            result = await tf(drone, tel)
            results.append(result)
            icon = "✅" if result.passed else "❌"
            print(f"  {icon} {result.actual}  (sai số: {result.error_val})")
        except Exception as ex:
            print(f"  ❌ Exception: {ex}")
            results.append(TestResult(
                name=tf.__name__, passed=False,
                expected="—", actual=f"Exception: {ex}", error_val=str(ex)))
        # Hover giữa các test
        await hover(drone, 2.0)

    # ── Kết quả ────────────────────────────────────────────────────────────
    print_results(results)

    # Landing
    print("\nLanding...")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("✓ Hạ cánh!")
            break

    await drone.action.disarm()
    tel.stop()
    print("✓ Hoàn tất.")


if __name__ == "__main__":
    asyncio.run(run())
