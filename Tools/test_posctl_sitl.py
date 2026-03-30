#!/usr/bin/env python3
"""
test_posctl_sitl.py -- Kiem chung px4_inverse_rc o POSITION mode bang van toc
==============================================================================
Input: van toc body frame (vx, vy, vz, yaw_dps)
Do: velocity NED thuc te tu telemetry, chuyen ve body frame de so sanh

Chay tren SITL:
  python3 test_posctl_sitl.py

Chay tren drone that:
  python3 test_posctl_sitl.py --connection serial:///dev/ttyACM0:57600
"""

import asyncio
import sys
import os
import math
import argparse
from dataclasses import dataclass
from typing import List

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_inverse_rc import position_mode_to_mavlink

from mavsdk import System
from mavsdk.manual_control import ManualControlError

# ── Cau hinh ──────────────────────────────────────────────────────────────────
SITL_ADDRESS  = "udp://:14540"
SEND_RATE_HZ  = 20
INTERVAL      = 1.0 / SEND_RATE_HZ
SETTLE_TIME   = 4.0   # s -- cho drone on dinh
MEASURE_TIME  = 3.0   # s -- do thuc te
MIN_ALT_M     = 5.0   # m -- do cao toi thieu de bat dau test

# Tolerance
VEL_TOL = 0.8   # m/s
YAW_TOL = 20.0  # deg/s

# Gia tri test
VX_TEST  = 3.0   # m/s tien/lui
VY_TEST  = 2.0   # m/s trai/phai
VZ_TEST  = 1.0   # m/s len
YAW_TEST = 30.0  # deg/s


# ── Data classes ───────────────────────────────────────────────────────────────

@dataclass
class DroneParams:
    vel_max:    float = 10.0
    vel_max_up: float = 3.0
    vel_max_dn: float = 1.5
    yaw_max:    float = 150.0
    deadzone:   float = 0.0
    source:     str   = "default"


@dataclass
class Sample:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate:    float = 0.0
    heading_deg: float = 0.0
    pitch_deg:   float = 0.0
    roll_deg:    float = 0.0
    alt_m:       float = 0.0


@dataclass
class TestResult:
    name:      str
    passed:    bool
    expected:  str
    actual:    str
    error_val: str


# ── Doc param ──────────────────────────────────────────────────────────────────

async def read_params(drone: System) -> DroneParams:
    PMAP = {
        'MPC_VEL_MANUAL':   ('vel_max',    10.0),
        'MPC_Z_VEL_MAX_UP': ('vel_max_up',  3.0),
        'MPC_Z_VEL_MAX_DN': ('vel_max_dn',  1.5),
        'MPC_MAN_Y_MAX':    ('yaw_max',   150.0),
        'MAN_DEADZONE':     ('deadzone',    0.0),
    }
    print("\nDoc param tu drone...")
    values = {}
    for px4_name, (field, default) in PMAP.items():
        try:
            val = await drone.param.get_param_float(px4_name)
            diff = abs(val - default) / max(abs(default), 0.001) * 100
            flag = " <- KHAC DEFAULT ({})".format(default) if diff > 1.0 else ""
            print("  {} = {:.3f}{}".format(px4_name, val, flag))
            values[field] = val
        except Exception:
            print("  {} = {} (default)".format(px4_name, default))
            values[field] = default
    p = DroneParams(**values, source="drone")
    print("  OK\n")
    return p


# ── Telemetry ──────────────────────────────────────────────────────────────────

class TelemetryCollector:
    def __init__(self):
        self.latest = Sample()
        self.samples: List[Sample] = []
        self.collecting = False
        self.current_mode = "UNKNOWN"
        self._tasks = []

    async def start(self, drone: System):
        self._tasks += [
            asyncio.ensure_future(self._vel(drone)),
            asyncio.ensure_future(self._att(drone)),
            asyncio.ensure_future(self._ang(drone)),
            asyncio.ensure_future(self._pos(drone)),
            asyncio.ensure_future(self._mode(drone)),
        ]

    async def _vel(self, drone):
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

    async def _att(self, drone):
        async for a in drone.telemetry.attitude_euler():
            self.latest.heading_deg = a.yaw_deg
            self.latest.pitch_deg   = a.pitch_deg
            self.latest.roll_deg    = a.roll_deg

    async def _ang(self, drone):
        async for a in drone.telemetry.attitude_angular_velocity_body():
            self.latest.yaw_rate = math.degrees(a.yaw_rad_s)

    async def _pos(self, drone):
        async for p in drone.telemetry.position():
            self.latest.alt_m = p.relative_altitude_m

    async def _mode(self, drone):
        async for m in drone.telemetry.flight_mode():
            s = str(m)
            if s != self.current_mode:
                print("  [Mode] {} -> {}".format(self.current_mode, s))
                self.current_mode = s

    def begin_collect(self):
        self.samples.clear()
        self.collecting = True

    def end_collect(self) -> List[Sample]:
        self.collecting = False
        return list(self.samples)

    def stop(self):
        for t in self._tasks:
            t.cancel()


def avg_samples(samples: List[Sample]) -> Sample:
    if not samples:
        return Sample()
    n = len(samples)
    sh = sum(math.sin(math.radians(s.heading_deg)) for s in samples)
    ch = sum(math.cos(math.radians(s.heading_deg)) for s in samples)
    return Sample(
        vx=sum(s.vx for s in samples) / n,
        vy=sum(s.vy for s in samples) / n,
        vz=sum(s.vz for s in samples) / n,
        yaw_rate=sum(s.yaw_rate for s in samples) / n,
        heading_deg=math.degrees(math.atan2(sh, ch)),
        pitch_deg=sum(s.pitch_deg for s in samples) / n,
        roll_deg=sum(s.roll_deg for s in samples) / n,
        alt_m=sum(s.alt_m for s in samples) / n,
    )


def ned_to_body(vx_ned, vy_ned, hdg):
    """Chuyen velocity NED -> body frame de so sanh."""
    yaw = math.radians(hdg)
    bx =  vx_ned * math.cos(yaw) + vy_ned * math.sin(yaw)
    by = -vx_ned * math.sin(yaw) + vy_ned * math.cos(yaw)
    return bx, by


# ── Control ────────────────────────────────────────────────────────────────────

def to_input(vx=0.0, vy=0.0, vz=0.0, yaw_dps=0.0, p: DroneParams = None):
    """Van toc body frame -> MAVSDK input.
    vx: tien(+)/lui(-) theo mui drone
    vy: phai(+)/trai(-) theo than drone
    vz: NED -- xuong(+)/len(-)
    """
    p = p or DroneParams()
    x, y, z, r = position_mode_to_mavlink(
        vx=vx, vy=vy, vz=vz, yaw_rate_dps=yaw_dps,
        vel_max=p.vel_max,
        vel_max_up=p.vel_max_up,
        vel_max_dn=p.vel_max_dn,
        yaw_max=p.yaw_max,
        deadzone=p.deadzone,
    )
    # Mapping da xac nhan tu thuc nghiem POSCTL:
    # pitch = +x/1000, roll = +y/1000
    return +x/1000.0, +y/1000.0, z/1000.0, r/1000.0


async def hover(drone, duration=2.0):
    for _ in range(int(duration * SEND_RATE_HZ)):
        await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)
        await asyncio.sleep(INTERVAL)


async def send_and_measure(drone, tel, pitch, roll, thr, yaw) -> List[Sample]:
    for _ in range(int(SETTLE_TIME * SEND_RATE_HZ)):
        await drone.manual_control.set_manual_control_input(pitch, roll, thr, yaw)
        await asyncio.sleep(INTERVAL)
    tel.begin_collect()
    for _ in range(int(MEASURE_TIME * SEND_RATE_HZ)):
        await drone.manual_control.set_manual_control_input(pitch, roll, thr, yaw)
        await asyncio.sleep(INTERVAL)
    return tel.end_collect()


# ── Test cases ─────────────────────────────────────────────────────────────────

async def tc_hover(drone, tel, p) -> TestResult:
    print("\n[TC1] Hover -- stick giua")
    samples = await send_and_measure(drone, tel, 0.0, 0.0, 0.5, 0.0)
    avg = avg_samples(samples)
    mag = math.sqrt(avg.vx**2 + avg.vy**2 + avg.vz**2)
    passed = mag < 0.5
    print("  |v|={:.2f}m/s pitch={:+.1f} roll={:+.1f}".format(
        mag, avg.pitch_deg, avg.roll_deg))
    return TestResult("TC1 Hover", passed,
        "|v| < 0.5 m/s",
        "|v|={:.3f}m/s".format(mag),
        "{:.3f}m/s".format(mag))


async def tc_forward(drone, tel, p) -> TestResult:
    vx_t = VX_TEST
    print("\n[TC2] Tien vx=+{} m/s (body frame)".format(vx_t))
    pt, r, t, y = to_input(vx=vx_t, p=p)
    print("  MAVSDK: pitch={:+.3f} roll={:+.3f} thr={:.3f}".format(pt, r, t))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    bx, _ = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err = abs(bx - vx_t)
    passed = err < VEL_TOL and bx > 0
    print("  [hdg={:.0f}] body_fwd={:+.2f}m/s (target={:+.1f})".format(
        avg.heading_deg, bx, vx_t))
    return TestResult("TC2 Tien +{}m/s".format(vx_t), passed,
        "body_fwd ~= +{} m/s".format(vx_t),
        "body_fwd={:+.2f}m/s".format(bx),
        "{:.2f}m/s".format(err))


async def tc_backward(drone, tel, p) -> TestResult:
    vx_t = -VX_TEST
    print("\n[TC3] Lui vx={} m/s (body frame)".format(vx_t))
    pt, r, t, y = to_input(vx=vx_t, p=p)
    print("  MAVSDK: pitch={:+.3f}".format(pt))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    bx, _ = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err = abs(bx - vx_t)
    passed = err < VEL_TOL and bx < 0
    print("  [hdg={:.0f}] body_fwd={:+.2f}m/s (target={:+.1f})".format(
        avg.heading_deg, bx, vx_t))
    return TestResult("TC3 Lui {}m/s".format(vx_t), passed,
        "body_fwd ~= {} m/s".format(vx_t),
        "body_fwd={:+.2f}m/s".format(bx),
        "{:.2f}m/s".format(err))


async def tc_right(drone, tel, p) -> TestResult:
    vy_t = VY_TEST
    print("\n[TC4] Sang phai vy=+{} m/s (body frame)".format(vy_t))
    pt, r, t, y = to_input(vy=vy_t, p=p)
    print("  MAVSDK: roll={:+.3f}".format(r))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    _, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err = abs(by - vy_t)
    passed = err < VEL_TOL and by > 0
    print("  [hdg={:.0f}] body_right={:+.2f}m/s (target={:+.1f})".format(
        avg.heading_deg, by, vy_t))
    return TestResult("TC4 Phai +{}m/s".format(vy_t), passed,
        "body_right ~= +{} m/s".format(vy_t),
        "body_right={:+.2f}m/s".format(by),
        "{:.2f}m/s".format(err))


async def tc_left(drone, tel, p) -> TestResult:
    vy_t = -VY_TEST
    print("\n[TC5] Sang trai vy={} m/s (body frame)".format(vy_t))
    pt, r, t, y = to_input(vy=vy_t, p=p)
    print("  MAVSDK: roll={:+.3f}".format(r))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    _, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    err = abs(by - vy_t)
    passed = err < VEL_TOL and by < 0
    print("  [hdg={:.0f}] body_right={:+.2f}m/s (target={:+.1f})".format(
        avg.heading_deg, by, vy_t))
    return TestResult("TC5 Trai {}m/s".format(vy_t), passed,
        "body_right ~= {} m/s".format(vy_t),
        "body_right={:+.2f}m/s".format(by),
        "{:.2f}m/s".format(err))


async def tc_climb(drone, tel, p) -> TestResult:
    vz_t = -VZ_TEST  # NED: am = len
    print("\n[TC6] Len vz={} m/s (NED)".format(vz_t))
    pt, r, t, y = to_input(vz=vz_t, p=p)
    print("  MAVSDK: thr={:.3f}".format(t))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.vz - vz_t)
    passed = err < VEL_TOL
    print("  vz_actual={:+.2f}m/s alt={:.1f}m".format(avg.vz, avg.alt_m))
    return TestResult("TC6 Len {}m/s".format(vz_t), passed,
        "vz ~= {} m/s".format(vz_t),
        "vz={:+.2f}m/s alt={:.1f}m".format(avg.vz, avg.alt_m),
        "{:.2f}m/s".format(err))


async def tc_descend(drone, tel, p) -> TestResult:
    vz_t = 0.5  # NED: duong = xuong
    print("\n[TC7] Xuong vz=+{} m/s (NED)".format(vz_t))
    pt, r, t, y = to_input(vz=vz_t, p=p)
    print("  MAVSDK: thr={:.3f}".format(t))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.vz - vz_t)
    passed = err < VEL_TOL and avg.vz > 0
    print("  vz_actual={:+.2f}m/s alt={:.1f}m".format(avg.vz, avg.alt_m))
    return TestResult("TC7 Xuong +{}m/s".format(vz_t), passed,
        "vz ~= +{} m/s".format(vz_t),
        "vz={:+.2f}m/s alt={:.1f}m".format(avg.vz, avg.alt_m),
        "{:.2f}m/s".format(err))


async def tc_yaw(drone, tel, p) -> TestResult:
    yr_t = YAW_TEST
    print("\n[TC8] Yaw phai +{} deg/s".format(yr_t))
    pt, r, t, y = to_input(yaw_dps=yr_t, p=p)
    print("  MAVSDK: yaw={:+.3f}".format(y))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    err = abs(avg.yaw_rate - yr_t)
    passed = err < YAW_TOL
    print("  yaw_rate={:+.1f}deg/s".format(avg.yaw_rate))
    return TestResult("TC8 Yaw +{}deg/s".format(yr_t), passed,
        "yaw_rate ~= +{} deg/s".format(yr_t),
        "yaw_rate={:+.1f}deg/s".format(avg.yaw_rate),
        "{:.1f}deg/s".format(err))


async def tc_combined(drone, tel, p) -> TestResult:
    vx_t, vy_t, vz_t, yr_t = VX_TEST*0.5, VY_TEST*0.5, -0.5, 15.0
    print("\n[TC9] Ket hop vx={} vy={} vz={} yaw={}".format(vx_t, vy_t, vz_t, yr_t))
    pt, r, t, y = to_input(vx=vx_t, vy=vy_t, vz=vz_t, yaw_dps=yr_t, p=p)
    print("  MAVSDK: pitch={:+.3f} roll={:+.3f} thr={:.3f} yaw={:+.3f}".format(pt, r, t, y))
    samples = await send_and_measure(drone, tel, pt, r, t, y)
    avg = avg_samples(samples)
    bx, by = ned_to_body(avg.vx, avg.vy, avg.heading_deg)
    ex = abs(bx - vx_t)
    ey = abs(by - vy_t)
    ez = abs(avg.vz - vz_t)
    passed = ex < VEL_TOL and ey < VEL_TOL and ez < VEL_TOL
    print("  body(fwd={:+.2f} right={:+.2f}) vz={:+.2f}".format(bx, by, avg.vz))
    return TestResult("TC9 Ket hop", passed,
        "fwd~={} right~={} vz~={}".format(vx_t, vy_t, vz_t),
        "fwd={:+.2f} right={:+.2f} vz={:+.2f}".format(bx, by, avg.vz),
        "ex={:.2f} ey={:.2f} ez={:.2f}".format(ex, ey, ez))


# ── Print results ──────────────────────────────────────────────────────────────

def print_results(results: List[TestResult], p: DroneParams):
    print("\n" + "=" * 70)
    print("KET QUA KIEM CHUNG -- POSITION MODE + px4_inverse_rc")
    print("Param source: {}".format(p.source))
    print("=" * 70)
    n_pass = sum(1 for r in results if r.passed)
    for r in results:
        icon = "OK" if r.passed else "FAIL"
        print("\n[{}] {}".format(icon, r.name))
        print("  Expect : {}".format(r.expected))
        print("  Actual : {}".format(r.actual))
        print("  Sai so : {}".format(r.error_val))
    print("\n" + "-" * 70)
    print("TONG KET: {}/{} PASS {}".format(
        n_pass, len(results),
        "-- TAT CA OK" if n_pass == len(results) else "-- CO FAIL"))
    print("=" * 70)


# ── Wait for POSCTL ────────────────────────────────────────────────────────────

async def wait_for_posctl(tel: TelemetryCollector):
    print("\n" + "=" * 70)
    print("Dung QGC: arm -> takeoff -> chuyen POSITION mode")
    print("Script tu bat dau khi POSCTL + alt > {}m".format(MIN_ALT_M))
    print("=" * 70)
    t0 = asyncio.get_event_loop().time()
    last_print = 0.0
    while True:
        elapsed = asyncio.get_event_loop().time() - t0
        if elapsed - last_print >= 5.0:
            print("  [{:.0f}s] Mode={}  Alt={:.1f}m".format(
                elapsed, tel.current_mode, tel.latest.alt_m))
            last_print = elapsed
        mode_str = tel.current_mode.upper()
        is_pos = "POS" in mode_str
        has_alt = tel.latest.alt_m >= MIN_ALT_M
        if is_pos and has_alt:
            print("\nPOSCTL + Alt={:.1f}m -- bat dau test!".format(tel.latest.alt_m))
            return
        # Fallback: du cao nhung chua POSCTL
        if has_alt and mode_str not in ("UNKNOWN","DISARMED","LAND","RTL","") and not is_pos:
            print("\n  Mode={} Alt={:.1f}m -- chua POSCTL".format(
                tel.current_mode, tel.latest.alt_m))
            print("  Chuyen sang POSITION mode trong QGC...")
        await asyncio.sleep(0.3)


# ── Main ───────────────────────────────────────────────────────────────────────

async def run(args):
    drone = System()
    print("Ket noi {}...".format(args.connection))
    await drone.connect(system_address=args.connection)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Ket noi OK!")
            break

    tel = TelemetryCollector()
    await tel.start(drone)
    await asyncio.sleep(1.5)

    if args.use_defaults:
        p = DroneParams()
        print("Dung param default")
    else:
        p = await read_params(drone)

    await wait_for_posctl(tel)

    # Warmup -- giong test_altitude.py da hoat dong
    print("\n[WARMUP] Gui MANUAL_CONTROL 3s de PX4 nhan dien RC source...")
    print("  (Nha joystick QGC ra neu dang giu)")
    await hover(drone, 3.0)

    # Goi start_position_control sau khi da co RC history
    activated = False
    for attempt in range(3):
        try:
            await drone.manual_control.start_position_control()
            print("Position control OK")
            activated = True
            break
        except Exception as e:
            print("  Lan {}: {} -- warmup them...".format(attempt+1, e))
            await hover(drone, 2.0)

    # Check phan hoi bang thr cao
    print("[CHECK] Kiem tra phan hoi (gui thr=0.7)...")
    for _ in range(int(1.0 * SEND_RATE_HZ)):
        await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.7, 0.0)
        await asyncio.sleep(INTERVAL)
    alt_before = tel.latest.alt_m
    await asyncio.sleep(0.3)
    alt_after = tel.latest.alt_m
    if alt_after - alt_before > 0.05 or tel.latest.vz < -0.1:
        print("  Drone phan hoi OK (alt {:.1f}->{:.1f}m)".format(alt_before, alt_after))
    else:
        print("  Drone CHUA phan hoi -- warmup them 5s...")
        await hover(drone, 5.0)
        try:
            await drone.manual_control.start_position_control()
            print("  Position control OK lan 2")
        except Exception as e:
            print("  {}".format(e))

    # Hover on dinh truoc khi test
    print("Hover on dinh 3s...")
    await hover(drone, 3.0)

    print("\n" + "=" * 70)
    print("BAT DAU TEST -- POSITION MODE -- van toc body frame")
    print("  Settle={}s | Measure={}s".format(SETTLE_TIME, MEASURE_TIME))
    print("  vx_test={} vy_test={} vz_test={} yaw_test={}".format(
        VX_TEST, VY_TEST, VZ_TEST, YAW_TEST))
    print("=" * 70)

    ALL_TESTS = [
        tc_hover,
        tc_forward,
        tc_backward,
        tc_right,
        tc_left,
        tc_climb,
        tc_descend,
        tc_yaw,
        tc_combined,
    ]

    results: List[TestResult] = []
    for tf in ALL_TESTS:
        if tel.latest.alt_m < 3.0:
            print("\nAlt={:.1f}m qua thap -- dung!".format(tel.latest.alt_m))
            break
        try:
            result = await tf(drone, tel, p)
            results.append(result)
            icon = "OK" if result.passed else "FAIL"
            print("  [{}] {}  (sai so: {})".format(icon, result.actual, result.error_val))
        except Exception as ex:
            print("  [FAIL] Exception: {}".format(ex))
            results.append(TestResult(tf.__name__, False, "--",
                "Exception: {}".format(ex), str(ex)))
        await hover(drone, 2.0)

    print_results(results, p)
    print("\nTest xong -- dung QGC de landing.")
    tel.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connection', '-c', default=SITL_ADDRESS)
    parser.add_argument('--use-defaults', action='store_true')
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
