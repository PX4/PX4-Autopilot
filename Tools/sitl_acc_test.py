#!/usr/bin/env python3
"""
sitl_acc_test.py
================
SITL integration test for external NED acceleration control
(Position Mode & Altitude Mode via acc_sp_external uORB topic).

Test Cases
----------
  TC-01  Smoke test  — acc_sp_external topic received by firmware
  TC-02  Clamping    — oversized acc clamped to MPC_ACC_HOR (~5 m/s²)
  TC-03  Kinematics  — step acc matches theoretical pos/vel trend
  TC-04  Watchdog    — stop publishing → velocity decays, drone does not diverge
  TC-05  Square      — four-leg trajectory returns near origin

Usage
-----
  # Terminal 1 — start SITL
  make px4_sitl gazebo-classic_iris

  # Terminal 2 — run tests
  python3 Tools/sitl_acc_test.py [--connection udp:127.0.0.1:14540] [--mode position|altitude]

Outputs
-------
  logs/sitl_acc_test_<timestamp>/
      telemetry.csv   — full MAVLink telemetry stream
      commands.csv    — every acceleration command sent
      results.txt     — pass/fail summary per test case
      <tc_id>.csv     — per-test trimmed dataset for offline analysis
"""

import argparse
import asyncio
import csv
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Dict

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("ERROR: pip install pymavlink")

try:
    from mavsdk import System
    from mavsdk.action import ActionError
except ImportError:
    sys.exit("ERROR: pip install mavsdk")

# Add Tools/ to path so acceleration_control is importable
sys.path.insert(0, str(Path(__file__).parent))
from acceleration_control import AccelerationControl, _ACC_SP_EXTERNAL_FLAG


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class TelemetrySample:
    """One telemetry snapshot from MAVLink."""
    wall_time_s:    float = 0.0   # host wall-clock seconds (float)
    boot_time_ms:   int   = 0     # vehicle time_boot_ms
    # Position NED (m)
    pos_x:  float = float('nan')  # North
    pos_y:  float = float('nan')  # East
    pos_z:  float = float('nan')  # Down (negative = up)
    # Velocity NED (m/s)
    vel_x:  float = float('nan')
    vel_y:  float = float('nan')
    vel_z:  float = float('nan')
    # Attitude (rad)
    roll:   float = float('nan')
    pitch:  float = float('nan')
    yaw:    float = float('nan')
    # Derived
    speed_h: float = float('nan') # horizontal speed m/s
    tilt_deg: float = float('nan')# total tilt angle degrees
    # Flight state
    armed:      bool  = False
    nav_state:  int   = 0
    mode_str:   str   = ''
    # Commanded acceleration (what we sent last)
    cmd_ax: float = 0.0
    cmd_ay: float = 0.0
    cmd_az: float = 0.0


@dataclass
class CommandRecord:
    """One acceleration command sent."""
    wall_time_s: float
    ax: float
    ay: float
    az: float
    yaw_rad: float
    test_case: str


@dataclass
class TestResult:
    tc_id:    str
    name:     str
    passed:   bool
    reason:   str
    metrics:  Dict = field(default_factory=dict)


# ─────────────────────────────────────────────────────────────────────────────
# Telemetry receiver (background thread)
# ─────────────────────────────────────────────────────────────────────────────

_PX4_NAV_STATES = {
    0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION',
    4: 'AUTO_LOITER', 5: 'AUTO_RTL', 14: 'OFFBOARD',
    15: 'STAB', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND',
}


class TelemetryReceiver:
    """
    Subscribes to MAVLink messages in a background thread and maintains
    the latest telemetry snapshot. Thread-safe reads via a lock.
    """

    def __init__(self, mav: mavutil.mavfile):
        self._mav = mav
        self._lock = threading.Lock()
        self._latest = TelemetrySample()
        self._history: List[TelemetrySample] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._t0 = time.time()

        # partial state (updated by different message types)
        self._pos  = [float('nan')] * 3
        self._vel  = [float('nan')] * 3
        self._att  = [float('nan')] * 3  # roll, pitch, yaw
        self._armed    = False
        self._nav_state = 0
        self._boot_ms  = 0

    def start(self):
        self._running = True
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def set_last_command(self, ax: float, ay: float, az: float):
        with self._lock:
            self._latest.cmd_ax = ax
            self._latest.cmd_ay = ay
            self._latest.cmd_az = az

    def get_latest(self) -> TelemetrySample:
        with self._lock:
            return TelemetrySample(**asdict(self._latest))

    def get_history(self) -> List[TelemetrySample]:
        with self._lock:
            return list(self._history)

    def clear_history(self):
        with self._lock:
            self._history.clear()

    # ------------------------------------------------------------------
    def _loop(self):
        while self._running:
            msg = self._mav.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            mt = msg.get_type()

            if mt == 'LOCAL_POSITION_NED':
                self._pos = [msg.x, msg.y, msg.z]
                self._vel = [msg.vx, msg.vy, msg.vz]
                self._boot_ms = msg.time_boot_ms
                self._commit_sample()

            elif mt == 'ATTITUDE':
                self._att = [msg.roll, msg.pitch, msg.yaw]

            elif mt == 'HEARTBEAT':
                base = msg.base_mode
                armed = bool(base & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._armed = armed
                self._nav_state = (msg.custom_mode >> 16) & 0xFF  # PX4: main_mode in bits 16-23

    def _commit_sample(self):
        """Build a TelemetrySample from current partial state and append."""
        wall = time.time() - self._t0
        vx, vy, vz = self._vel
        roll, pitch, yaw = self._att

        speed_h = math.sqrt(vx**2 + vy**2) if all(
            not math.isnan(v) for v in [vx, vy]) else float('nan')
        tilt_deg = math.degrees(math.sqrt(roll**2 + pitch**2)) if all(
            not math.isnan(a) for a in [roll, pitch]) else float('nan')

        with self._lock:
            s = TelemetrySample(
                wall_time_s  = round(wall, 4),
                boot_time_ms = self._boot_ms,
                pos_x = self._pos[0], pos_y = self._pos[1], pos_z = self._pos[2],
                vel_x = vx,           vel_y = vy,            vel_z = vz,
                roll  = round(roll,  4) if not math.isnan(roll)  else float('nan'),
                pitch = round(pitch, 4) if not math.isnan(pitch) else float('nan'),
                yaw   = round(yaw,   4) if not math.isnan(yaw)   else float('nan'),
                speed_h   = round(speed_h,   3) if not math.isnan(speed_h)   else float('nan'),
                tilt_deg  = round(tilt_deg,  2) if not math.isnan(tilt_deg)  else float('nan'),
                armed      = self._armed,
                nav_state  = self._nav_state,
                mode_str   = _PX4_NAV_STATES.get(self._nav_state, str(self._nav_state)),
                cmd_ax = self._latest.cmd_ax,
                cmd_ay = self._latest.cmd_ay,
                cmd_az = self._latest.cmd_az,
            )
            self._latest = s
            self._history.append(TelemetrySample(**asdict(s)))


# ─────────────────────────────────────────────────────────────────────────────
# Logger
# ─────────────────────────────────────────────────────────────────────────────

TELEMETRY_FIELDS = list(TelemetrySample.__dataclass_fields__.keys())
COMMAND_FIELDS   = list(CommandRecord.__dataclass_fields__.keys())


class Logger:
    def __init__(self, log_dir: Path):
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir = log_dir
        self._tele_file  = open(log_dir / 'telemetry.csv',  'w', newline='')
        self._cmd_file   = open(log_dir / 'commands.csv',   'w', newline='')
        self._tele_writer = csv.DictWriter(self._tele_file, fieldnames=TELEMETRY_FIELDS)
        self._cmd_writer  = csv.DictWriter(self._cmd_file,  fieldnames=COMMAND_FIELDS)
        self._tele_writer.writeheader()
        self._cmd_writer.writeheader()
        self._results: List[TestResult] = []

    def log_telemetry(self, s: TelemetrySample):
        self._tele_writer.writerow(asdict(s))
        self._tele_file.flush()

    def log_command(self, cmd: CommandRecord):
        self._cmd_writer.writerow(asdict(cmd))
        self._cmd_file.flush()

    def log_result(self, r: TestResult):
        self._results.append(r)
        status = 'PASS' if r.passed else 'FAIL'
        print(f"\n  [{status}] {r.tc_id}: {r.name}")
        print(f"         {r.reason}")
        if r.metrics:
            for k, v in r.metrics.items():
                print(f"         {k:30s} = {v}")

    def save_tc_snapshot(self, tc_id: str, samples: List[TelemetrySample]):
        """Save trimmed dataset for a single test case."""
        path = self.log_dir / f'{tc_id}.csv'
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=TELEMETRY_FIELDS)
            w.writeheader()
            for s in samples:
                w.writerow(asdict(s))

    def write_summary(self):
        path = self.log_dir / 'results.txt'
        total  = len(self._results)
        passed = sum(1 for r in self._results if r.passed)
        lines = [
            '=' * 60,
            f'  SITL Acceleration Control Test Results',
            f'  {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            '=' * 60,
            f'  Passed: {passed}/{total}',
            '',
        ]
        for r in self._results:
            status = 'PASS' if r.passed else 'FAIL'
            lines.append(f'  [{status}]  {r.tc_id}  {r.name}')
            lines.append(f'           {r.reason}')
            for k, v in r.metrics.items():
                lines.append(f'           {k} = {v}')
            lines.append('')
        lines.append('=' * 60)
        with open(path, 'w') as f:
            f.write('\n'.join(lines))
        print('\n' + '\n'.join(lines))

    def close(self):
        self._tele_file.close()
        self._cmd_file.close()


# ─────────────────────────────────────────────────────────────────────────────
# Test runner
# ─────────────────────────────────────────────────────────────────────────────

class SITLAccTest:

    # Kinematic tolerance (fraction)
    KINEMATIC_TOL = 0.30     # 30% of theoretical value
    # Watchdog — max velocity after 1s of no command (m/s)
    WATCHDOG_MAX_SPEED = 0.5
    # Square — max distance from origin at end (m)
    SQUARE_RETURN_TOL = 3.0
    # Tilt clamp: drone must respond (> MIN) but stay under clamp limit (< MAX)
    CLAMP_TILT_MAX_DEG = 30.0  # atan(5/9.81)=27° + 3° margin
    CLAMP_MIN_RESPONSE_TILT_DEG = 2.0  # must tilt at least 2° to prove response

    TAKEOFF_ALT_M = 10.0

    def __init__(self, acc_connection: str, mavsdk_address: str,
                 mode: str, log_dir: Path):
        self._acc_conn = acc_connection   # pymavlink — acc commands only
        self._mavsdk_addr = mavsdk_address  # mavsdk — arm/takeoff/land/mode
        self._mode = mode
        self._log_dir = log_dir
        self._logger: Optional[Logger] = None
        self._drone: Optional[System] = None
        self._ac: Optional[AccelerationControl] = None
        self._telem: Optional[TelemetryReceiver] = None
        # Persistent event loop — reused for setup and teardown
        self._loop = asyncio.new_event_loop()

    # ------------------------------------------------------------------
    # mavsdk helpers (async)
    # ------------------------------------------------------------------

    async def _mavsdk_setup(self):
        """Arm, takeoff to TAKEOFF_ALT_M, set mode — via mavsdk."""
        self._drone = System()
        print(f"[Setup] mavsdk connecting to {self._mavsdk_addr}...")
        await self._drone.connect(system_address=self._mavsdk_addr)

        async for state in self._drone.core.connection_state():
            if state.is_connected:
                print("[Setup] mavsdk connected")
                break

        print("[Setup] Waiting for health checks...")
        async for health in self._drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("[Setup] Health OK (GPS + EKF ready)")
                break

        print("[Setup] Arming...")
        await self._drone.action.arm()
        print("[Setup] Armed")

        print(f"[Setup] Taking off to {self.TAKEOFF_ALT_M}m...")
        await self._drone.action.set_takeoff_altitude(self.TAKEOFF_ALT_M)
        await self._drone.action.takeoff()

        async for pos in self._drone.telemetry.position():
            if pos.relative_altitude_m >= self.TAKEOFF_ALT_M * 0.9:
                print(f"[Setup] Reached {pos.relative_altitude_m:.1f}m")
                break

        print(f"[Setup] Switching to {self._mode.upper()} mode...")
        # mavsdk requires at least one manual control input before start_*_control()
        await self._drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)
        await asyncio.sleep(0.5)
        if self._mode == 'altitude':
            await self._drone.manual_control.start_altitude_control()
        else:
            await self._drone.manual_control.start_position_control()

        await asyncio.sleep(3.0)
        print(f"[Setup] {self._mode.upper()} mode active, stabilising...")

    async def _mavsdk_teardown(self):
        """Land and wait until on ground — via mavsdk."""
        if self._drone is None:
            return
        print("[Teardown] Landing...")
        await self._drone.action.land()
        async for in_air in self._drone.telemetry.in_air():
            if not in_air:
                print("[Teardown] Landed")
                break

    # ------------------------------------------------------------------
    # Infrastructure
    # ------------------------------------------------------------------

    def setup(self):
        self._logger = Logger(self._log_dir)
        print(f"\n[Setup] Log directory: {self._log_dir}")

        # mavsdk handles arm + takeoff + mode switch
        self._loop.run_until_complete(self._mavsdk_setup())

        # pymavlink for acc commands + telemetry during tests
        self._ac = AccelerationControl(self._acc_conn)
        self._ac.connect(timeout_s=10.0)
        self._telem = TelemetryReceiver(self._ac._mav)
        self._telem.start()

        print(f"[Setup] pymavlink connected, final stabilisation...")
        self._wait(2.0)

    def teardown(self):
        if self._telem:
            self._telem.stop()
        self._loop.run_until_complete(self._mavsdk_teardown())
        self._loop.close()
        if self._logger:
            self._logger.write_summary()
            self._logger.close()
            print(f"[Teardown] Logs saved to: {self._log_dir}")

    def run_all(self):
        self.setup()
        try:
            self._tc01_smoke_test()
            self._wait(2.0)
            self._tc02_clamping()
            self._wait(2.0)
            self._tc03_kinematics()
            self._wait(3.0)
            self._tc04_watchdog()
            self._wait(2.0)
            self._tc05_square()
        finally:
            self.teardown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _wait(self, seconds: float, desc: str = ''):
        """Wait while continuously logging telemetry."""
        if desc:
            print(f"  [wait {seconds:.1f}s] {desc}")
        t0 = time.time()
        while time.time() - t0 < seconds:
            s = self._telem.get_latest()
            self._logger.log_telemetry(s)
            time.sleep(0.05)

    def _send_loop(self, ax: float, ay: float, az: float,
                   duration_s: float, tc_id: str,
                   yaw_rad: Optional[float] = None,
                   rate_hz: float = 50.0) -> List[TelemetrySample]:
        """
        Send constant acceleration for duration_s at rate_hz.
        Returns list of telemetry samples captured during this period.
        """
        samples: List[TelemetrySample] = []
        self._telem.set_last_command(ax, ay, az)
        dt = 1.0 / rate_hz
        t0 = time.time()

        while time.time() - t0 < duration_s:
            self._ac.send_acceleration_ned(ax, ay, az, yaw_rad)
            self._logger.log_command(CommandRecord(
                wall_time_s = round(time.time(), 4),
                ax=ax, ay=ay, az=az,
                yaw_rad=yaw_rad if yaw_rad is not None else float('nan'),
                test_case=tc_id,
            ))
            s = self._telem.get_latest()
            self._logger.log_telemetry(s)
            samples.append(s)
            time.sleep(dt)

        return samples

    def _hover_brake(self, duration_s: float = 3.0, tc_id: str = '') -> List[TelemetrySample]:
        """Send zero acceleration to brake / return to stick control."""
        return self._send_loop(0.0, 0.0, 0.0, duration_s, tc_id)

    @staticmethod
    def _mean(values):
        v = [x for x in values if not math.isnan(x)]
        return sum(v) / len(v) if v else float('nan')

    @staticmethod
    def _max_abs(values):
        v = [abs(x) for x in values if not math.isnan(x)]
        return max(v) if v else float('nan')

    # ------------------------------------------------------------------
    # TC-01: Smoke test
    # ------------------------------------------------------------------

    def _tc01_smoke_test(self):
        TC = 'TC-01'
        print(f"\n{'─'*50}\n  {TC}: Smoke test — acc_sp_external received\n{'─'*50}")

        self._telem.clear_history()

        # Send a single non-zero command and wait one cycle
        self._send_loop(1.0, 0.0, 0.0, duration_s=0.5, tc_id=TC)
        self._hover_brake(1.0, TC)

        samples = self._telem.get_history()
        self._logger.save_tc_snapshot(TC, samples)

        # Verify: drone reacted — tilt should have changed from zero
        tilts = [s.tilt_deg for s in samples if not math.isnan(s.tilt_deg)]
        max_tilt = max(tilts) if tilts else 0.0
        # Drone should have tilted at least 1° in response to 1 m/s² command
        passed = max_tilt > 1.0
        self._logger.log_result(TestResult(
            tc_id=TC, name='Smoke test',
            passed=passed,
            reason='Tilt > 1° indicates acc_sp_external was received and applied'
                   if passed else 'No tilt detected — topic may not be received',
            metrics={
                'max_tilt_deg': round(max_tilt, 2),
                'samples_collected': len(samples),
            }
        ))

    # ------------------------------------------------------------------
    # TC-02: Clamping
    # ------------------------------------------------------------------

    def _tc02_clamping(self):
        TC = 'TC-02'
        print(f"\n{'─'*50}\n  {TC}: Clamping — oversized acc clamped to MPC_ACC_HOR\n{'─'*50}")

        self._telem.clear_history()

        # Send 3× the limit to trigger clamp
        self._send_loop(20.0, 0.0, 0.0, duration_s=2.0, tc_id=TC)
        self._hover_brake(2.0, TC)

        samples = self._telem.get_history()
        self._logger.save_tc_snapshot(TC, samples)

        tilts = [s.tilt_deg for s in samples if not math.isnan(s.tilt_deg)]
        max_tilt = max(tilts) if tilts else 0.0
        # Drone must respond (tilt > MIN) AND stay within clamp limit (tilt < MAX)
        responded = max_tilt > self.CLAMP_MIN_RESPONSE_TILT_DEG
        clamped   = max_tilt < self.CLAMP_TILT_MAX_DEG
        passed = responded and clamped
        if passed:
            reason = f'max_tilt {max_tilt:.1f}° in ({self.CLAMP_MIN_RESPONSE_TILT_DEG}°, {self.CLAMP_TILT_MAX_DEG}°) → drone responded and clamp active'
        elif not responded:
            reason = f'max_tilt {max_tilt:.1f}° ≤ {self.CLAMP_MIN_RESPONSE_TILT_DEG}° → no response to acc command'
        else:
            reason = f'max_tilt {max_tilt:.1f}° ≥ {self.CLAMP_TILT_MAX_DEG}° → clamp may not work'
        self._logger.log_result(TestResult(
            tc_id=TC, name='Acceleration clamping',
            passed=passed,
            reason=reason,
            metrics={
                'commanded_acc_m_s2': 20.0,
                'expected_clamped_acc_m_s2': 5.0,
                'max_tilt_deg': round(max_tilt, 2),
                'min_response_tilt_deg': self.CLAMP_MIN_RESPONSE_TILT_DEG,
                'clamp_threshold_deg': self.CLAMP_TILT_MAX_DEG,
            }
        ))

    # ------------------------------------------------------------------
    # TC-03: Kinematics
    # ------------------------------------------------------------------

    def _tc03_kinematics(self):
        TC = 'TC-03'
        ACC   = 2.0   # m/s²
        T     = 3.0   # seconds
        print(f"\n{'─'*50}\n  {TC}: Kinematics — ax={ACC} for {T}s\n{'─'*50}")

        self._telem.clear_history()

        # Snapshot position at start — guard NaN if telemetry not yet arrived
        s0 = self._telem.get_latest()
        x0 = s0.pos_x if not math.isnan(s0.pos_x) else None
        y0 = s0.pos_y if not math.isnan(s0.pos_y) else None

        samples = self._send_loop(ACC, 0.0, 0.0, duration_s=T, tc_id=TC)
        self._hover_brake(2.0, TC)

        all_samples = self._telem.get_history()
        self._logger.save_tc_snapshot(TC, all_samples)

        # Theoretical values at end of acceleration phase
        v_theory = ACC * T               # 6 m/s
        x_theory = 0.5 * ACC * T**2     # 9 m

        # Measured at end of command phase
        end = samples[-1] if samples else TelemetrySample()
        v_actual = end.vel_x if not math.isnan(end.vel_x) else 0.0
        if x0 is not None and not math.isnan(end.pos_x):
            x_actual = end.pos_x - x0
        else:
            x_actual = 0.0

        tol = self.KINEMATIC_TOL
        v_ok = abs(v_actual - v_theory) < tol * v_theory
        x_ok = abs(x_actual - x_theory) < tol * x_theory
        # Direction: must move North (pos_x increases)
        dir_ok = x_actual > 0.5

        passed = v_ok and x_ok and dir_ok
        self._logger.log_result(TestResult(
            tc_id=TC, name='Kinematic validation',
            passed=passed,
            reason='Position and velocity within 30% of theoretical values'
                   if passed else
                   f'velocity_ok={v_ok}, position_ok={x_ok}, direction_ok={dir_ok}',
            metrics={
                'acc_commanded_m_s2': ACC,
                'duration_s': T,
                'vel_theoretical_m_s': round(v_theory, 2),
                'vel_actual_m_s': round(v_actual, 3),
                'vel_error_pct': round(abs(v_actual - v_theory) / v_theory * 100, 1),
                'disp_theoretical_m': round(x_theory, 2),
                'disp_actual_m': round(x_actual, 3),
                'disp_error_pct': round(abs(x_actual - x_theory) / x_theory * 100, 1),
            }
        ))

    # ------------------------------------------------------------------
    # TC-04: Watchdog
    # ------------------------------------------------------------------

    def _tc04_watchdog(self):
        TC = 'TC-04'
        ACC = 2.0
        print(f"\n{'─'*50}\n  {TC}: Watchdog — stop publishing → velocity decays\n{'─'*50}")

        self._telem.clear_history()

        # Accelerate for 2s to build velocity
        self._send_loop(ACC, 0.0, 0.0, duration_s=2.0, tc_id=TC)

        vel_at_stop = self._telem.get_latest().vel_x
        print(f"  Velocity at stop: {vel_at_stop:.2f} m/s — now going silent for 1.5s")

        # STOP publishing entirely (watchdog = 500ms)
        silence_samples: List[TelemetrySample] = []
        t0 = time.time()
        while time.time() - t0 < 1.5:
            s = self._telem.get_latest()
            self._logger.log_telemetry(s)
            silence_samples.append(s)
            time.sleep(0.05)

        vel_after_silence = self._telem.get_latest().vel_x

        # Continue logging decay phase
        post_samples = self._hover_brake(3.0, TC)

        all_samples = self._telem.get_history()
        self._logger.save_tc_snapshot(TC, all_samples)

        # Pass: velocity did NOT continue growing during silence
        # (watchdog fired → position hold → velocity should not increase)
        vel_grew = (vel_after_silence - vel_at_stop) > 0.5  # allow 0.5 m/s inertia
        passed = not vel_grew

        self._logger.log_result(TestResult(
            tc_id=TC, name='Watchdog fallback',
            passed=passed,
            reason='Velocity did not increase after command silence — watchdog fired'
                   if passed else
                   f'Velocity grew {vel_after_silence - vel_at_stop:.2f} m/s during silence — watchdog may not have fired',
            metrics={
                'vel_when_publishing_stopped_m_s': round(vel_at_stop, 3),
                'vel_after_1500ms_silence_m_s': round(vel_after_silence, 3),
                'vel_delta_m_s': round(vel_after_silence - vel_at_stop, 3),
                'watchdog_timeout_ms': 500,
            }
        ))

    # ------------------------------------------------------------------
    # TC-05: Square trajectory
    # ------------------------------------------------------------------

    def _tc05_square(self):
        TC = 'TC-05'
        ACC  = 1.5    # m/s²
        PUSH = 2.0    # s — acceleration phase per leg
        BRAKE = 2.5   # s — braking phase per leg
        print(f"\n{'─'*50}\n  {TC}: Square trajectory — 4 legs, return to origin\n{'─'*50}")

        self._telem.clear_history()

        s0 = self._telem.get_latest()
        x0, y0 = s0.pos_x, s0.pos_y

        legs = [
            ('North', ACC,  0.0),
            ('East',  0.0,  ACC),
            ('South', -ACC, 0.0),
            ('West',  0.0, -ACC),
        ]

        all_leg_metrics = []
        for name, ax, ay in legs:
            print(f"  Leg: {name}  ax={ax:.1f}  ay={ay:.1f}")

            leg_s0 = self._telem.get_latest()
            push_samples = self._send_loop(ax, ay, 0.0, PUSH, TC)
            brake_samples = self._hover_brake(BRAKE, TC)

            leg_end = self._telem.get_latest()
            disp = math.sqrt(
                (leg_end.pos_x - leg_s0.pos_x)**2 +
                (leg_end.pos_y - leg_s0.pos_y)**2
            ) if not any(math.isnan(v) for v in [
                leg_end.pos_x, leg_end.pos_y, leg_s0.pos_x, leg_s0.pos_y]) else float('nan')

            all_leg_metrics.append({
                'leg': name,
                'disp_m': round(disp, 2) if not math.isnan(disp) else 'nan',
                'end_speed_m_s': round(leg_end.speed_h, 3) if not math.isnan(leg_end.speed_h) else 'nan',
            })
            print(f"    displacement={disp:.2f}m  end_speed={leg_end.speed_h:.2f}m/s")

        all_samples = self._telem.get_history()
        self._logger.save_tc_snapshot(TC, all_samples)

        # Final position vs origin
        sf = self._telem.get_latest()
        dist_from_origin = math.sqrt(
            (sf.pos_x - x0)**2 + (sf.pos_y - y0)**2
        ) if not any(math.isnan(v) for v in [sf.pos_x, sf.pos_y]) else float('nan')

        passed = (not math.isnan(dist_from_origin) and
                  dist_from_origin < self.SQUARE_RETURN_TOL)

        metrics = {
            'origin_x': round(x0, 2), 'origin_y': round(y0, 2),
            'final_x':  round(sf.pos_x, 2) if not math.isnan(sf.pos_x) else 'nan',
            'final_y':  round(sf.pos_y, 2) if not math.isnan(sf.pos_y) else 'nan',
            'dist_from_origin_m': round(dist_from_origin, 3) if not math.isnan(dist_from_origin) else 'nan',
            'tolerance_m': self.SQUARE_RETURN_TOL,
        }
        for i, leg in enumerate(all_leg_metrics):
            for k, v in leg.items():
                metrics[f'leg{i+1}_{k}'] = v

        self._logger.log_result(TestResult(
            tc_id=TC, name='Square trajectory',
            passed=passed,
            reason=f'Returned within {dist_from_origin:.2f}m of origin (tol={self.SQUARE_RETURN_TOL}m)'
                   if passed else
                   f'Final distance {dist_from_origin:.2f}m > {self.SQUARE_RETURN_TOL}m tolerance',
            metrics=metrics,
        ))


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='SITL integration tests for NED acceleration control')
    parser.add_argument('--mavsdk', default='udp://:14540',
                        help='mavsdk address for arm/takeoff/land (default: udp://:14540)')
    parser.add_argument('--connection', default='udpin:0.0.0.0:14550',
                        help='pymavlink connection for acc commands (default: udpin:0.0.0.0:14550)')
    parser.add_argument('--mode', choices=['position', 'altitude'], default='position',
                        help='Flight mode to test (default: position)')
    parser.add_argument('--log-dir', default=None,
                        help='Log output directory (default: logs/sitl_acc_test_<ts>)')
    args = parser.parse_args()

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir = Path(args.log_dir) if args.log_dir else \
              Path(__file__).parent / 'logs' / f'sitl_acc_test_{ts}'

    runner = SITLAccTest(
        acc_connection=args.connection,
        mavsdk_address=args.mavsdk,
        mode=args.mode,
        log_dir=log_dir,
    )

    print(f"""
╔══════════════════════════════════════════════════╗
║  SITL Acceleration Control Test Suite            ║
║  Mode      : {args.mode.upper():<34} ║
║  mavsdk    : {args.mavsdk:<34} ║
║  acc conn  : {args.connection:<34} ║
║  Log dir   : {str(log_dir):<34} ║
╚══════════════════════════════════════════════════╝
""")

    try:
        runner.run_all()
    except KeyboardInterrupt:
        print("\n[!] Interrupted by user")
        runner.teardown()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        runner.teardown()
        sys.exit(1)


if __name__ == '__main__':
    main()
