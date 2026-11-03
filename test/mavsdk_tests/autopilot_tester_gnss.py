"""AutopilotTester for SIH failover between two simulated GNSS receivers. The receivers differ by
their SIM_GNSSn_BIAS_N/E/D, so a switch shows as an estimator reset by the difference of the biases.

Checks that should not end the case, so that it still lands and grades its log, go through
check(). Their failures fail the case when the tester is closed.
"""

import math
import os
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, List, Optional, Tuple

from mavsdk.plugins.failure import FailureType  # type: ignore
from mavsdk.plugins.mission import MissionError  # type: ignore
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import TEST_DIR, AutopilotTester, MissionOptions, log
from gnss_failover_scenario import GnssFailover, Injection

# The reset moves the estimate from one receiver's error to the other's. What is left is the
# receivers' noise and the estimate's own error against the old receiver.
RESET_JUMP_TOLERANCE_M = 0.5

# A jump smaller than this isn't a reset of that axis
NO_JUMP_M = 0.3

NO_YAW_JUMP_RAD = 0.035  # 2 deg

LOG_DOWNLOAD_TIMEOUT_S = 120.0

# The logger closes the file shortly after the disarm. A size that holds this long means it is done.
LOG_CLOSED_STABLE_TIME_S = 2.0
LOG_CLOSED_TIMEOUT_S = 60.0

ULOG_MAGIC = b"ULog\x01\x12\x35"

REPORT = os.path.normpath(os.path.join(TEST_DIR, "..", "..", "Tools", "gnss_failover_report.py"))

Vec3 = Tuple[float, float, float]


class LogSource(Enum):
    ROOTFS = 0  # read from the SITL rootfs (--px4-rootfs) when given, else downloaded
    DOWNLOAD = 1  # downloaded over MAVLink, as the gnss_failover companion tool does


@dataclass
class _Mark:
    reset_count: int = 0
    reset_index: int = 0
    vehicle_time_us: int = 0
    ground_truth: Any = None


def ulog_start_time_us(path: str) -> Optional[int]:
    """ULog header: 7 magic bytes, a version byte, then the log start time [us] since boot."""
    try:
        with open(path, "rb") as f:
            header = f.read(16)
    except OSError:
        return None

    if len(header) < 16 or header[:7] != ULOG_MAGIC:
        return None

    return int(struct.unpack_from("<Q", header, 8)[0])


class AutopilotTesterGnss(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None, px4_rootfs: str = "",
                 download_dir: str = "") -> None:
        """px4_rootfs: working directory of a PX4 SITL on this host, where the logs are read
        instead of downloaded. Empty when PX4 doesn't run on this host. download_dir: where
        downloaded logs go."""
        super().__init__(speed_factor)
        self._px4_rootfs = px4_rootfs
        self._download_dir = download_dir
        self._gnss: Optional[GnssFailover] = None
        self._check_failures: List[str] = []

        # Logs written before connect() belong to another test case
        self._connected_at = 0.0
        self._first_mark_us = 0
        self._mark = _Mark()

    def connect(self, url: str) -> None:
        self._connected_at = time.time()
        super().connect(url)
        self._gnss = GnssFailover(self._system, self._events)
        self._gnss.set_record(sys.stdout)
        assert self._gnss.request_streams(), "No ODOMETRY from the vehicle"
        self.request_ground_truth()

    def close(self) -> List[str]:
        return self._check_failures + super().close()

    def check(self, ok: bool, message: str) -> None:
        """A check that lets the case go on. It fails the case when the tester is closed."""
        if not ok:
            log("Check failed: {}".format(message))
            self._check_failures.append(message)

    def gnss(self) -> GnssFailover:
        assert self._gnss is not None, "connect() was not called"
        return self._gnss

    # ------------------------------------------------------------------
    # Setup and flight
    # ------------------------------------------------------------------

    def set_preferred_receiver(self, instance: int) -> None:
        """SENS_GNSS_PRIME: preferred receiver, 0-based, -1 for none."""
        self.set_param_int("SENS_GNSS_PRIME", instance)

    def takeoff_and_hold(self, altitude_m: float) -> None:
        self.set_takeoff_altitude(altitude_m)
        self.sleep_for(1)  # for the takeoff altitude to apply
        self.arm()
        self.takeoff()
        self.wait_until_hovering()
        self.wait_until_altitude(altitude_m, 30)

        assert self.gnss().wait_until(lambda: self.flight_mode() == FlightMode.HOLD, 30.0), \
            "Hold not reached within 30 s"

        # The in-flight magnetic yaw alignment resets the estimator shortly after takeoff
        self.sleep_for(5)

    def flight_mode(self) -> Any:
        return self._telemetry.flight_mode()

    def start_mission_leg(self, leg_length_m: float, altitude_m: float) -> None:
        self.prepare_straight_mission(MissionOptions(
            leg_length_m=leg_length_m, relative_altitude_m=altitude_m, rtl_at_end=False))

        def started() -> bool:
            try:
                self._mission.start_mission()
                return True
            except MissionError:
                return False

        assert self.gnss().wait_until(started, 3.0), "Could not start mission"

    # ------------------------------------------------------------------
    # Injection
    # ------------------------------------------------------------------

    def mark(self) -> None:
        """Starts a check window: the estimator reset count, the ground truth position, the mode
        and the time."""
        gnss = self.gnss()
        self._mark = _Mark(
            reset_count=gnss.reset_count(),
            reset_index=len(gnss.resets()),
            vehicle_time_us=gnss.vehicle_time_us(),
            ground_truth=self._telemetry.ground_truth())

        if self._first_mark_us == 0:
            self._first_mark_us = self._mark.vehicle_time_us

        assert math.isfinite(self._mark.ground_truth.latitude_deg), "No ground truth position"
        gnss.record("mark")

    def inject(self, injection: Injection) -> None:
        log("Injecting {}".format(injection))
        self.check(self.gnss().inject(injection), "Injecting {} failed".format(injection))

    def inject_off(self, instance: int) -> None:
        self.inject(Injection(failure_type=FailureType.OFF, instance=instance))

    def inject_without_ack(self, failure_type: Any, instance: int) -> None:
        self.gnss().inject_without_ack(failure_type, instance)

    def clear(self, instance: int = 0) -> None:
        log("Clearing GNSS injection, instance {}".format(instance))
        self.check(self.gnss().clear(instance),
                   "Clearing the GNSS injection of instance {} failed".format(instance))
        self.check(self.gnss().restore_parameters(), "Restoring SYS_FAIL_GPS_* failed")

    # ------------------------------------------------------------------
    # Checks since mark()
    # ------------------------------------------------------------------

    def resets_since_mark(self) -> int:
        """Reset counter increase since mark()."""
        return self.gnss().reset_count() - self._mark.reset_count

    def check_resets_since_mark(self, count: int) -> None:
        resets = self.resets_since_mark()
        self.check(resets == count,
                   "{} estimator resets since the mark, expected {}".format(resets, count))

    def wait_for_resets(self, count: int, timeout_s: float) -> None:
        """Waits until the estimator reset counter rose by at least count since mark()."""
        reset = self.gnss().wait_until(lambda: self.resets_since_mark() >= count, timeout_s)

        if not reset:
            log("Expected {} estimator resets, saw {}".format(count, self.resets_since_mark()))

        assert reset, "No {} estimator resets within {} s".format(count, timeout_s)

    def _receiver_bias(self, instance: int) -> Vec3:
        bias = [self.get_param_float("SIM_GNSS{}_BIAS_{}".format(instance, axis))
                for axis in "NED"]
        return bias[0], bias[1], bias[2]

    def check_switch_reset(self, from_receiver: int, to_receiver: int, vertical: bool) -> None:
        """The resets since mark() moved the position by the bias of to_receiver minus the bias of
        from_receiver (0-based). Vertically only when GNSS is the height reference."""
        resets = self.gnss().resets()
        assert len(resets) > self._mark.reset_index, "No estimator reset since the mark"

        # Horizontal and vertical resets can land in different estimator outputs
        jump = [sum(reset.position_jump[axis] for reset in resets[self._mark.reset_index:])
                for axis in range(3)]

        bias_from = self._receiver_bias(from_receiver)
        bias_to = self._receiver_bias(to_receiver)
        offset = [bias_to[axis] - bias_from[axis] for axis in range(3)]

        log("Reset jump N {:.3f} E {:.3f} D {:.3f}, receiver offset N {:.3f} E {:.3f} D {:.3f}"
            .format(*jump, *offset))

        horizontal_error = math.hypot(jump[0] - offset[0], jump[1] - offset[1])
        self.check(horizontal_error < RESET_JUMP_TOLERANCE_M,
                   "Horizontal reset {:.2f} m off the receiver offset".format(horizontal_error))

        if vertical:
            self.check(abs(jump[2] - offset[2]) < RESET_JUMP_TOLERANCE_M,
                       "Vertical reset {:.2f} m, receiver offset {:.2f} m".format(
                           jump[2], offset[2]))
        else:
            self.check(abs(jump[2]) < NO_JUMP_M, "Vertical jump {:.2f} m".format(jump[2]))

    def check_held_horizontally(self, tolerance_m: float) -> None:
        """The vehicle stayed where it was at mark(), by SIH ground truth."""
        mark = self._mark.ground_truth
        distance_m = self._ground_truth_horizontal_distance_to(mark.latitude_deg,
                                                               mark.longitude_deg)
        self.check(distance_m < tolerance_m,
                   "Moved {:.2f} m horizontally, expected < {} m".format(distance_m, tolerance_m))

    def check_held_vertically(self, tolerance_m: float) -> None:
        climb_m = (self._telemetry.ground_truth().absolute_altitude_m
                   - self._mark.ground_truth.absolute_altitude_m)

        if abs(climb_m) >= tolerance_m:
            log("Altitude changed by {:.2f} m".format(climb_m))

        self.check(abs(climb_m) < tolerance_m,
                   "Altitude changed by {:.2f} m, expected < {} m".format(climb_m, tolerance_m))

    def check_position_and_mode_kept(self, mode: Any) -> None:
        """Since mark(): position control stayed available and the vehicle stayed in this mode."""
        self.check(self.gnss().position_ok_since(self._mark.vehicle_time_us),
                   "Position not valid throughout since the mark")

        for visited in self.gnss().modes_since(self._mark.vehicle_time_us):
            self.check(visited == mode, "Mode {} since the mark, expected {}".format(
                FlightMode(visited).name, FlightMode(mode).name))

    def check_no_yaw_reset_since_mark(self) -> None:
        for reset in self.gnss().resets()[self._mark.reset_index:]:
            self.check(abs(reset.yaw_jump) < NO_YAW_JUMP_RAD,
                       "Yaw reset by {:.3f} rad".format(reset.yaw_jump))

    def wait_for_position_ok(self, ok: bool, timeout_s: float) -> None:
        assert self.gnss().wait_until(lambda: self.gnss().position_ok() == ok, timeout_s), \
            "Position {} within {} s".format("not valid" if ok else "still valid", timeout_s)

    def wait_for_mode_other_than(self, mode: Any, timeout_s: float) -> None:
        assert self.gnss().wait_until(lambda: self.flight_mode() != mode, timeout_s), \
            "Still in {} after {} s".format(FlightMode(mode).name, timeout_s)

    def check_switch_events(self, count: int) -> None:
        """Receiver switch events (gnss_receiver_switched) since mark()."""
        switches = 0

        for event in self.gnss().events():
            if (event.vehicle_time_us >= self._mark.vehicle_time_us
                    and event.name == "px4/gnss_receiver_switched"):
                # The event names the new receiver and the reason
                self.check(bool(event.message), "Switch event without a message")
                switches += 1

        self.check(switches == count,
                   "{} receiver switch events since the mark, expected {}".format(switches, count))

    # ------------------------------------------------------------------
    # Log
    # ------------------------------------------------------------------

    def _rootfs_flight_log(self) -> str:
        """The closed log in the SITL rootfs that covers the first mark(), empty if there is
        none."""
        # A case that arms again starts another log, so the newest file isn't necessarily the
        # flight under test: take the latest log that started before the first mark
        best = ""
        best_start_us = 0

        for directory, _subdirectories, files in os.walk(self._px4_rootfs):
            for name in files:
                path = os.path.join(directory, name)
                if not name.endswith(".ulg"):
                    continue
                try:
                    if os.path.getmtime(path) < self._connected_at:
                        continue
                except OSError:
                    continue
                start_us = ulog_start_time_us(path)
                if start_us is None:
                    continue
                if self._first_mark_us != 0 and start_us > self._first_mark_us:
                    continue
                if not best or start_us > best_start_us:
                    best = path
                    best_start_us = start_us

        if not best:
            log("No log of this flight in {}".format(self._px4_rootfs))
            return ""

        # The logger closes the file after the disarm. Until then its size keeps changing.
        deadline = time.monotonic() + LOG_CLOSED_TIMEOUT_S
        size = os.path.getsize(best)
        stable_since = time.monotonic()

        while time.monotonic() - stable_since < LOG_CLOSED_STABLE_TIME_S:
            if time.monotonic() > deadline:
                log("Log {} still grows {:.0f} s after landing".format(best, LOG_CLOSED_TIMEOUT_S))
                return ""

            time.sleep(0.25)
            current = os.path.getsize(best)

            if current != size:
                size = current
                stable_since = time.monotonic()

        return best

    def check_log(self, checks: str, source: LogSource = LogSource.ROOTFS) -> None:
        """Grades these check groups of Tools/gnss_failover_report.py on the log of the flight
        under test, after landing."""
        assert os.path.isfile(REPORT), "{} not found".format(REPORT)

        if source == LogSource.ROOTFS and self._px4_rootfs:
            path = self._rootfs_flight_log()
            assert path, "No closed log of this flight in {}".format(self._px4_rootfs)

        else:
            path = os.path.join(self._download_dir, "gnss_failover.ulg")
            assert self.gnss().download_last_log(path, LOG_DOWNLOAD_TIMEOUT_S), \
                "Log download failed"

        command = [sys.executable, REPORT, "--checks", checks, path]
        log(" ".join(command))
        sys.stdout.flush()
        returncode = subprocess.call(command)
        self.check(returncode == 0, "{} failed on {}".format(os.path.basename(REPORT), path))
