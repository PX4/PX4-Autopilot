"""GNSS failure injection and its observation over MAVLink, shared by the SIH failover tests and the
gnss_failover companion tool. It never arms or changes the mode. Injections go through
MAV_CMD_INJECT_FAILURE, with the SYS_FAIL_GPS_* parameters set first. The vehicle applies them
where each receiver's sample is published.

MAVSDK calls the handlers on its own threads, so the observed state is kept under a lock.
"""

import json
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, TextIO, Tuple

from mavsdk.plugins.events import Events  # type: ignore
from mavsdk.plugins.failure import (  # type: ignore
    Failure, FailureError, FailureResult, FailureType, FailureUnit)
from mavsdk.plugins.log_files import LogFiles, LogFilesError, LogFilesResult  # type: ignore
from mavsdk.plugins.mavlink_direct import (  # type: ignore
    MavlinkDirect, MavlinkDirectError, MavlinkMessage)
from mavsdk.plugins.param import Param, ParamError  # type: ignore
from mavsdk.plugins.telemetry import FlightMode, Telemetry  # type: ignore

# A receiver whose MAVLink message is older than this has no fix for the injection preconditions
RECEIVER_TIMEOUT_US = 2_000_000

ODOMETRY_RATE_HZ = 30.0
ESTIMATOR_STATUS_RATE_HZ = 10.0
GPS_RATE_HZ = 5.0

MAVLINK_MSG_ID_GPS_RAW_INT = 24
MAVLINK_MSG_ID_GPS2_RAW = 124
MAVLINK_MSG_ID_ESTIMATOR_STATUS = 230
MAVLINK_MSG_ID_ODOMETRY = 331
MAV_CMD_INJECT_FAILURE = 420
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_COMP_ID_AUTOPILOT1 = 1
GPS_FIX_TYPE_3D_FIX = 3

# ESTIMATOR_STATUS flags of a position good enough for position control: relative and absolute
# horizontal, absolute vertical
ESTIMATOR_POS_HORIZ_REL = 8
ESTIMATOR_POS_HORIZ_ABS = 16
ESTIMATOR_POS_VERT_ABS = 32
POSITION_OK_FLAGS = ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS

RECEIVER_MESSAGES = ("GPS_RAW_INT", "GPS2_RAW")

POSITION_CONTROLLED_MODES = (FlightMode.POSCTL, FlightMode.HOLD, FlightMode.MISSION,
                             FlightMode.RETURN_TO_LAUNCH)


@dataclass
class WrongPayload:
    """SYS_FAIL_GPS_* payload of a wrong injection. Every field but the fix type is left unchanged
    at 0."""
    fix_type: int = 2  # SYS_FAIL_GPS_WRG, 0 = unchanged
    eph: float = 0.0  # SYS_FAIL_GPS_EPH [m]
    epv: float = 0.0  # SYS_FAIL_GPS_EPV [m]
    speed_accuracy: float = 0.0  # SYS_FAIL_GPS_SAC [m/s]
    satellites: int = 0  # SYS_FAIL_GPS_SAT
    jamming_state: int = 0  # SYS_FAIL_GPS_JAM
    spoofing_state: int = 0  # SYS_FAIL_GPS_SPF


@dataclass
class Injection:
    failure_type: Any = FailureType.OFF
    instance: int = 1  # receiver, 1-based (sensor_gnss instance + 1), 0 = all
    wrong: WrongPayload = field(default_factory=WrongPayload)
    slow_divider: int = 4  # SYS_FAIL_GPS_DIV

    def __str__(self) -> str:
        text = "gps {} instance {}".format(type_name(self.failure_type), self.instance)
        if self.failure_type == FailureType.WRONG:
            wrong = self.wrong
            text += (" (fix {}, eph {:g}, epv {:g}, sacc {:g}, sats {}, jamming {}, spoofing {})"
                     .format(wrong.fix_type, wrong.eph, wrong.epv, wrong.speed_accuracy,
                             wrong.satellites, wrong.jamming_state, wrong.spoofing_state))
        elif self.failure_type == FailureType.SLOW:
            text += " (one sample in {})".format(self.slow_divider)
        return text


@dataclass
class Reset:
    """An ODOMETRY sample whose reset counter changed."""
    vehicle_time_us: int = 0
    counter_increase: int = 0
    position_jump: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # [m] NED, motion removed
    yaw_jump: float = 0.0  # [rad]


@dataclass
class Receiver:
    """GPS_RAW_INT (index 0) or GPS2_RAW (index 1): the receivers MAVLink keeps for the session."""
    vehicle_time_us: int = 0  # last message, 0 before the first one
    fix_type: int = 0  # GPS_FIX_TYPE
    satellites: int = 0
    eph: float = 0.0  # [m] 0 when unknown


@dataclass
class PositionChange:
    vehicle_time_us: int
    position_ok: bool


@dataclass
class ModeChange:
    vehicle_time_us: int
    mode: Any


@dataclass
class EventRecord:
    vehicle_time_us: int
    name: str  # namespace/name
    message: str


@dataclass
class _OdometrySample:
    time_us: int = 0
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    reset_counter: int = 0


def type_name(failure_type: Any) -> str:
    return str(FailureType(failure_type).name).lower()


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GnssFailover:

    def __init__(self, system: Any, events: Any = None) -> None:
        """events: the caller's Events plugin to subscribe to, None to create one."""
        self._system = system
        self._failure = Failure(system)
        self._param = Param(system)
        self._telemetry = Telemetry(system)
        self._mavlink_direct = MavlinkDirect(system)
        self._log_files = LogFiles(system)
        self._events = events if events is not None else Events(system)

        self._lock = threading.Lock()
        self._odometry: List[_OdometrySample] = []  # newest first, up to 2
        self._reset_count = 0
        self._resets: List[Reset] = []
        self._receivers = [Receiver(), Receiver()]
        self._position_changes: List[PositionChange] = []
        self._mode_changes: List[ModeChange] = []
        self._event_records: List[EventRecord] = []

        # SYS_FAIL_GPS_* values before the first inject() that changed them
        self._saved_int_params: Dict[str, int] = {}
        self._saved_float_params: Dict[str, float] = {}

        self._record: Optional[TextIO] = None
        self._record_lock = threading.Lock()

        self._mavlink_direct.subscribe_message("ODOMETRY", self._on_odometry)
        self._mavlink_direct.subscribe_message("GPS_RAW_INT", self._on_gps_raw)
        self._mavlink_direct.subscribe_message("GPS2_RAW", self._on_gps_raw)
        self._mavlink_direct.subscribe_message("ESTIMATOR_STATUS", self._on_estimator_status)
        self._telemetry.subscribe_flight_mode(self._on_flight_mode)
        self._events.subscribe_events(self._on_event)

    # ------------------------------------------------------------------
    # Streams and handlers
    # ------------------------------------------------------------------

    def _target_system_id(self) -> int:
        return int(self._system.get_system_id())

    def _command_long_fields(self, command: int, params: List[Optional[float]]) -> str:
        # MAVSDK takes null as NaN for float fields
        fields: Dict[str, Any] = {
            "target_system": self._target_system_id(),
            "target_component": MAV_COMP_ID_AUTOPILOT1,
            "command": command, "confirmation": 0,
        }
        for i, value in enumerate(params):
            fields["param{}".format(i + 1)] = value
        return json.dumps(fields)

    def _set_message_interval(self, message_id: int, rate_hz: float) -> None:
        # request_streams() checks the outcome by waiting for the messages
        try:
            self._mavlink_direct.send_command(MavlinkMessage(
                message_name="COMMAND_LONG", fields_json=self._command_long_fields(
                    MAV_CMD_SET_MESSAGE_INTERVAL,
                    [float(message_id), 1e6 / rate_hz, None, None, None, None, 0.0])))
        except MavlinkDirectError:
            pass

    def request_streams(self) -> bool:
        """Streams ODOMETRY, ESTIMATOR_STATUS, GPS_RAW_INT and GPS2_RAW at the rates the observation
        needs. False if no ODOMETRY arrives."""
        self._set_message_interval(MAVLINK_MSG_ID_ODOMETRY, ODOMETRY_RATE_HZ)
        self._set_message_interval(MAVLINK_MSG_ID_ESTIMATOR_STATUS, ESTIMATOR_STATUS_RATE_HZ)
        self._set_message_interval(MAVLINK_MSG_ID_GPS_RAW_INT, GPS_RATE_HZ)
        self._set_message_interval(MAVLINK_MSG_ID_GPS2_RAW, GPS_RATE_HZ)
        return self.wait_until(lambda: self.vehicle_time_us() > 0, 10.0)

    def _on_odometry(self, message: Any, _user_data: Any) -> None:
        odometry = json.loads(message.fields_json)
        q = odometry["q"]
        sample = _OdometrySample(
            time_us=int(odometry["time_usec"]),
            position=(float(odometry["x"]), float(odometry["y"]), float(odometry["z"])),
            yaw=math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                           1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])),
            reset_counter=int(odometry["reset_counter"]))

        reset_text = ""
        with self._lock:
            if self._odometry and sample.time_us < self._odometry[0].time_us:
                # The vehicle rebooted: start over
                self._odometry = []

            if self._odometry and sample.reset_counter != self._odometry[0].reset_counter:
                previous = self._odometry[0]
                before = self._odometry[1] if len(self._odometry) > 1 else None

                # The motion between the two samples before the reset stands in for the motion
                # across it
                jump = tuple(
                    sample.position[i] - previous.position[i]
                    - ((previous.position[i] - before.position[i]) if before else 0.0)
                    for i in range(3))
                turn = wrap_pi(previous.yaw - before.yaw) if before else 0.0
                reset = Reset(
                    vehicle_time_us=sample.time_us,
                    counter_increase=(sample.reset_counter - previous.reset_counter) % 256,
                    position_jump=(jump[0], jump[1], jump[2]),
                    yaw_jump=wrap_pi(sample.yaw - previous.yaw - turn))

                self._reset_count += reset.counter_increase
                self._resets.append(reset)
                reset_text = ("estimator reset +{} position jump N {:g} E {:g} D {:g} yaw jump {:g}"
                              .format(reset.counter_increase, *reset.position_jump, reset.yaw_jump))

            self._odometry = [sample] + self._odometry[:1]

        if reset_text:
            self.record(reset_text)

    def _on_estimator_status(self, message: Any, _user_data: Any) -> None:
        flags = int(json.loads(message.fields_json)["flags"])
        position_ok = (flags & POSITION_OK_FLAGS) == POSITION_OK_FLAGS

        with self._lock:
            changed = (not self._position_changes
                       or self._position_changes[-1].position_ok != position_ok)
            if changed:
                self._position_changes.append(
                    PositionChange(self._vehicle_time_us_locked(), position_ok))

        if changed:
            self.record("{}, estimator flags 0x{:x}".format(
                "position valid" if position_ok else "position not valid", flags))

    def _on_gps_raw(self, message: Any, _user_data: Any) -> None:
        gps = json.loads(message.fields_json)
        h_acc = int(gps.get("h_acc") or 0)

        with self._lock:
            receiver = self._receivers[RECEIVER_MESSAGES.index(message.message_name)]
            receiver.vehicle_time_us = self._vehicle_time_us_locked()
            receiver.fix_type = int(gps["fix_type"])
            receiver.satellites = int(gps["satellites_visible"])
            receiver.eph = 1e-3 * h_acc if h_acc > 0 else 0.0

    def _on_flight_mode(self, mode: Any, _user_data: Any) -> None:
        with self._lock:
            changed = not self._mode_changes or self._mode_changes[-1].mode != mode
            if changed:
                self._mode_changes.append(ModeChange(self._vehicle_time_us_locked(), mode))

        if changed:
            self.record("mode {}".format(FlightMode(mode).name))

    def _on_event(self, event: Any, _user_data: Any) -> None:
        name = "{}/{}".format(event.event_namespace, event.event_name)
        with self._lock:
            self._event_records.append(
                EventRecord(self._vehicle_time_us_locked(), name, str(event.message)))

        self.record("event {}: {}".format(name, event.message))

    # ------------------------------------------------------------------
    # Injection
    # ------------------------------------------------------------------

    def injection_refusal(self, require_flying: bool) -> str:
        """Empty when the companion may inject, otherwise why not: SYS_FAILURE_EN set, both
        receivers report a 3D fix, and, if require_flying, the vehicle is armed, in the air and in
        a position-controlled mode."""
        try:
            enabled = int(self._param.get_param_int("SYS_FAILURE_EN"))
        except ParamError:
            enabled = 0

        if enabled != 1:
            return "SYS_FAILURE_EN is not set"

        now = self.vehicle_time_us()

        for i, name in enumerate(RECEIVER_MESSAGES):
            gps = self.receiver(i)

            if gps.vehicle_time_us == 0 or now > gps.vehicle_time_us + RECEIVER_TIMEOUT_US:
                return "{} receiver is silent".format(name)

            if gps.fix_type < GPS_FIX_TYPE_3D_FIX:
                return "{} receiver has no 3D fix".format(name)

        if require_flying:
            if not self._telemetry.armed():
                return "not armed"

            if not self._telemetry.in_air():
                return "not in the air"

            mode = self._telemetry.flight_mode()

            if mode not in POSITION_CONTROLLED_MODES:
                return "mode {} is not position controlled".format(FlightMode(mode).name)

        return ""

    # Only the thread that injects touches the saved parameters

    def _set_param_int(self, name: str, value: int) -> bool:
        try:
            if name not in self._saved_int_params:
                self._saved_int_params[name] = int(self._param.get_param_int(name))
            self._param.set_param_int(name, value)
            return True
        except ParamError:
            return False

    def _set_param_float(self, name: str, value: float) -> bool:
        try:
            if name not in self._saved_float_params:
                self._saved_float_params[name] = float(self._param.get_param_float(name))
            self._param.set_param_float(name, value)
            return True
        except ParamError:
            return False

    def inject(self, injection: Injection) -> bool:
        """Sets the payload parameters, then injects. The parameters are restored by
        restore_parameters()."""
        parameters_set = True

        if injection.failure_type == FailureType.WRONG:
            wrong = injection.wrong
            parameters_set = (self._set_param_int("SYS_FAIL_GPS_WRG", wrong.fix_type)
                              and self._set_param_float("SYS_FAIL_GPS_EPH", wrong.eph)
                              and self._set_param_float("SYS_FAIL_GPS_EPV", wrong.epv)
                              and self._set_param_float("SYS_FAIL_GPS_SAC", wrong.speed_accuracy)
                              and self._set_param_int("SYS_FAIL_GPS_SAT", wrong.satellites)
                              and self._set_param_int("SYS_FAIL_GPS_JAM", wrong.jamming_state)
                              and self._set_param_int("SYS_FAIL_GPS_SPF", wrong.spoofing_state))

        elif injection.failure_type == FailureType.SLOW:
            parameters_set = self._set_param_int("SYS_FAIL_GPS_DIV", injection.slow_divider)

        if not parameters_set:
            self.record("{}: setting SYS_FAIL_GPS_* failed".format(injection))
            return False

        try:
            result = self._failure.inject(FailureUnit.SENSOR_GPS, injection.failure_type,
                                          injection.instance)
        except FailureError as e:
            result = e.result

        self.record("{}: {}".format(injection, FailureResult(result).name))
        return bool(result == FailureResult.SUCCESS)

    def inject_without_ack(self, failure_type: Any, instance: int) -> None:
        """Sends an off or ok injection without waiting for the acknowledgement, for timing that
        the round trip would distort. The payload parameters aren't set."""
        self._mavlink_direct.send_message(MavlinkMessage(
            message_name="COMMAND_LONG", fields_json=self._command_long_fields(
                MAV_CMD_INJECT_FAILURE,
                [float(FailureUnit.SENSOR_GPS), float(failure_type), float(instance),
                 None, None, None, None])))
        self.record("{}: sent".format(Injection(failure_type=failure_type, instance=instance)))

    def clear(self, instance: int = 0) -> bool:
        """Ends every GNSS injection of the receiver (0 = all receivers)."""
        return self.inject(Injection(failure_type=FailureType.OK, instance=instance))

    def restore_parameters(self) -> bool:
        """Puts back the SYS_FAIL_GPS_* values that inject() changed."""
        int_params, self._saved_int_params = self._saved_int_params, {}
        float_params, self._saved_float_params = self._saved_float_params, {}

        restored = True

        for name, value in int_params.items():
            try:
                self._param.set_param_int(name, value)
            except ParamError:
                restored = False

        for name, float_value in float_params.items():
            try:
                self._param.set_param_float(name, float_value)
            except ParamError:
                restored = False

        return restored

    # ------------------------------------------------------------------
    # Time
    # ------------------------------------------------------------------

    def _vehicle_time_us_locked(self) -> int:
        return self._odometry[0].time_us if self._odometry else 0

    def vehicle_time_us(self) -> int:
        """Vehicle time of the latest ODOMETRY sample, 0 before the first one."""
        with self._lock:
            return self._vehicle_time_us_locked()

    def wait_until(self, condition: Callable[[], bool], timeout_s: float) -> bool:
        """Waits in vehicle time, so that it follows the simulation speed. False on timeout."""
        timeout_us = int(timeout_s * 1e6)
        vehicle_start_us = self.vehicle_time_us()
        host_start = time.monotonic()

        while not condition():
            time.sleep(0.005)
            vehicle_now_us = self.vehicle_time_us()

            if vehicle_start_us > 0 and vehicle_now_us > 0:
                if vehicle_now_us - vehicle_start_us > timeout_us:
                    return False

            elif time.monotonic() - host_start > timeout_s:
                return False

        return True

    def sleep(self, seconds: float) -> None:
        self.wait_until(lambda: False, seconds)

    # ------------------------------------------------------------------
    # Observations
    # ------------------------------------------------------------------

    def reset_count(self) -> int:
        """Sum of the ODOMETRY reset counter increases since the first sample: position, velocity
        and attitude resets."""
        with self._lock:
            return self._reset_count

    def resets(self) -> List[Reset]:
        with self._lock:
            return list(self._resets)

    def receiver(self, index: int) -> Receiver:
        with self._lock:
            if 0 <= index < len(self._receivers):
                receiver = self._receivers[index]
                return Receiver(receiver.vehicle_time_us, receiver.fix_type, receiver.satellites,
                                receiver.eph)
            return Receiver()

    def position_changes(self) -> List[PositionChange]:
        with self._lock:
            return list(self._position_changes)

    def mode_changes(self) -> List[ModeChange]:
        with self._lock:
            return list(self._mode_changes)

    def events(self) -> List[EventRecord]:
        with self._lock:
            return list(self._event_records)

    def position_ok(self) -> bool:
        """The estimator reports a valid position now."""
        with self._lock:
            return bool(self._position_changes) and self._position_changes[-1].position_ok

    def position_ok_since(self, since_us: int) -> bool:
        """The estimator reported a valid position throughout [since_us, now]."""
        with self._lock:
            known = False
            ok = True

            for change in self._position_changes:
                known = True
                if change.vehicle_time_us <= since_us:
                    # the state at since_us
                    ok = change.position_ok
                else:
                    ok = ok and change.position_ok

            return known and ok

    def modes_since(self, since_us: int) -> List[Any]:
        """Modes the vehicle was in during [since_us, now]."""
        with self._lock:
            modes: List[Any] = []

            for change in self._mode_changes:
                if change.vehicle_time_us <= since_us:
                    modes = [change.mode]
                else:
                    modes.append(change.mode)

            return modes

    # ------------------------------------------------------------------
    # Log and record
    # ------------------------------------------------------------------

    def download_last_log(self, path: str, timeout_s: float) -> bool:
        """Downloads the newest log on the vehicle. False on failure or after timeout_s of host
        time."""
        try:
            entries = self._log_files.get_entries()
        except LogFilesError:
            entries = []

        if not entries:
            self.record("log download: no log entries")
            return False

        # Log IDs follow the logs' order, so the highest is the newest
        newest = max(entries, key=lambda entry: entry.id)

        results: List[Any] = []
        done = threading.Event()

        def on_progress(result: Any, _progress: Any, _user_data: Any) -> None:
            if result != LogFilesResult.NEXT and not done.is_set():
                results.append(result)
                done.set()

        self._log_files.download_log_file_async(newest, path, on_progress)
        result = results[0] if done.wait(timeout_s) else LogFilesResult.TIMEOUT

        self.record("log download of {} ({} bytes): {}".format(
            newest.date, newest.size_bytes, LogFilesResult(result).name))
        return bool(result == LogFilesResult.SUCCESS)

    def set_record(self, record: Optional[TextIO]) -> None:
        """Every injection, acknowledgement and observed change, one CSV line each: wall time [us],
        vehicle time [us], what."""
        self._record = record

    def record(self, what: str) -> None:
        wall_us = time.time_ns() // 1000
        vehicle_us = self.vehicle_time_us()

        with self._record_lock:
            if self._record is not None:
                self._record.write('{},{},"{}"\n'.format(wall_us, vehicle_us,
                                                         what.replace('"', "'")))
                self._record.flush()
