"""AutopilotTester for precision landing on a target estimated by the vision target estimator.

The landing pad is fixed on the ground next to home. The tester simulates the target sensors
from ground truth: TARGET_RELATIVE for a downward looking camera, and TARGET_ABSOLUTE for a GNSS
receiver on the pad. PX4 streams its estimate back as LANDING_TARGET.
"""

import json
import math
import os
import threading
import time
from enum import Enum
from typing import Any, Dict, Optional

from mavsdk.plugins.telemetry import LandedState  # type: ignore

from autopilot_tester import (
    MAV_CMD_NAV_LAND, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT,
    TEST_DIR, AutopilotTester, log, raw_mission_item)
from geometry import CoordinateTransformation

# TODO: Change development.xml to common.xml when TARGET_* are promoted.
MAVLINK_XML = os.path.join(TEST_DIR, "..", "..", "src", "modules", "mavlink", "mavlink",
                           "message_definitions", "v1.0", "development.xml")

MAV_CMD_DO_SET_MODE = 176
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND = 9
MAVLINK_MSG_ID_LANDING_TARGET = 149
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2
TARGET_OBS_FRAME_OTHER = 3
TARGET_ABSOLUTE_SENSOR_CAPABILITY_POSITION = 1

MISSION_OFFSET_NORTH_M = -3.0
TARGET_YAW_DEG = 120.0
# Host time: detects telemetry that stopped, independent of the simulation.
TELEMETRY_TIMEOUT_S = 2.0
OBSERVATION_POLL_INTERVAL_S = 0.001

MINIMUM_DISTURBANCE_HEIGHT_M = 3.0
ACQUISITION_RADIUS_M = 0.75
CORRECTED_RADIUS_M = 0.5


class Scenario(Enum):
    VISION = "vision"
    GNSS = "gnss"
    VISION_AND_GNSS = "vision_and_gnss"
    MISSION = "mission"
    GNSS_JUMP = "gnss_jump"
    VISION_LOSS = "vision_loss"


# VTE_AID_MASK: target GNSS position (bit 0), vehicle GNSS velocity (bit 1), vision (bit 2),
# mission position (bit 3). Home aiding is always disabled.
AID_MASKS = {
    Scenario.VISION: 4,
    Scenario.GNSS: 3,
    Scenario.MISSION: 14,
}
DEFAULT_AID_MASK = 7


class AutopilotTesterVisionTarget(AutopilotTester):

    def __init__(self, speed_factor: Optional[float] = None) -> None:
        super().__init__(speed_factor)
        self._target_latitude_deg = math.nan
        self._target_longitude_deg = math.nan
        self._target_altitude_m = math.nan
        self._estimate_lock = threading.Lock()
        self._estimate: Optional[Dict[str, Any]] = None

    def connect(self, url: str) -> None:
        super().connect(url)

        with open(MAVLINK_XML) as xml:
            self._mavlink_direct.load_custom_xml(xml.read())

        # Without VTE, TARGET_RELATIVE can bypass the estimator and feed precland directly.
        assert self._param.get_param_int("VTE_EN") == 1, "VTE_EN needs to be set"

    def set_landing_target(self, north_m: float, east_m: float) -> None:
        """Place the pad at an offset from home."""
        self.store_home()
        home_latitude_deg, home_longitude_deg = self._home_or_fail()
        self._target_altitude_m = self.home_absolute_altitude_m()
        assert math.isfinite(self._target_altitude_m)
        self._target_latitude_deg, self._target_longitude_deg = CoordinateTransformation(
            home_latitude_deg, home_longitude_deg).global_from_local(north_m, east_m)

    def _target_frame(self) -> CoordinateTransformation:
        return CoordinateTransformation(self._target_latitude_deg, self._target_longitude_deg)

    def _start_landing_mission(self) -> None:
        # An approximate mission location: vision must correct the landing to the true pad.
        latitude_deg, longitude_deg = self._target_frame().global_from_local(
            MISSION_OFFSET_NORTH_M, 0.0)
        x = round(latitude_deg * 1e7)
        y = round(longitude_deg * 1e7)
        waypoint = raw_mission_item(
            0, MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT, x=x, y=y,
            z=self._telemetry.position().relative_altitude_m,
            param2=1.0,  # Waypoint acceptance radius in metres.
            current=1)
        waypoint.param4 = math.nan
        land = raw_mission_item(
            1, MAV_CMD_NAV_LAND, MAV_FRAME_GLOBAL_RELATIVE_ALT, x=x, y=y, z=0.0,
            param2=2.0)  # Required precision landing.
        land.param3 = math.nan
        land.param4 = math.nan
        self.upload_mission_raw([waypoint, land])
        self._mission_raw.start_mission()

    def _on_landing_target(self, message: Any, _user_data: Any) -> None:
        fields = json.loads(message.fields_json)
        with self._estimate_lock:
            self._estimate = fields

    def precision_land(self, scenario: Scenario, timeout_s: float) -> None:
        self.set_param_int("VTE_AID_MASK", AID_MASKS.get(scenario, DEFAULT_AID_MASK))
        self._telemetry.set_rate_attitude_euler(15)

        # PX4 streams the VTE output as LANDING_TARGET, with the relative position in x, y, z.
        self._mavlink_direct.subscribe_message("LANDING_TARGET", self._on_landing_target)
        self.send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL,
                               param1=MAVLINK_MSG_ID_LANDING_TARGET,
                               param2=50000)  # 20 Hz

        if scenario == Scenario.MISSION:
            self._start_landing_mission()
        else:
            self.send_command_long(MAV_CMD_DO_SET_MODE,
                                   param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                   param2=PX4_CUSTOM_MAIN_MODE_AUTO,
                                   param3=PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND)

        # Use simulator time for flight deadlines so speedup does not change the scenario.
        start_us = int(self._telemetry.ground_truth().timestamp_us)
        assert start_us != 0, "No ground truth timestamp"
        last_sample_received = time.monotonic()
        last_sample_us = start_us
        vision_loss_us = 0
        spoofed_samples = 0
        vision_enabled = scenario != Scenario.MISSION
        mission_correction_checked = False
        yaw_checked = False
        target_frame = self._target_frame()

        while (self._telemetry.armed()
               and (last_sample_us - start_us) / 1e6 < timeout_s):
            vehicle = self._telemetry.ground_truth()

            if vehicle.timestamp_us != last_sample_us:
                assert math.isfinite(vehicle.latitude_deg)
                assert math.isfinite(vehicle.longitude_deg)
                assert math.isfinite(vehicle.absolute_altitude_m)
                north_m, east_m = target_frame.local_from_global(
                    vehicle.latitude_deg, vehicle.longitude_deg)
                distance_m = math.hypot(north_m, east_m)
                height_m = vehicle.absolute_altitude_m - self._target_altitude_m

                if scenario == Scenario.MISSION:
                    # Below PLD_SRCH_ALT (pinned in sih-sitl.json), so searching alone cannot
                    # expose vision.
                    mission_vision_height_m = 9.0

                    if not vision_enabled and height_m < mission_vision_height_m:
                        # Verify the aircraft is over the mission position before exposing the
                        # true pad.
                        mission_error_m = math.hypot(north_m - MISSION_OFFSET_NORTH_M, east_m)
                        assert mission_error_m < ACQUISITION_RADIUS_M, \
                            "Not over the mission position: error {:.2f} m".format(
                                mission_error_m)
                        assert distance_m > abs(MISSION_OFFSET_NORTH_M) - ACQUISITION_RADIUS_M, \
                            "Already over the pad before vision: {:.2f} m".format(distance_m)
                        vision_enabled = True

                    correction_check_height_m = 4.0

                    if vision_enabled and height_m < correction_check_height_m:
                        # Allow the controller time to complete the 3 m correction before
                        # checking alignment.
                        assert distance_m < CORRECTED_RADIUS_M, \
                            "Not corrected to the pad: {:.2f} m".format(distance_m)
                        mission_correction_checked = True

                vision_loss_height_m = 8.0
                minimum_bias_overlap_s = 4.0

                if (scenario == Scenario.VISION_LOSS and vision_enabled
                        and MINIMUM_DISTURBANCE_HEIGHT_M < height_m < vision_loss_height_m
                        and distance_m < CORRECTED_RADIUS_M
                        and (vehicle.timestamp_us - start_us) / 1e6 >= minimum_bias_overlap_s):
                    # Verify yaw acquisition before vision disappears and the orientation
                    # estimate expires.
                    self._check_target_yaw()
                    yaw_checked = True
                    vision_enabled = False
                    vision_loss_us = vehicle.timestamp_us
                    log("Vision lost at {:.1f} m".format(height_m))

                # Spoof only after acquisition, while descending with time left to react.
                spoof_start_height_m = 8.0
                start_spoof = (scenario == Scenario.GNSS_JUMP
                               and distance_m < ACQUISITION_RADIUS_M
                               and MINIMUM_DISTURBANCE_HEIGHT_M < height_m < spoof_start_height_m)
                spoof_gnss = start_spoof or spoofed_samples > 0
                gnss_bias_east_m = 2.0
                gnss_offset_east_m = gnss_bias_east_m if scenario == Scenario.VISION_LOSS else 0.0

                if spoof_gnss:
                    gnss_jump_east_m = 200.0
                    gnss_offset_east_m = gnss_jump_east_m

                self._send_target_observations(scenario, vehicle, vision_enabled,
                                               gnss_offset_east_m)
                yaw_check_height_m = 4.0

                if (scenario != Scenario.GNSS and vision_enabled
                        and height_m < yaw_check_height_m):
                    self._check_target_yaw()
                    yaw_checked = True

                if spoof_gnss:
                    spoofed_samples += 1
                    # Catch a transient excursion even if vision subsequently recovers the
                    # landing.
                    spoof_max_distance_m = 2.0
                    assert distance_m < spoof_max_distance_m, \
                        "Followed the GNSS jump: {:.2f} m from the pad".format(distance_m)

                if vision_loss_us != 0:
                    assert distance_m < CORRECTED_RADIUS_M, \
                        "Drifted after vision loss: {:.2f} m from the pad".format(distance_m)

                if 1.0 < height_m < 4.0:
                    self._check_estimate(vehicle.timestamp_us, north_m, east_m, height_m)

                last_sample_us = vehicle.timestamp_us
                last_sample_received = time.monotonic()

            # Do not keep publishing a stale observation if ground-truth telemetry stops.
            assert time.monotonic() - last_sample_received < TELEMETRY_TIMEOUT_S, \
                "Ground-truth telemetry stopped during precision landing"

            time.sleep(OBSERVATION_POLL_INTERVAL_S)

        assert not self._telemetry.armed(), \
            "Still armed after {} s of precision landing".format(timeout_s)

        if scenario != Scenario.GNSS:
            assert yaw_checked, "Target yaw was never checked"

        if scenario == Scenario.MISSION:
            assert mission_correction_checked, "Mission correction was never checked"

        if scenario == Scenario.GNSS_JUMP:
            minimum_spoofed_samples = 10
            assert spoofed_samples >= minimum_spoofed_samples, \
                "Only {} spoofed samples".format(spoofed_samples)

        if scenario == Scenario.VISION_LOSS:
            minimum_vision_outage_s = 8.0
            assert vision_loss_us != 0, "Vision was never lost"
            outage_s = (last_sample_us - vision_loss_us) / 1e6
            assert outage_s >= minimum_vision_outage_s, \
                "Vision outage only {:.1f} s".format(outage_s)

    def _check_estimate(self, timestamp_us: int, north_m: float, east_m: float,
                        height_m: float) -> None:
        """VTE must keep a live estimate that matches the pad in 3D, including the height."""
        with self._estimate_lock:
            estimate = self._estimate
        assert estimate is not None, "No LANDING_TARGET received"
        estimate_age_s = (timestamp_us - estimate["time_usec"]) * 1e-6
        estimate_error_m = math.hypot(estimate["x"] + north_m, estimate["y"] + east_m,
                                      estimate["z"] - height_m)
        details = "age {:.3f} s, error {:.2f} m, z {:.2f} m, height {:.2f} m".format(
            estimate_age_s, estimate_error_m, estimate["z"], height_m)
        assert estimate["position_valid"], "Target estimate not valid: " + details
        maximum_estimate_age_s = 0.25
        assert abs(estimate_age_s) < maximum_estimate_age_s, \
            "Target estimate too old: " + details
        estimate_acceptance_m = 0.5
        assert estimate_error_m < estimate_acceptance_m, \
            "Target estimate off: " + details

    def _send_target_observations(self, scenario: Scenario, vehicle: Any, vision_enabled: bool,
                                  gnss_offset_east_m: float) -> None:
        target_frame = self._target_frame()

        if scenario != Scenario.GNSS and vision_enabled:
            vehicle_north_m, vehicle_east_m = target_frame.local_from_global(
                vehicle.latitude_deg, vehicle.longitude_deg)
            target_ned_m = (-vehicle_north_m, -vehicle_east_m,
                            vehicle.absolute_altitude_m - self._target_altitude_m)
            # Fixed virtual camera looking down (yaw 90, then pitch -90 deg):
            # sensor (x, y, z) = (down, -north, -east). Independent of the estimated attitude;
            # the pitch stops q_sensor commuting with q_target.
            target_sensor_m = (target_ned_m[2], -target_ned_m[0], -target_ned_m[1])
            # q_target = conj(q_sensor) * yaw(TARGET_YAW_DEG): the pad lies flat, so
            # q_sensor * q_target is a pure yaw.
            cos_half_yaw = math.cos(math.radians(TARGET_YAW_DEG) / 2)
            sin_half_yaw = math.sin(math.radians(TARGET_YAW_DEG) / 2)
            q_target_wy = 0.5 * (cos_half_yaw + sin_half_yaw)
            q_target_xz = 0.5 * (sin_half_yaw - cos_half_yaw)
            self._send_target_message("TARGET_RELATIVE", {
                "frame": TARGET_OBS_FRAME_OTHER,
                "type": LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                "x": target_sensor_m[0], "y": target_sensor_m[1], "z": target_sensor_m[2],
                "pos_std": [0.1, 0.1, 0.1],
                "q_sensor": [0.5, 0.5, -0.5, 0.5],
                "q_target": [q_target_wy, q_target_xz, q_target_wy, q_target_xz],
                "yaw_std": 0.1,
            })

        if scenario not in (Scenario.VISION, Scenario.MISSION):
            latitude_deg, longitude_deg = target_frame.global_from_local(
                0.0, gnss_offset_east_m)
            self._send_target_message("TARGET_ABSOLUTE", {
                "sensor_capabilities": TARGET_ABSOLUTE_SENSOR_CAPABILITY_POSITION,
                "lat": round(latitude_deg * 1e7),
                "lon": round(longitude_deg * 1e7),
                "alt": self._target_altitude_m,
                "position_std": [0.1, 0.1],
            })

    def _send_target_message(self, name: str, fields: Dict[str, Any]) -> None:
        # Generate observations immediately, with UNIX timestamps as required by TARGET_*.
        # No TIMESYNC: these flight tests use PX4's unsynchronized arrival-time fallback.
        self.send_mavlink_message(name, {"timestamp": int(time.time() * 1e6), **fields})

    def _check_target_yaw(self) -> None:
        attitude = self._telemetry.attitude_euler()
        sample_age_us = abs(self._telemetry.ground_truth().timestamp_us - attitude.timestamp_us)
        maximum_attitude_age_us = 1e6
        assert sample_age_us < maximum_attitude_age_us, \
            "Attitude too old: {} us".format(sample_age_us)
        yaw_error_deg = abs(math.remainder(attitude.yaw_deg - TARGET_YAW_DEG, 360.0))
        yaw_acceptance_deg = 5.0
        assert yaw_error_deg < yaw_acceptance_deg, \
            "Yaw {:.1f} deg, error to target yaw {:.1f} deg".format(
                attitude.yaw_deg, yaw_error_deg)

    def check_landed_at_target(self, acceptance_radius_m: float) -> None:
        assert self._telemetry.landed_state() == LandedState.ON_GROUND, \
            "Landed state is {}".format(self._telemetry.landed_state())

        # Wait for a fresh simulator position after disarming before checking touchdown.
        previous_sample_us = self._telemetry.ground_truth().timestamp_us
        deadline = time.monotonic() + TELEMETRY_TIMEOUT_S
        while (self._telemetry.ground_truth().timestamp_us <= previous_sample_us
               and time.monotonic() < deadline):
            time.sleep(OBSERVATION_POLL_INTERVAL_S)

        vehicle = self._telemetry.ground_truth()
        assert vehicle.timestamp_us > previous_sample_us, "No fresh ground truth after landing"
        north_m, east_m = self._target_frame().local_from_global(
            vehicle.latitude_deg, vehicle.longitude_deg)
        landing_error_m = math.hypot(north_m, east_m)
        log("Landing error: {:.2f} m".format(landing_error_m))
        assert landing_error_m < acceptance_radius_m, \
            "Landed {:.2f} m from the pad, expected < {} m".format(
                landing_error_m, acceptance_radius_m)
