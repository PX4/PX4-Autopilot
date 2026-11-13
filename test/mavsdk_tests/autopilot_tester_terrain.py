"""AutopilotTester for flight over SIH terrain.

The checks use only the sensor, the estimator and the ground truth, so the tests do not depend on
the terrain library.
"""

import math
import threading
from typing import Any, List, NamedTuple

from mavsdk.plugins.action import OrbitYawBehavior  # type: ignore

from autopilot_tester import AutopilotTester, log

ORBIT_SPEED_M_S = 0.6           # under 0.7 x EKF2_RNG_A_VMAX, where the range aid engages
RANGE_AID_HEIGHT_M = 5.0        # EKF2_RNG_A_HMAX default

# The ground has to show through the reading as both a rise and a dip, read at the 2nd and 98th
# percentiles of the samples so the sensor's rare faults, under 1 percent of readings, stay out.
MIN_TERRAIN_SEEN_M = 2.0
TERRAIN_PERCENTILE = 0.02
# The ride on the ground is partial with a faulty sensor, each fault drops EKF2's conditional range
# aid and the aid only re-engages once the ground is within 0.7 times EKF2_RNG_A_HMAX again.
MIN_RIDE_M = 1.0
# The fused clearance follows the raw range, the estimator's own noise and lag stay well under this.
CLEARANCE_TOLERANCE_M = 0.5


class RangeSample(NamedTuple):
    range_m: float
    flat_plane_m: float         # true altitude over the cosine of tilt, the flat reading
    bottom_clearance_m: float
    true_altitude_m: float
    est_altitude_m: float       # EKF2's relative altitude, follows the ground under range aid
    cos_tilt: float             # at the moment of the sample, scales the flat plane reading


class AutopilotTesterTerrain(AutopilotTester):

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._lock = threading.Lock()
        self._sampling = False
        self._cos_tilt = 1.0
        self._true_altitude_m = math.nan
        self._bottom_clearance_m = math.nan
        self._est_altitude_m = math.nan
        self._samples: List[RangeSample] = []

    def start_sampling(self) -> None:
        # store_home() keeps the takeoff altitude, home is reset to the landing spot on touchdown.
        home_altitude_m = self.home_absolute_altitude_m()
        self._telemetry.set_rate_distance_sensor(20)
        self._telemetry.set_rate_attitude_euler(20)
        self._telemetry.set_rate_altitude(20)

        def on_attitude(attitude: Any, _user_data: Any) -> None:
            with self._lock:
                self._cos_tilt = (math.cos(math.radians(attitude.roll_deg))
                                  * math.cos(math.radians(attitude.pitch_deg)))

        # The true altitude, since the estimated one follows the ground once range aid engages.
        def on_ground_truth(ground_truth: Any, _user_data: Any) -> None:
            with self._lock:
                self._true_altitude_m = ground_truth.absolute_altitude_m - home_altitude_m

        def on_altitude(altitude: Any, _user_data: Any) -> None:
            with self._lock:
                self._bottom_clearance_m = altitude.bottom_clearance_m
                self._est_altitude_m = altitude.altitude_relative_m

        def on_distance(distance: Any, _user_data: Any) -> None:
            with self._lock:
                # A dropout reports the sensor maximum, the Lightware convention, not a range.
                if (self._sampling and self._cos_tilt > 0.5
                        and math.isfinite(distance.current_distance_m)
                        and distance.current_distance_m < distance.maximum_distance_m
                        and math.isfinite(self._true_altitude_m)):
                    self._samples.append(RangeSample(
                        distance.current_distance_m, self._true_altitude_m / self._cos_tilt,
                        self._bottom_clearance_m, self._true_altitude_m, self._est_altitude_m,
                        self._cos_tilt))

        self._telemetry.subscribe_attitude_euler(on_attitude)
        self._telemetry.subscribe_ground_truth(on_ground_truth)
        self._telemetry.subscribe_altitude(on_altitude)
        self._telemetry.subscribe_distance_sensor(on_distance)

    def set_sampling(self, on: bool) -> None:
        with self._lock:
            self._sampling = on

    def check_leg_over_terrain(self, check_band: bool, check_following: bool) -> None:
        """check_band compares the range with the range aid height, which only reads on a leg flown
        at a fixed altitude, and check_following asks for the ground being held instead."""
        with self._lock:
            samples = list(self._samples)
        assert len(samples) > 100

        # The ground height under the beam is the flat reading minus the real one.
        terrain = sorted((s.flat_plane_m - s.range_m) * s.cos_tilt for s in samples)
        terrain_min = terrain[int(TERRAIN_PERCENTILE * (len(terrain) - 1))]
        terrain_max = terrain[int((1.0 - TERRAIN_PERCENTILE) * (len(terrain) - 1))]
        aid_fraction = sum(s.range_m < RANGE_AID_HEIGHT_M for s in samples) / len(samples)
        true_altitudes = [s.true_altitude_m for s in samples]
        estimates = [s.est_altitude_m for s in samples]
        true_span = max(true_altitudes) - min(true_altitudes)
        est_span = max(estimates) - min(estimates)
        clearance = [s for s in samples if math.isfinite(s.bottom_clearance_m)]
        clearance_close = sum(abs(s.bottom_clearance_m - s.range_m) < CLEARANCE_TOLERANCE_M
                              for s in clearance)

        log("terrain seen by the range: {:.2f} to {:.2f} m over {} samples".format(
            terrain_min, terrain_max, len(samples)))
        log("range under {} m for {:.0f}% of the samples".format(
            RANGE_AID_HEIGHT_M, 100.0 * aid_fraction))
        log("true altitude moved {:.2f} m, estimated {:.2f} m".format(true_span, est_span))
        log("fused clearance within {} m of the range on {} of {} samples".format(
            CLEARANCE_TOLERANCE_M, clearance_close, len(clearance)))

        assert terrain_max > MIN_TERRAIN_SEEN_M
        assert terrain_min < -MIN_TERRAIN_SEEN_M

        if check_band:
            assert 0.3 < aid_fraction < 0.7

        if check_following:
            # With the range as the height source the true altitude rides the ground while the
            # estimate holds, so the estimate moves by less than half of what the vehicle moved.
            assert true_span > MIN_RIDE_M
            assert est_span < 1.0
            assert est_span < 0.5 * true_span

        assert len(clearance) > 100
        assert clearance_close > 0.9 * len(clearance)

    def check_landed_on_rise(self, rise_m: float, min_rise_m: float) -> None:
        landed_m = (self._telemetry.ground_truth().absolute_altitude_m
                    - self.home_absolute_altitude_m())
        log("landed {:.2f} m above home, the rise is {} m".format(landed_m, rise_m))
        assert landed_m > min_rise_m

    def fly_orbit(self, center_n: float, center_e: float, radius_m: float, altitude_m: float,
                  timeout_s: float) -> None:
        """One full turn around center at altitude through PX4's orbit mode, the DO_ORBIT command a
        ground station sends. The turn is counted from telemetry so the test knows when the ring is
        closed."""
        latitude_deg, longitude_deg = self.get_coordinate_transformation().global_from_local(
            center_n, center_e)
        self._action.do_orbit(radius_m, ORBIT_SPEED_M_S,
                              OrbitYawBehavior.HOLD_FRONT_TANGENT_TO_CIRCLE,
                              latitude_deg, longitude_deg,
                              self.home_absolute_altitude_m() + altitude_m)

        last_angle = [math.nan]
        swept_rad = [0.0]

        def ring_closed() -> bool:
            position = self._telemetry.position_velocity_ned().position
            angle = math.atan2(position.east_m - center_e, position.north_m - center_n)
            if math.isfinite(last_angle[0]):
                delta = angle - last_angle[0]
                swept_rad[0] += math.atan2(math.sin(delta), math.cos(delta))
            last_angle[0] = angle
            return abs(swept_rad[0]) >= 2.0 * math.pi

        assert self.poll_condition_with_timeout(ring_closed, timeout_s), \
            "Orbit swept {:.0f} deg within {} s".format(math.degrees(swept_rad[0]), timeout_s)
        log("orbit swept {:.0f} deg".format(math.degrees(swept_rad[0])))
