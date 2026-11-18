"""Checks on how a multicopter leaves the ground or a moving platform during takeoff.

The recording is made by AutopilotTester.start_recording_takeoff(). Heights are relative to the
liftoff position, velocities relative to the platform, which is the ground for a normal takeoff.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple

# Below this height the vehicle is still on the platform or about to leave it. Tilting there
# pivots it over its landing gear, so it risks tipping over.
GROUND_HEIGHT_M = 0.1
GROUND_MAX_TILT_DEG = 3.0

# Below this height the landing gear can still scrape over the platform, so the vehicle should
# climb level and not accelerate horizontally beyond what drag explains.
LEVEL_HEIGHT_M = 0.5
LEVEL_MAX_TILT_DEG = 5.0
LEVEL_MAX_HORIZONTAL_ACCELERATION_M_S2 = 1.0
LEVEL_MAX_VELOCITY_CHANGE_M_S = 0.5

# Above this height the vehicle counts as lifted off for the time drag acts on it.
LIFTOFF_HEIGHT_M = 0.03

# Time span to differentiate the velocity over, long enough to not amplify the estimate's noise.
ACCELERATION_WINDOW_S = 0.1


@dataclass
class AttitudeSample:
    time_s: float
    height_m: float  # from the latest position sample
    tilt_deg: float


@dataclass
class PositionSample:
    time_s: float
    height_m: float
    velocity_ne_m_s: Tuple[float, float]


@dataclass
class TakeoffRecording:
    attitude: List[AttitudeSample] = field(default_factory=list)
    position: List[PositionSample] = field(default_factory=list)

    def max_tilt_deg(self, below_height_m: float) -> float:
        return max((s.tilt_deg for s in self.attitude if s.height_m < below_height_m), default=0.0)

    def max_velocity_change_m_s(self, platform_ne: Tuple[float, float],
                                below_height_m: float) -> float:
        return max((math.dist(s.velocity_ne_m_s, platform_ne)
                    for s in self.position if s.height_m < below_height_m), default=0.0)

    def max_horizontal_acceleration_m_s2(self, below_height_m: float) -> float:
        """Maximum horizontal acceleration, differentiated over ACCELERATION_WINDOW_S."""
        maximum = 0.0
        j = 0
        for i, start in enumerate(self.position):
            if start.height_m >= below_height_m:
                continue
            j = max(j, i + 1)
            while (j < len(self.position)
                   and self.position[j].time_s - start.time_s < ACCELERATION_WINDOW_S):
                j += 1
            if j >= len(self.position):
                break
            end = self.position[j]
            dv = math.dist(end.velocity_ne_m_s, start.velocity_ne_m_s)
            maximum = max(maximum, dv / (end.time_s - start.time_s))
        return maximum

    def time_airborne_below_s(self, below_height_m: float) -> float:
        """Time from lifting off until first reaching below_height_m."""
        liftoff = next((s.time_s for s in self.position if s.height_m > LIFTOFF_HEIGHT_M), None)
        reached = next((s.time_s for s in self.position if s.height_m >= below_height_m), None)
        if liftoff is None or reached is None:
            return 0.0
        return reached - liftoff


def summarize_takeoff(recording: TakeoffRecording, platform_ne: Tuple[float, float],
                      heights_m: List[float]) -> List[str]:
    """Maximum tilt, acceleration and velocity change relative to the platform below each height."""
    return ["Below {:.1f} m: max tilt {:.1f} deg, max horizontal acceleration {:.2f} m/s^2, "
            "max velocity change {:.2f} m/s".format(
                height_m,
                recording.max_tilt_deg(height_m),
                recording.max_horizontal_acceleration_m_s2(height_m),
                recording.max_velocity_change_m_s(platform_ne, height_m))
            for height_m in heights_m]


def check_takeoff(recording: TakeoffRecording, platform_ne: Tuple[float, float] = (0.0, 0.0),
                  drag_per_kg: float = 0.0) -> List[str]:
    """Return the failed checks, empty if the takeoff was fine.

    Leaving a moving platform, the vehicle is decelerated by drag through the air at rest. The
    vehicle should stay level instead of compensating for that, so the drag deceleration
    drag_per_kg * platform speed [1/s * m/s] is allowed on top of the limits.
    """
    failures = []

    tilt_deg = recording.max_tilt_deg(GROUND_HEIGHT_M)
    if tilt_deg > GROUND_MAX_TILT_DEG:
        failures.append("Tilted {:.1f} deg on the ground (below {} m), more than {} deg".format(
            tilt_deg, GROUND_HEIGHT_M, GROUND_MAX_TILT_DEG))

    tilt_deg = recording.max_tilt_deg(LEVEL_HEIGHT_M)
    if tilt_deg > LEVEL_MAX_TILT_DEG:
        failures.append("Tilted {:.1f} deg below {} m, more than {} deg".format(
            tilt_deg, LEVEL_HEIGHT_M, LEVEL_MAX_TILT_DEG))

    drag_m_s2 = drag_per_kg * math.hypot(*platform_ne)

    max_acceleration_m_s2 = LEVEL_MAX_HORIZONTAL_ACCELERATION_M_S2 + drag_m_s2
    acceleration_m_s2 = recording.max_horizontal_acceleration_m_s2(LEVEL_HEIGHT_M)
    if acceleration_m_s2 > max_acceleration_m_s2:
        failures.append("Accelerated horizontally with {:.2f} m/s^2 below {} m, "
                        "more than {:.2f} m/s^2".format(acceleration_m_s2, LEVEL_HEIGHT_M,
                                                        max_acceleration_m_s2))

    max_velocity_change_m_s = (LEVEL_MAX_VELOCITY_CHANGE_M_S
                               + drag_m_s2 * recording.time_airborne_below_s(LEVEL_HEIGHT_M))
    velocity_change_m_s = recording.max_velocity_change_m_s(platform_ne, LEVEL_HEIGHT_M)
    if velocity_change_m_s > max_velocity_change_m_s:
        failures.append("Velocity changed by {:.2f} m/s relative to the platform below {} m, "
                        "more than {:.2f} m/s".format(velocity_change_m_s, LEVEL_HEIGHT_M,
                                                      max_velocity_change_m_s))

    return failures
