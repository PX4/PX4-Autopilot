"""AutopilotTester for taking off from a moving platform.

SIH simulates the platform as a ground plane moving north with SIH_PLAT_VN, so a vehicle on
the ground moves with it.
"""

import math
from typing import Tuple

from autopilot_tester import AutopilotTester


class AutopilotTesterMovingPlatform(AutopilotTester):

    def platform_velocity_ne(self) -> Tuple[float, float]:
        return self.get_param_float("SIH_PLAT_VN"), 0.0

    def drag_per_kg(self) -> float:
        """SIH's first order drag deceleration per airspeed [1/s]."""
        return self.get_param_float("SIH_KDV") / self.get_param_float("SIH_MASS")

    def estimated_velocity_ne(self) -> Tuple[float, float]:
        velocity = self._telemetry.position_velocity_ned().velocity
        return float(velocity.north_m_s), float(velocity.east_m_s)

    def wait_until_estimate_tracks_platform(self, tolerance_m_s: float, timeout_s: float) -> None:
        """Wait until the platform reached its velocity and the estimate follows it."""
        platform_ne = self.platform_velocity_ne()

        def tracking() -> bool:
            estimate_ne = self.estimated_velocity_ne()
            return math.dist(estimate_ne, platform_ne) < tolerance_m_s

        assert self.poll_condition_with_timeout(tracking, timeout_s), \
            "Estimated velocity {} doesn't follow the platform velocity {}".format(
                self.estimated_velocity_ne(), platform_ne)
