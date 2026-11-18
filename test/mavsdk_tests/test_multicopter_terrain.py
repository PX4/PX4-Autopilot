"""The SIH ground is synthetic terrain (SIH_TERR_MODE 2) for these tests, set through the
environment in the config. The vehicle flies a low, slow leg over rising and falling ground with
the distance sensor on, which is the regime EKF2's conditional range aid runs in. The aid starts
below 0.7 times EKF2_RNG_A_HMAX and EKF2_RNG_A_VMAX, 3.5 m and 0.7 m/s at the defaults, and holds
until 5 m and 1 m/s, so the leg flies at 5 m and 0.6 m/s.
"""

import math

import pytest
from mavsdk.plugins.offboard import PositionNedYaw  # type: ignore

from autopilot_tester import RcLossException
from autopilot_tester_terrain import AutopilotTesterTerrain

# Seed 80 from the config, the library's 5 m amplitude and 200 m wavelength, bearing 30 deg,
# evaluated with the terrain library on the host. Along the 100 m leg the ground runs from 2.9 m
# below home to 2.3 m above. At 5 m of flight altitude the range is under 3.5 m for a third of the
# leg and under 5 m for half of it, with 2.7 m of clearance at the least. The highest point past
# 40 m out is 57 m along the leg, 2.3 m above home.
FLIGHT_ALTITUDE_M = 5.0
LEG_LENGTH_M = 100.0
BEARING_DEG = 30.0
LANDING_DISTANCE_M = 57.0
LANDING_RISE_M = 2.3
# A landed vehicle sits on the rise, not at home altitude, with margin for the contact model.
MIN_LANDED_RISE_M = 1.5

# The orbit sits on the same leg, centered 45 m out with a 20 m radius. Around the ring the ground
# runs from 3.0 m below home to 2.2 m above, and at 5 m the range is under 5 m for half the ring.
# The ring starts due north of the center, on ground 1.2 m above home.
ORBIT_CENTER_DISTANCE_M = 45.0
ORBIT_RADIUS_M = 20.0
ORBIT_START_RISE_M = 1.2

ACCEPTANCE_RADIUS_M = 1.0
GOTO_TIMEOUT_S = 240

NORTH_PER_M = math.cos(math.radians(BEARING_DEG))
EAST_PER_M = math.sin(math.radians(BEARING_DEG))


def on_the_leg(distance_m: float, yaw_deg: float) -> PositionNedYaw:
    return PositionNedYaw(north_m=distance_m * NORTH_PER_M, east_m=distance_m * EAST_PER_M,
                          down_m=-FLIGHT_ALTITUDE_M, yaw_deg=yaw_deg)


def prepare(tester: AutopilotTesterTerrain) -> None:
    tester.wait_until_ready()
    tester.store_home()
    tester.start_sampling()
    tester.set_rc_loss_exception(RcLossException.OFFBOARD)
    tester.arm()


@pytest.mark.terrain
def test_rangefinder_reads_the_terrain_and_lands_on_it(
        tester_terrain: AutopilotTesterTerrain) -> None:
    prepare(tester_terrain)

    # The nose points along the leg on the way out and back at it on the return.
    tester_terrain.offboard_goto(on_the_leg(0.0, BEARING_DEG), ACCEPTANCE_RADIUS_M, GOTO_TIMEOUT_S)

    tester_terrain.set_sampling(True)
    tester_terrain.offboard_goto(on_the_leg(LEG_LENGTH_M, BEARING_DEG), ACCEPTANCE_RADIUS_M,
                                 GOTO_TIMEOUT_S)
    tester_terrain.set_sampling(False)
    tester_terrain.check_leg_over_terrain(check_band=True, check_following=False)

    tester_terrain.offboard_goto(on_the_leg(LANDING_DISTANCE_M, BEARING_DEG + 180.0),
                                 ACCEPTANCE_RADIUS_M, GOTO_TIMEOUT_S)
    tester_terrain.offboard_land()
    tester_terrain.wait_until_disarmed(120)
    tester_terrain.check_landed_on_rise(LANDING_RISE_M, MIN_LANDED_RISE_M)


@pytest.mark.terrain
def test_rangefinder_reads_the_terrain_around_an_orbit(
        tester_terrain: AutopilotTesterTerrain) -> None:
    prepare(tester_terrain)

    center_n = ORBIT_CENTER_DISTANCE_M * NORTH_PER_M
    center_e = ORBIT_CENTER_DISTANCE_M * EAST_PER_M

    tester_terrain.offboard_goto(on_the_leg(0.0, 0.0), ACCEPTANCE_RADIUS_M, GOTO_TIMEOUT_S)
    tester_terrain.offboard_goto(
        PositionNedYaw(north_m=center_n + ORBIT_RADIUS_M, east_m=center_e,
                       down_m=-FLIGHT_ALTITUDE_M, yaw_deg=90.0),
        ACCEPTANCE_RADIUS_M, GOTO_TIMEOUT_S)

    tester_terrain.set_sampling(True)
    tester_terrain.fly_orbit(center_n, center_e, ORBIT_RADIUS_M, FLIGHT_ALTITUDE_M, 420)
    tester_terrain.set_sampling(False)
    tester_terrain.check_leg_over_terrain(check_band=False, check_following=True)

    tester_terrain.land()
    tester_terrain.wait_until_disarmed(120)
    tester_terrain.check_landed_on_rise(ORBIT_START_RISE_M, 0.5)
