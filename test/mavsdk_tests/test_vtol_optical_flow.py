"""Optical flow aided navigation on a VTOL (sihsim_standard_vtol_flow).

Like the multicopter flow test this model flies without GNSS fusion, so the EKF origin has to be
provided the way a GCS would. The vehicle takes off in multicopter mode, transitions to fixed wing
on command and flies a loiter, which keeps it banked and turning while it translates much faster
than a multicopter would. The whole loiter is flown low enough for the flow to stay in range
(SIH_OF_H_MAX / SIH_DISTSNSR_MAX are 30 m on this model), and the estimated NED velocity is
compared against the SIH ground truth velocity throughout.
"""

import math

import pytest

from autopilot_tester_flow import AutopilotTesterFlow

FLIGHT_ALTITUDE_M = 10.0

# A wide loiter keeps the bank angle small, which keeps the downward facing flow sensor looking at
# the ground and the distance it measures close to the altitude.
LOITER_RADIUS_M = 100.0

# Band the vehicle has to enter to count as established on the loiter, and the band the flown
# (ground truth) circle is checked against afterwards. The latter has to swallow how tightly the
# fixed wing tracks the circle plus the drift of the flow only position estimate, observed is
# ~1.6 m.
LOITER_CAPTURE_TOLERANCE_M = 20.0
LOITER_RADIUS_TOLERANCE_M = 10.0

# The estimate has to track the truth: the two streams are paired by arrival, so a short
# acceleration between the two samples being compared already shows up as an error. Observed is
# ~0.1 m/s mean and ~0.3 m/s worst case, the tolerances leave room for the sampling jitter.
MEAN_VELOCITY_ERROR_TOLERANCE_M_S = 0.3
MAX_VELOCITY_ERROR_TOLERANCE_M_S = 1.0

# A full turn of the heading, which the vehicle only reaches by flying a whole loiter.
HEADING_SWEEP_DEG = 360.0


@pytest.mark.optical_flow_vtol
def test_loiter_tracks_ground_truth_velocity(tester_flow: AutopilotTesterFlow) -> None:
    tester = tester_flow

    # The ground truth is streamed independently of the estimator, so home can be stored before
    # there is any global position, and it is the position the EKF origin gets set to.
    tester.store_home()
    tester.set_ekf_origin_to_home()

    tester.wait_until_armable()
    tester.set_takeoff_altitude(FLIGHT_ALTITUDE_M)
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(FLIGHT_ALTITUDE_M, 60)

    tester.transition_to_fixedwing()
    tester.wait_until_fixedwing(10)
    tester.sleep_for(5)  # let the vehicle accelerate to its cruise speed and settle its heading

    # Center the loiter so that the current position and heading are already on and tangent to
    # the circle: the vehicle joins it immediately instead of flying out to it from the (near-)
    # center, which would otherwise dominate the ground truth comparison below.
    loiter_center = tester.tangent_circle_center(LOITER_RADIUS_M)

    tester.command_orbit(loiter_center, LOITER_RADIUS_M, math.nan, FLIGHT_ALTITUDE_M)
    tester.wait_until_on_circle(loiter_center, LOITER_RADIUS_M, LOITER_CAPTURE_TOLERANCE_M, 180)

    # Brief settle for the lateral guidance to lock onto the new path precisely.
    tester.sleep_for(3)

    tester.start_ground_truth_comparison()

    # Fly a full circle: the heading of a fixed wing follows the circle, so the sweep doubles as
    # the "the loiter is really being flown" check.
    tester.wait_until_heading_swept(HEADING_SWEEP_DEG, 180)
    tester.stop_ground_truth_comparison()

    tester.check_ground_truth_on_circle(loiter_center, LOITER_RADIUS_M, LOITER_RADIUS_TOLERANCE_M)
    tester.check_velocity_error_below(MEAN_VELOCITY_ERROR_TOLERANCE_M_S,
                                      MAX_VELOCITY_ERROR_TOLERANCE_M_S)
