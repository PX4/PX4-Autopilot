"""Optical flow aided navigation on a multicopter (sihsim_quadx_flow).

The model has no GNSS fusion: the horizontal velocity is observed by the simulated downward
facing optical flow sensor together with the distance sensor, and the height comes from the
barometer. The test provides the EKF origin the way a GCS would (SET_GPS_GLOBAL_ORIGIN, the
estimator never picks one on its own without GNSS), takes off and flies an orbit with the nose
pointing at the circle center. Circling that way keeps the vehicle translating in every direction
while it also keeps turning, so the flow rate the sensor reports is a mix of ground relative
velocity and body rates. Throughout the manoeuvre the estimated NED velocity is compared against
the SIH ground truth velocity.
"""

import pytest

from autopilot_tester_flow import AutopilotTesterFlow

# Well below SIH_OF_H_MAX / SIH_DISTSNSR_MAX (30 m) so the flow stays valid throughout.
FLIGHT_ALTITUDE_M = 10.0

# The center is one radius north of home, so the takeoff position is already on the circle.
ORBIT_RADIUS_M = 20.0
ORBIT_VELOCITY_M_S = 5.0  # ~25 s and, with the nose on the center, ~14 deg/s per turn
ORBIT_CENTER = (ORBIT_RADIUS_M, 0.0)

# The estimate has to track the truth: the two streams are paired by arrival, so a short
# acceleration between the two samples being compared already shows up as an error. Observed is
# ~0.1 m/s mean and ~0.3 m/s worst case, the tolerances leave room for the sampling jitter.
MEAN_VELOCITY_ERROR_TOLERANCE_M_S = 0.3
MAX_VELOCITY_ERROR_TOLERANCE_M_S = 1.0

# The circle the vehicle actually flew (ground truth) is checked against the commanded one. The
# band has to swallow the initial acceleration onto the circle and whatever the flow only position
# estimate drifts over one orbit, observed is ~2.5 m.
ORBIT_RADIUS_TOLERANCE_M = 8.0

# A full turn of the heading, which the vehicle only reaches by flying a whole orbit.
HEADING_SWEEP_DEG = 360.0


@pytest.mark.optical_flow
def test_orbit_tracks_ground_truth_velocity(tester_flow: AutopilotTesterFlow) -> None:
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
    tester.sleep_for(3)  # let the position controller settle into the hover

    tester.command_orbit(ORBIT_CENTER, ORBIT_RADIUS_M, ORBIT_VELOCITY_M_S, FLIGHT_ALTITUDE_M)
    tester.start_ground_truth_comparison()

    # Fly a full circle. The heading turns with it because the nose follows the circle center, so
    # the sweep doubles as the "the orbit is really being flown" check.
    tester.wait_until_heading_swept(HEADING_SWEEP_DEG, 120)
    tester.stop_ground_truth_comparison()

    tester.check_ground_truth_on_circle(ORBIT_CENTER, ORBIT_RADIUS_M, ORBIT_RADIUS_TOLERANCE_M)
    tester.check_velocity_error_below(MEAN_VELOCITY_ERROR_TOLERANCE_M_S,
                                      MAX_VELOCITY_ERROR_TOLERANCE_M_S)

    tester.land()
    tester.wait_until_disarmed()
