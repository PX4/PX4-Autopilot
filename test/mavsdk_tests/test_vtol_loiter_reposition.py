import pytest

from autopilot_tester_loiter import AutopilotTesterLoiter

# Home-relative center of the "work" loiter. Placed well away from home so that Return climbs to
# the full return altitude (close to home the return cone would cap the climb).
WORK_CENTER = (250.0, 0.0)

TAKEOFF_ALTITUDE_M = 40.0
LOITER_ALTITUDE_M = 40.0

# Clearly different from the default NAV_LOITER_RAD (80 m), so a default-radius circle would fall
# outside the tolerance band.
NON_DEFAULT_RADIUS_M = 120.0
RADIUS_TOLERANCE_M = 30.0
ALTITUDE_TOLERANCE_M = 8.0


def arm_takeoff_transition_hold(tester: AutopilotTesterLoiter) -> None:
    tester.wait_until_ready()
    tester.store_home()
    tester.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)
    tester.sleep_for(3)
    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 30)
    tester.transition_to_fixedwing()
    tester.wait_until_fixedwing(5)
    tester.sleep_for(1)
    tester.enter_hold_and_wait()


def establish_work_orbit(tester: AutopilotTesterLoiter) -> None:
    tester.command_orbit(WORK_CENTER, NON_DEFAULT_RADIUS_M, LOITER_ALTITUDE_M)
    tester.wait_until_on_loiter(WORK_CENTER, NON_DEFAULT_RADIUS_M, RADIUS_TOLERANCE_M,
                                settle_s=12, timeout_s=180)


@pytest.mark.vtol
def test_return_climbs_on_established_non_default_radius_loiter(
        tester_loiter: AutopilotTesterLoiter) -> None:
    tester = tester_loiter
    arm_takeoff_transition_hold(tester)
    establish_work_orbit(tester)

    # Climb clearly above the loiter altitude when Return is engaged.
    rtl_altitude_m = 80.0
    tester.set_rtl_altitude(rtl_altitude_m)
    tester.execute_rtl()

    # Accept the altitude the controller itself considers reached, with 20% margin on top.
    altitude_tolerance_m = 1.2 * tester.fw_altitude_acceptance_radius()

    # Return must climb to the return altitude while circling the very same loiter (same center
    # and non-default radius), not re-center a new circle.
    tester.check_climbs_on_loiter(WORK_CENTER, NON_DEFAULT_RADIUS_M, RADIUS_TOLERANCE_M,
                                  rtl_altitude_m, altitude_tolerance_m, 180)


@pytest.mark.vtol
def test_hold_during_altitude_change_keeps_loiter_and_locks_altitude(
        tester_loiter: AutopilotTesterLoiter) -> None:
    tester = tester_loiter
    arm_takeoff_transition_hold(tester)
    establish_work_orbit(tester)

    # Command the same loiter at a higher altitude, so the vehicle climbs while staying on it.
    climb_target_m = 200.0
    tester.command_orbit(WORK_CENTER, NON_DEFAULT_RADIUS_M, climb_target_m)

    # Catch the vehicle part way through the climb and issue an all-NaN Hold reposition.
    tester.wait_until_relative_altitude_above(60.0, 120)
    tester.command_hold_here()
    # Let the Hold take effect and the climb arrest.
    tester.sleep_for(2)

    locked_altitude_m = tester.current_relative_altitude()

    # The Hold must arrest the climb well before the commanded target altitude ...
    assert locked_altitude_m < climb_target_m - ALTITUDE_TOLERANCE_M
    # ... lock the altitude the vehicle had reached ...
    tester.check_holds_altitude(locked_altitude_m, ALTITUDE_TOLERANCE_M, 40)
    # ... and keep circling the same loiter.
    tester.check_stays_on_loiter(WORK_CENTER, NON_DEFAULT_RADIUS_M, RADIUS_TOLERANCE_M, 40)


@pytest.mark.vtol
def test_hold_while_transiting_creates_new_loiter_at_current_position(
        tester_loiter: AutopilotTesterLoiter) -> None:
    tester = tester_loiter
    arm_takeoff_transition_hold(tester)

    # Command a loiter far away and Hold before getting anywhere near it.
    far_center = (600.0, 0.0)
    tester.command_reposition(far_center, LOITER_ALTITUDE_M)
    tester.sleep_for(6)

    hold_position = tester.current_local_position()
    tester.command_hold_here()
    # Let the fresh loiter circle be captured before checking it.
    tester.sleep_for(10)

    default_radius_m = tester.default_loiter_radius()

    # A fresh loiter must be created around where Hold was pressed ...
    tester.check_stays_within(hold_position, default_radius_m + RADIUS_TOLERANCE_M, 40)
    # ... and the vehicle must not keep flying toward the far loiter.
    tester.check_never_reaches(far_center, 150.0, 40)


@pytest.mark.vtol
def test_hold_on_figure_eight_reverts_to_circular_loiter(
        tester_loiter: AutopilotTesterLoiter) -> None:
    tester = tester_loiter
    arm_takeoff_transition_hold(tester)

    tester.command_figure_eight(WORK_CENTER, 150.0, 50.0, LOITER_ALTITUDE_M)
    # Give it time to actually track the pattern.
    tester.sleep_for(35)

    hold_position = tester.current_local_position()
    tester.command_hold_here()
    tester.sleep_for(2)

    default_radius_m = tester.default_loiter_radius()

    # After Hold the vehicle must settle into a single circular loiter around the Hold position.
    # If it kept flying the figure eight it would swing out to the ~150 m lobes and leave this
    # bound.
    tester.check_stays_within(hold_position, default_radius_m + 70.0, 40)


@pytest.mark.vtol
def test_altitude_only_reposition_keeps_loiter(tester_loiter: AutopilotTesterLoiter) -> None:
    tester = tester_loiter
    arm_takeoff_transition_hold(tester)
    establish_work_orbit(tester)

    # Only an altitude change (lat/lon NaN): the loiter center and non-default radius must be kept
    # while the vehicle climbs to the new altitude.
    new_altitude_m = 70.0
    tester.command_reposition_altitude(new_altitude_m)
    tester.check_climbs_on_loiter(WORK_CENTER, NON_DEFAULT_RADIUS_M, RADIUS_TOLERANCE_M,
                                  new_altitude_m, ALTITUDE_TOLERANCE_M, 120)
