import os

import pytest

from autopilot_tester_rtl import AutopilotTesterRtl

PLAN_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "multicopter_mission_geofence_avoid.plan")


@pytest.mark.multicopter
def test_return_direct_with_geofence_obstruction(tester_rtl: AutopilotTesterRtl) -> None:
    """Exercise the RTL_DIRECT geofence avoidance planner (visibility graph + Dijkstra).

    The mission flies the vehicle to a point from which the straight-line return to home would
    cut through an exclusion polygon, so the planner must detour around it. The vehicle must
    return home and disarm, and its ground truth position must never breach any of the fences.
    """
    tester = tester_rtl
    tester.wait_until_ready()
    tester.store_home()

    tester.load_qgc_mission_and_geofence_here(PLAN_FILE)

    # Warning only: the planner still picks up the fence (it reads the polygons regardless of
    # GF_ACTION), but a breach won't trigger a Hold/Return failsafe that would mask the failure.
    tester.set_param_int("GF_ACTION", 1)

    tester.set_rtl_type(0)  # RTL_DIRECT
    tester.set_rtl_appr_force(0)

    tester.sleep_for(3)

    tester.arm()

    # Latch any breach from here until disarm.
    tester.start_monitoring_geofence_breach()
    tester.execute_rtl_when_reaching_mission_sequence(3)

    tester.wait_until_disarmed(400)

    tester.check_no_geofence_breach()
    tester.check_home_within(5.0)
