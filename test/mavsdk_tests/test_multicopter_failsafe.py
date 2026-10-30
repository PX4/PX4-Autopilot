from typing import Any

import pytest
from mavsdk.plugins.failure import FailureType, FailureUnit  # type: ignore

from autopilot_tester import AutopilotTester, HeightSource, MissionOptions

HEIGHT_SOURCES = [pytest.param(HeightSource.BARO, id="baro"),
                  pytest.param(HeightSource.GPS, id="gps")]


def start_mission_and_inject_failure(tester: AutopilotTester, unit: Any,
                                     failure_type: Any) -> None:
    tester.wait_until_ready()
    tester.prepare_square_mission(MissionOptions(rtl_at_end=True))
    tester.arm()
    tester.set_param_int("SYS_FAILURE_EN", 1)
    tester.start_and_wait_for_mission_sequence(1)
    tester.inject_failure(unit, failure_type)


@pytest.mark.multicopter
@pytest.mark.parametrize("height_source, disarm_timeout_s", [
    pytest.param(HeightSource.BARO, 300, id="baro"),
    pytest.param(HeightSource.GPS, 180, id="gps"),
])
def test_land_on_gps_lost_during_mission(tester: AutopilotTester, height_source: HeightSource,
                                         disarm_timeout_s: float) -> None:
    tester.set_height_source(height_source)
    start_mission_and_inject_failure(tester, FailureUnit.SENSOR_GPS, FailureType.OFF)
    # With no position aiding left, a blind descend is performed. MAVSDK has no Descend flight
    # mode, so we just wait for the disarm after landing.
    tester.wait_until_disarmed(disarm_timeout_s)


@pytest.mark.multicopter
def test_continue_on_mag_lost_during_mission(tester: AutopilotTester) -> None:
    start_mission_and_inject_failure(tester, FailureUnit.SENSOR_MAG, FailureType.OFF)
    tester.wait_for_mission_finished(90)
    tester.wait_until_disarmed(180)


# Disabled: the estimator sometimes diverges right after landing, which then prevents auto-disarm.
# It has no markers so that the runner's filters don't start PX4 just to skip it. It used to be
# marked multicopter and vtol.
@pytest.mark.skip(reason="estimator sometimes diverges after landing, preventing auto-disarm")
def test_continue_on_mag_stuck_during_mission(tester: AutopilotTester) -> None:
    start_mission_and_inject_failure(tester, FailureUnit.SENSOR_MAG, FailureType.STUCK)
    tester.wait_for_mission_finished(120)
    tester.wait_until_disarmed(180)


@pytest.mark.multicopter
@pytest.mark.parametrize("height_source", HEIGHT_SOURCES)
def test_continue_on_baro_lost_during_mission(tester: AutopilotTester,
                                              height_source: HeightSource) -> None:
    tester.set_height_source(height_source)
    start_mission_and_inject_failure(tester, FailureUnit.SENSOR_BARO, FailureType.OFF)
    tester.wait_for_mission_finished(90)
    tester.wait_until_disarmed(180)


@pytest.mark.multicopter
@pytest.mark.parametrize("height_source", HEIGHT_SOURCES)
def test_continue_on_baro_stuck_during_mission(tester: AutopilotTester,
                                               height_source: HeightSource) -> None:
    tester.set_height_source(height_source)
    start_mission_and_inject_failure(tester, FailureUnit.SENSOR_BARO, FailureType.STUCK)
    tester.wait_for_mission_finished(90)
    tester.wait_until_disarmed(180)
