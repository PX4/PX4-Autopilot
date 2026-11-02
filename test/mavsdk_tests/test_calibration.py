from typing import Dict, Optional

import pytest

from autopilot_tester_calibration import AutopilotTesterCalibration

MAV_RESULT_ACCEPTED = 0

# Before a cancel was handed to the running calibration, these took from tens of seconds to never.
CANCEL_TIMEOUT_S = 2.0


@pytest.mark.calibration
def test_mag_calibration(tester_calibration: AutopilotTesterCalibration) -> None:
    tester = tester_calibration
    tester.wait_until_ready()
    tester.start_calibration(param2=1)
    tester.shell("simulator_sih orient all")
    tester.wait_for_text("[cal] calibration done: mag", 150)


@pytest.mark.calibration
def test_accel_calibration(tester_calibration: AutopilotTesterCalibration) -> None:
    tester = tester_calibration
    tester.wait_until_ready()
    tester.start_calibration(param5=1)
    tester.shell("simulator_sih orient all")
    tester.wait_for_text("[cal] calibration done: accel", 150)


@pytest.mark.calibration
@pytest.mark.parametrize("params, orient, cancel_after_text, cancel_after_s", [
    pytest.param({"param5": 1}, "down rotate", None, 3.0, id="accel_never_still"),
    pytest.param({"param2": 1}, "all", "[cal] down side done", 2.0, id="mag_between_sides"),
    pytest.param({"param2": 1}, "all", "[cal] down side calibration: progress", 0.0,
                 id="mag_collecting"),
    pytest.param({"param5": 2}, "down rotate", None, 2.0, id="level_moving"),
    pytest.param({"param1": 1}, "down rotate", None, 2.0, id="gyro_moving"),
    pytest.param({"param3": 1}, None, None, 0.0, id="baro"),
])
def test_calibration_cancel(tester_calibration: AutopilotTesterCalibration,
                            params: Dict[str, float], orient: Optional[str],
                            cancel_after_text: Optional[str], cancel_after_s: float) -> None:
    tester = tester_calibration
    tester.wait_until_ready()

    if orient:
        tester.shell("simulator_sih orient " + orient)

    tester.start_calibration(**params)

    if cancel_after_text:
        tester.wait_for_text(cancel_after_text, 60)

    tester.sleep_for(cancel_after_s)
    tester.cancel_calibration()
    tester.wait_for_text("[cal] calibration cancelled", CANCEL_TIMEOUT_S)

    # Late acks would arrive in this time.
    tester.sleep_for(2.0)
    assert tester.calibration_acks() == [MAV_RESULT_ACCEPTED]
    assert not tester.has_text("[cal] calibration failed")
