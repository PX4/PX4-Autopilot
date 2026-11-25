"""Estimator instance failover on SIH with two IMUs.

A failure injected on one IMU has to land on that IMU and on no other, the estimator selector has
to leave the instance behind it, and the vehicle has to ride through the handover. The vehicle is
judged against the SIH ground truth, because the estimate is what the fault disturbs. A handover
steps the estimate on purpose, so a band on the estimate would have to allow that step and would
stop catching anything.

The rules the selector decides by are covered in src/modules/ekf2/EKF2SelectorTest.cpp, which
needs no simulator. These tests cover what that unit test cannot: a vehicle with real estimators
and controllers staying in the air while the fault moves.

Needs SIH_IMU_COUNT 2, EKF2_MULTI_IMU 2 and SENS_IMU_MODE 0, which the ekf_failover entry in
configs/sih-sitl.json sets.
"""

import statistics
from typing import Dict, List, Tuple

import pytest
from mavsdk.plugins.failure import FailureType, FailureUnit  # type: ignore

from autopilot_tester import MissionOptions, log
from autopilot_tester_imu_failover import AccelSample, AutopilotTesterImuFailover

# The fault is a constant accelerometer bias on all three axes (`failure accel wrong`). The sensor
# keeps publishing, so one estimator instance is degraded without being silenced. The bias is far
# beyond what the estimator can learn (EKF2_ABL_LIM), so the innovations of the instance on that
# IMU stay one-sided and its test ratios rise above those of the instance on the clean IMU. With
# two IMUs the selector cannot tell which accelerometer is faulty, so the test ratios are what it
# switches on.
FAULT_TYPE = FailureType.WRONG
FAULT_BIAS_PARAM = "SYS_FAIL_A_BIAS"
FAULT_BIAS_M_S2 = 7.0

CONSTANTS_ONE_G = 9.80665
MG_PER_M_S2 = 1000.0 / CONSTANTS_ONE_G

IMU_RATE_HZ = 50.0

TAKEOFF_ALTITUDE_M = 20.0

# The bounds below come from repeated runs of both tests at speed factors 10 and 1, with margin
# over the worst case seen. They are in simulation time and against the ground truth, so the
# speed factor does not move them.

# How far the vehicle may climb above the hover. An estimate that thinks the vehicle is falling
# makes the controller add thrust, so the vehicle goes up for as long as the biased instance is
# primary. It reached 2.5 m.
ALTITUDE_CEILING_M = 3.5

# And how far it may sink below it, which it does on the way back from that climb. It reached
# 0.5 m.
ALTITUDE_FLOOR_M = 1.5

# How far it may move sideways from where it hovered. The bias is on all three axes, so the
# vehicle is pushed sideways as well until the handover. It reached 2.6 m.
HOVER_DRIFT_M = 5.0

# How far it may still be from where it started once the faults are cleared and it has settled.
# It was within 0.3 m. This is the altitude tolerance of main's other failure injection tests.
RECOVERY_TOLERANCE_M = 1.0

# A fault change has to reach the IMU drivers and the samples already in flight have to drain
# before the next measurement starts.
FAULT_SETTLE_S = 3.0

# Long enough to collect many samples of both IMUs and for the selector to settle on an
# instance, so that one phase does not bleed into the next.
FAULT_MEASURE_S = 17.0

# Ground truth arrives as HIL_STATE_QUATERNION at 15 Hz of simulated time. Requiring a third of
# that over a window catches a stream that stalled or was starved under the speed factor, which
# would otherwise leave every extreme at its starting value and read as a pass.
MIN_GROUND_TRUTH_RATE_HZ = 5.0

# How long the selector may take to leave the instance behind the faulty IMU, from the injection
# to the switch. It switches on the relative test ratios, within 0.6 s. A selector that waits
# for the primary to turn unhealthy takes 5.6 s or more. The bound sits between the two. The
# time the injection command spends on the link counts against it and grows with the speed
# factor, which is what the room above 0.6 s is for.
HANDOVER_TIMEOUT_S = 3.0

MISSION_LEG_M = 30.0
MISSION_ALTITUDE_M = 20.0

# How long the mission may take to reach its first leg, where the fault is injected.
MISSION_FIRST_LEG_TIMEOUT_S = 120.0

MISSION_FINISH_TIMEOUT_S = 180.0

# How long the IMUs are recorded mid mission to show the fault landed. Longer than
# HANDOVER_TIMEOUT_S, so the handover is due by the time the recording ends.
MISSION_BIAS_CHECK_S = 4.0

# How far the vehicle may stray from the mission legs. The handover pushes it up to 1.3 m off the
# leg, against 0.3 m on the leg after it.
MISSION_CORRIDOR_M = 2.0

# How far the vehicle may leave the altitude it flew at when the fault was injected. It reached
# 2.4 m.
MISSION_ALTITUDE_BAND_M = 3.5


def bias_mg(biased: List[AccelSample], other: List[AccelSample]) -> List[float]:
    """Per axis difference between the mean accelerations of two IMUs, in mG. Both sit on the
    same vehicle, so everything but an injected bias cancels."""
    return [statistics.fmean(biased_axis) - statistics.fmean(other_axis)
            for biased_axis, other_axis in zip(zip(*biased), zip(*other))]


def format_mg(axes_mg: List[float]) -> str:
    return ", ".join("{:.0f}".format(axis) for axis in axes_mg)


def check_bias_on(imu: int, imu1: List[AccelSample], imu2: List[AccelSample]) -> None:
    """Check that the injected bias is on IMU `imu` (the failure instance, 1 or 2) and not on the
    other one. A bias on both IMUs, or on neither, reads as no difference."""
    biased, other = (imu1, imu2) if imu == 1 else (imu2, imu1)
    measured_mg = bias_mg(biased, other)
    expected_mg = FAULT_BIAS_M_S2 * MG_PER_M_S2
    log("IMU {} reads {} mG more than the other IMU, {:.0f} mG injected".format(
        imu, format_mg(measured_mg), expected_mg))
    assert all(abs(axis - expected_mg) < expected_mg / 2 for axis in measured_mg), \
        "IMU {} reads {} mG more than the other IMU, expected {:.0f} mG on every axis".format(
            imu, format_mg(measured_mg), expected_mg)


def check_no_bias(imu1: List[AccelSample], imu2: List[AccelSample]) -> None:
    """Check that the two IMUs read the same again."""
    measured_mg = bias_mg(imu1, imu2)
    limit_mg = FAULT_BIAS_M_S2 * MG_PER_M_S2 / 2
    log("IMU 1 reads {} mG more than IMU 2".format(format_mg(measured_mg)))
    assert all(abs(axis) < limit_mg for axis in measured_mg), \
        "IMU 1 reads {} mG more than IMU 2 with the faults cleared, expected < {:.0f} mG".format(
            format_mg(measured_mg), limit_mg)


def check_two_estimator_instances(tester: AutopilotTesterImuFailover) -> None:
    """Without two instances there is nothing to fail over to, and every handover check below
    would fail for the wrong reason."""
    instances = tester.estimator_selector_status("instances_available").get("instances_available")
    assert instances == 2, \
        "{} estimator instances, expected 2 (SIH_IMU_COUNT 2, EKF2_MULTI_IMU 2)".format(instances)


def selector_status(tester: AutopilotTesterImuFailover) -> Dict[str, int]:
    """The instance the selector flies on, how often it has changed and when it last did, in
    autopilot time."""
    status = tester.estimator_selector_status("primary_instance", "instance_changed_count",
                                              "last_instance_change")
    assert status, "No estimator_selector_status from the MAVLink shell"
    return status


def check_handover(tester: AutopilotTesterImuFailover, before: Dict[str, int],
                   fault_time_s: float, instance: int) -> Dict[str, int]:
    """Check that the selector has moved to `instance` exactly once since `before` was read, no
    later than HANDOVER_TIMEOUT_S after the fault injected at fault_time_s. Return the status,
    to compare the next fault with."""
    status = selector_status(tester)
    changes = status["instance_changed_count"] - before["instance_changed_count"]
    handover_s = status["last_instance_change"] * 1e-6 - fault_time_s
    log("primary estimator instance {}, {} change(s) since the fault{}".format(
        status["primary_instance"], changes,
        ", the last one {:.2f} s after it".format(handover_s) if changes else ""))
    assert status["primary_instance"] == instance, \
        "Primary estimator instance {}, expected {}".format(status["primary_instance"], instance)
    # More than one change means the selector went back to the faulty instance in between.
    assert changes == 1, \
        "Primary estimator instance changed {} times, expected once".format(changes)
    assert handover_s < HANDOVER_TIMEOUT_S, \
        "Primary estimator instance changed {:.2f} s after the fault, expected < {} s".format(
            handover_s, HANDOVER_TIMEOUT_S)
    return status


def enable_bias_fault(tester: AutopilotTesterImuFailover) -> None:
    tester.set_param_int("SYS_FAILURE_EN", 1)
    tester.set_param_float(FAULT_BIAS_PARAM, FAULT_BIAS_M_S2)


def measure_phase(tester: AutopilotTesterImuFailover
                  ) -> Tuple[List[AccelSample], List[AccelSample]]:
    """Let a fault change settle, then record both IMUs over FAULT_MEASURE_S. The ground truth has
    to have been sampled at a sane rate over the same window."""
    tester.sleep_for(FAULT_SETTLE_S)
    tester.reset_ground_truth_altitude_samples()
    imu1, imu2 = tester.record_scaled_imus(FAULT_MEASURE_S)
    truth_samples = tester.ground_truth_altitude_samples()
    assert truth_samples >= FAULT_MEASURE_S * MIN_GROUND_TRUTH_RATE_HZ, \
        "Only {} ground truth samples in {} s".format(truth_samples, FAULT_MEASURE_S)
    return imu1, imu2


@pytest.mark.ekf_failover
def test_vehicle_survives_alternating_imu_faults(
        tester_imu_failover: AutopilotTesterImuFailover) -> None:
    """A hovering vehicle with an accelerometer bias on one IMU, then on the other, with both
    IMUs clean in between and at the end.

    The bias has to land on the IMU the failure names and on no other, and the selector has to
    leave the instance behind that IMU once each time, within HANDOVER_TIMEOUT_S. The vehicle
    has to stay within ALTITUDE_FLOOR_M below and ALTITUDE_CEILING_M above the hover and within
    HOVER_DRIFT_M of it sideways, and be back at the hover altitude once the faults are
    cleared, all measured against the ground truth."""
    tester = tester_imu_failover
    tester.wait_until_ready()
    check_two_estimator_instances(tester)

    tester.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)
    tester.store_home()
    # Necessary for the takeoff altitude to be applied properly.
    tester.sleep_for(1)

    tester.arm()
    tester.takeoff()
    tester.wait_until_hovering()
    tester.wait_until_altitude(TAKEOFF_ALTITUDE_M, 30)
    tester.sleep_for(5)

    tester.request_scaled_imus(IMU_RATE_HZ)
    tester.start_tracking_ground_truth_altitude()

    # Where the selector starts. Both instances come up together and the selector takes the
    # first one, so the handovers below are changes rather than coincidences.
    status = selector_status(tester)
    log("primary estimator instance before the faults {}".format(status["primary_instance"]))
    assert status["primary_instance"] == 0

    enable_bias_fault(tester)

    # Bias the IMU behind instance 0.
    fault_time_s = tester.autopilot_time_s()
    tester.inject_failure(FailureUnit.SENSOR_ACCEL, FAULT_TYPE, 1)
    imu1, imu2 = measure_phase(tester)
    check_bias_on(1, imu1, imu2)
    status = check_handover(tester, status, fault_time_s, 1)

    # Clear it and let the instance behind IMU 1 come back before the next fault. EKF2 holds an
    # instance unhealthy for BADACC_PROBATION after its accelerometer was bad, and the selector
    # does not switch to an unhealthy instance, so a fault moved straight over would leave the
    # vehicle on the biased instance until that has run out.
    tester.inject_failure(FailureUnit.SENSOR_ACCEL, FailureType.OK, 1)
    imu1, imu2 = measure_phase(tester)
    check_no_bias(imu1, imu2)

    # With both IMUs clean the selector has no reason to move, and the next handover needs it
    # on the instance behind IMU 2.
    recovered = selector_status(tester)
    assert recovered["primary_instance"] == 1 \
        and recovered["instance_changed_count"] == status["instance_changed_count"], \
        "Primary estimator instance {} after {} changes with the fault cleared, expected 1 " \
        "after {}".format(recovered["primary_instance"], recovered["instance_changed_count"],
                          status["instance_changed_count"])

    # Bias the other IMU. The bias has to follow the failure instance, which is what says the
    # injection is per instance rather than shared.
    fault_time_s = tester.autopilot_time_s()
    tester.inject_failure(FailureUnit.SENSOR_ACCEL, FAULT_TYPE, 2)
    imu1, imu2 = measure_phase(tester)
    check_bias_on(2, imu1, imu2)
    check_handover(tester, status, fault_time_s, 0)

    # Clear the faults on every IMU, and both have to read the same again.
    tester.inject_failure(FailureUnit.SENSOR_ACCEL, FailureType.OK, 0)
    imu1, imu2 = measure_phase(tester)
    check_no_bias(imu1, imu2)
    # Either instance is a legitimate choice once both are clean, so this is logged only.
    log("primary estimator instance after the faults cleared {}".format(
        tester.primary_estimator_instance()))

    drift_m = tester.ground_truth_horizontal_drift_m()
    lowest_m, highest_m, recovered_m = tester.stop_tracking_ground_truth_altitude()
    log("ground truth relative to the hover, lowest {:.2f} m, highest {:.2f} m, farthest "
        "sideways {:.2f} m, altitude after recovery {:.2f} m".format(
            lowest_m, highest_m, drift_m, recovered_m))

    assert highest_m < ALTITUDE_CEILING_M, \
        "Climbed {:.2f} m above the hover, expected < {} m".format(highest_m, ALTITUDE_CEILING_M)
    assert lowest_m > -ALTITUDE_FLOOR_M, \
        "Dropped {:.2f} m below the hover, expected < {} m".format(-lowest_m, ALTITUDE_FLOOR_M)
    assert drift_m < HOVER_DRIFT_M, \
        "Moved {:.2f} m sideways from the hover, expected < {} m".format(drift_m, HOVER_DRIFT_M)
    assert abs(recovered_m) < RECOVERY_TOLERANCE_M, \
        "{:.2f} m from the hover after recovery, expected < {} m".format(
            recovered_m, RECOVERY_TOLERANCE_M)

    tester.land()
    tester.wait_until_disarmed(90)


@pytest.mark.ekf_failover
def test_estimator_failover_during_mission(
        tester_imu_failover: AutopilotTesterImuFailover) -> None:
    """An accelerometer bias on the IMU behind the primary estimator instance mid mission, with a
    second IMU and a second instance to fall back on.

    The bias has to be on that IMU, the selector has to leave the instance behind it within
    HANDOVER_TIMEOUT_S while the mission is still running and stay off it, and the mission has to
    run to the end. The vehicle has to stay on the mission legs and hold its altitude, measured
    against the ground truth."""
    tester = tester_imu_failover
    tester.wait_until_ready()
    check_two_estimator_instances(tester)
    tester.store_home()

    tester.prepare_square_mission(MissionOptions(leg_length_m=MISSION_LEG_M,
                                                 relative_altitude_m=MISSION_ALTITUDE_M))
    tester.start_tracking_mission_ground_truth(MISSION_CORRIDOR_M)
    tester.request_scaled_imus(IMU_RATE_HZ)

    tester.arm()

    # Inject once the vehicle flies the first leg, so that the fault starts inside the mission.
    tester.start_mission_and_wait_for_item(1, MISSION_FIRST_LEG_TIMEOUT_S)
    assert tester.mission_is_running(), "Mission finished before the fault"

    tester.start_tracking_ground_truth_altitude()

    # Where the selector starts, so that the handover below is a change rather than a
    # coincidence.
    status = selector_status(tester)
    log("primary estimator instance before the fault {}".format(status["primary_instance"]))
    assert status["primary_instance"] == 0

    enable_bias_fault(tester)
    fault_start_s = tester.autopilot_time_s()
    tester.inject_failure(FailureUnit.SENSOR_ACCEL, FAULT_TYPE, 1)

    # The bias has to show on the IMU while the mission is still running. Without this the test
    # would pass when the injection silently did nothing.
    imu1, imu2 = tester.record_scaled_imus(MISSION_BIAS_CHECK_S)
    check_bias_on(1, imu1, imu2)

    # The recording outlasts HANDOVER_TIMEOUT_S, so by now the selector has to have left the
    # instance behind the faulty IMU, with the mission still running. That is the handover the
    # rest of the mission then rides through.
    status = check_handover(tester, status, fault_start_s, 1)
    assert tester.mission_is_running(), "Mission finished before the handover"

    tester.wait_for_mission_finished(MISSION_FINISH_TIMEOUT_S)
    fault_window_s = tester.autopilot_time_s() - fault_start_s

    # The fault is still on, so the selector has to have stayed off the instance behind it.
    final_status = selector_status(tester)
    assert final_status["primary_instance"] == 1 \
        and final_status["instance_changed_count"] == status["instance_changed_count"], \
        "Primary estimator instance {} after {} changes by the end of the mission, expected 1 " \
        "after {}".format(final_status["primary_instance"], final_status["instance_changed_count"],
                          status["instance_changed_count"])

    # Clear the fault before flying home. Landing on a faulty accelerometer is a different test,
    # and leaving it on would quietly make the disarm timeout load bearing.
    tester.inject_failure(FailureUnit.SENSOR_ACCEL, FailureType.OK, 0)

    lowest_m, highest_m, _ = tester.stop_tracking_ground_truth_altitude()
    truth_samples = tester.ground_truth_altitude_samples()
    worst_m = max(-lowest_m, highest_m)
    log("ground truth samples through the fault {} over {:.1f} s, worst altitude deviation "
        "{:.2f} m".format(truth_samples, fault_window_s, worst_m))

    # A deviation of zero because the stream stalled would read as a pass, so the window has to
    # have been sampled at a sane rate first.
    assert truth_samples >= fault_window_s * MIN_GROUND_TRUTH_RATE_HZ, \
        "Only {} ground truth samples in {:.1f} s".format(truth_samples, fault_window_s)
    assert worst_m < MISSION_ALTITUDE_BAND_M, \
        "Altitude deviated {:.2f} m, expected < {} m".format(worst_m, MISSION_ALTITUDE_BAND_M)

    tester.stop_tracking_mission_ground_truth()

    tester.execute_rtl()
    tester.wait_until_disarmed(180)
