"""Failover between the two simulated receivers of the gnss_failover entry in configs/sih-sitl.json.

Receiver 0 (S, failure instance 1) is selected, receiver 1 (B, failure instance 2) is the standby
and biased against S, so the switch shows as an estimator reset by the difference of the biases.
The baro is the height reference except where the height reset is under test, so that each case
grades one reset.
"""

import pytest
from mavsdk.plugins.failure import FailureType  # type: ignore
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from autopilot_tester import HeightSource
from autopilot_tester_gnss import AutopilotTesterGnss, LogSource
from gnss_failover_scenario import Injection, WrongPayload

FLIGHT_ALTITUDE_M = 10.0

# Failure instances, 1-based
SELECTED = 1
STANDBY = 2
ALL = 0

# Receivers, 0-based
RECEIVER_S = 0
RECEIVER_B = 1

# The selector replaces a silent receiver at its 2 s timeout, and one without usable samples after
# 2 s. The reset follows with the estimator delay.
SWITCH_TIMEOUT_S = 6

# After a switch the hold setpoint follows the reset, so the vehicle stays where it was
HELD_HORIZONTAL_M = 1.0
HELD_VERTICAL_M = 0.7

# Without a primary, the selection moves to a receiver one ranking level higher after this long
RANKING_HOLD_ARMED_S = 10
RANKING_HOLD_DISARMED_S = 2

# While armed, a receiver the selection left because it failed is not picked again unless the
# current one fails. The tests watch this long for a return that must not happen.
NO_RETURN_WATCH_S = 30


def prepare(tester: AutopilotTesterGnss, preferred_receiver: int) -> None:
    tester.wait_until_ready()
    tester.set_preferred_receiver(preferred_receiver)
    tester.set_height_source(HeightSource.BARO)


def finish(tester: AutopilotTesterGnss) -> None:
    tester.clear(ALL)
    tester.execute_rtl()
    tester.wait_until_disarmed(180)


def finish_and_check_reporting(tester: AutopilotTesterGnss) -> None:
    """Lands, then grades why GNSS wasn't fused until the switch and the receivers' inconsistency
    in the log."""
    finish(tester)
    tester.check_log("reporting")


def wrong_injection(instance: int, payload: WrongPayload) -> Injection:
    return Injection(failure_type=FailureType.WRONG, instance=instance, wrong=payload)


def check_failover_in_hold(tester: AutopilotTesterGnss, injection: Injection) -> None:
    """Selected receiver fails while hovering in Hold: one switch to the standby, one reset by the
    receiver offset, and the vehicle keeps a valid position and its mode."""
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()
    tester.inject(injection)
    tester.wait_for_resets(1, SWITCH_TIMEOUT_S)
    tester.sleep_for(10)

    tester.check_resets_since_mark(1)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)
    tester.check_position_and_mode_kept(FlightMode.HOLD)


@pytest.mark.gnss_failover
def test_selected_receiver_off_in_hold(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    check_failover_in_hold(tester, Injection(failure_type=FailureType.OFF, instance=SELECTED))

    # The hold setpoint follows the reset, so the vehicle doesn't move
    tester.check_held_horizontally(HELD_HORIZONTAL_M)
    tester.check_held_vertically(HELD_VERTICAL_M)

    finish_and_check_reporting(tester)


@pytest.mark.gnss_failover
def test_selected_receiver_loses_3d_fix_in_hold(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    check_failover_in_hold(tester, wrong_injection(SELECTED, WrongPayload(fix_type=2)))
    finish_and_check_reporting(tester)


@pytest.mark.gnss_failover
def test_selected_receiver_off_on_mission_leg(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.start_mission_leg(20.0, FLIGHT_ALTITUDE_M)
    tester.sleep_for(8)  # on the second leg, at cruise speed

    tester.mark()
    tester.inject_off(SELECTED)
    tester.wait_for_resets(1, SWITCH_TIMEOUT_S)
    tester.sleep_for(5)

    tester.check_resets_since_mark(1)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)
    tester.check_position_and_mode_kept(FlightMode.MISSION)

    tester.wait_for_mission_finished(120)
    tester.check_resets_since_mark(1)

    finish_and_check_reporting(tester)


@pytest.mark.gnss_failover
def test_height_reset_with_gnss_height_reference(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    tester.set_height_source(HeightSource.GPS)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()
    tester.inject_off(SELECTED)
    tester.wait_for_resets(1, SWITCH_TIMEOUT_S)
    tester.sleep_for(10)

    # Horizontal and vertical reset, and the altitude setpoint follows the vertical one
    tester.check_resets_since_mark(2)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, True)
    tester.check_held_vertically(HELD_VERTICAL_M)
    tester.check_position_and_mode_kept(FlightMode.HOLD)

    finish_and_check_reporting(tester)


@pytest.mark.gnss_failover
def test_return_to_primary_receiver_on_disarm(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()
    tester.inject_off(SELECTED)
    tester.wait_for_resets(1, SWITCH_TIMEOUT_S)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)
    tester.sleep_for(8)

    # S recovers, but the selection stays on B while armed
    tester.clear(SELECTED)
    tester.mark()
    tester.sleep_for(NO_RETURN_WATCH_S)
    tester.check_resets_since_mark(0)
    tester.check_position_and_mode_kept(FlightMode.HOLD)

    # Disarmed, the primary is selected again as soon as it publishes. The mark goes first, since
    # the reset can come before the disarm shows on MAVLink.
    tester.mark()
    tester.execute_rtl()
    tester.wait_until_disarmed(180)
    tester.wait_for_resets(1, 5)
    tester.sleep_for(2)
    tester.check_resets_since_mark(1)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_B, RECEIVER_S, False)
    tester.clear(ALL)
    tester.check_log("reporting")


@pytest.mark.gnss_failover
def test_no_return_without_preferred_receiver(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, -1)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()
    tester.inject_off(SELECTED)
    tester.wait_for_resets(1, SWITCH_TIMEOUT_S)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)
    tester.sleep_for(8)

    # S recovers at the same ranking level as B, which keeps the selection
    tester.clear(SELECTED)
    tester.mark()
    tester.sleep_for(NO_RETURN_WATCH_S)
    tester.check_resets_since_mark(0)
    tester.check_position_and_mode_kept(FlightMode.HOLD)

    finish(tester)


@pytest.mark.gnss_failover
def test_standby_receiver_off(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)

    # Two receivers required, landing when one is lost
    tester.set_param_int("SYS_HAS_NUM_GNSS", 2)
    tester.set_param_int("COM_GNSSLOSS_ACT", 2)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()
    tester.inject_off(STANDBY)
    tester.wait_for_mode_other_than(FlightMode.HOLD, 10)
    mode = tester.flight_mode()
    tester.check(mode == FlightMode.LAND, "Mode {}, expected LAND".format(FlightMode(mode).name))

    # The estimate stays on S throughout the landing
    tester.wait_until_disarmed(120)
    tester.check_resets_since_mark(0)
    tester.check_switch_events(0)
    tester.check(tester.gnss().position_ok(), "Position not valid after landing")
    tester.clear(ALL)

    # The other cases read the log from the SITL rootfs. This short flight keeps the companion
    # tool's download covered.
    tester.check_log("selection,reset", LogSource.DOWNLOAD)


@pytest.mark.gnss_failover
def test_total_loss_and_recovery_on_standby(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()
    tester.inject_off(ALL)

    # Position goes invalid after EKF2_NOAID_TOUT, and the failsafe takes the vehicle out of Hold
    tester.wait_for_position_ok(False, 15)
    tester.wait_for_mode_other_than(FlightMode.HOLD, 10)

    # B comes back alone, S stays off: fusion resumes on B
    tester.clear(STANDBY)
    tester.wait_for_position_ok(True, 20)

    # Commander accepts Return once its own position checks pass again
    tester.sleep_for(5)
    finish(tester)


@pytest.mark.gnss_failover
def test_selected_receiver_toggling_at_1_hz(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()

    # S never stays silent for the 2 s timeout, but its availability drops by the margin within a
    # few seconds
    start_us = tester.gnss().vehicle_time_us()
    toggle_duration_us = 20_000_000
    off = False

    while tester.gnss().vehicle_time_us() - start_us < toggle_duration_us:
        off = not off
        tester.inject_without_ack(FailureType.OFF if off else FailureType.OK, SELECTED)
        tester.gnss().sleep(0.5)

    tester.clear(SELECTED)

    # One switch to B, not on the first dropout, and no return while S kept failing
    tester.check_resets_since_mark(1)
    resets = tester.gnss().resets()
    assert resets, "No estimator reset"
    switch_after_us = resets[-1].vehicle_time_us - start_us
    tester.check(switch_after_us > 2_000_000,
                 "Switch {:.2f} s after the first dropout".format(switch_after_us * 1e-6))
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)

    # S stopped failing, but B hasn't failed: no return while armed
    tester.mark()
    tester.sleep_for(NO_RETURN_WATCH_S)
    tester.check_resets_since_mark(0)
    tester.check_switch_events(0)

    finish(tester)


@pytest.mark.gnss_failover
def test_selected_receiver_off_without_preferred_receiver(
        tester_gnss: AutopilotTesterGnss) -> None:
    # Without a preference the first receiver to publish, S, is selected
    tester = tester_gnss
    prepare(tester, -1)
    check_failover_in_hold(tester, Injection(failure_type=FailureType.OFF, instance=SELECTED))
    finish_and_check_reporting(tester)


@pytest.mark.gnss_failover
def test_selected_receiver_eph_above_relaxed_gate(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)

    # Fix type unchanged, above the relaxed in-flight gate of 50 m
    check_failover_in_hold(tester, wrong_injection(SELECTED, WrongPayload(fix_type=0, eph=60.0)))
    finish_and_check_reporting(tester)


@pytest.mark.gnss_failover
def test_selected_receiver_speed_accuracy_above_relaxed_gate(
        tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)

    # Fix type unchanged, above the relaxed in-flight gate of 10 m/s
    check_failover_in_hold(
        tester, wrong_injection(SELECTED, WrongPayload(fix_type=0, speed_accuracy=12.0)))
    finish_and_check_reporting(tester)


@pytest.mark.gnss_failover
def test_ranking_moves_to_receiver_meeting_requirements_in_flight(
        tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, -1)
    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    # S stays within the relaxed in-flight gate of 50 m, but no longer meets GNSS_REQ_EPH. B does.
    required_eph = tester.get_param_float("GNSS_REQ_EPH")
    payload = WrongPayload(fix_type=0, eph=3.0 * required_eph)

    tester.mark()
    tester.inject(wrong_injection(SELECTED, payload))
    tester.sleep_for(RANKING_HOLD_ARMED_S - 2)
    tester.check_resets_since_mark(0)
    tester.wait_for_resets(1, 6)
    tester.sleep_for(5)
    tester.check_resets_since_mark(1)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)
    tester.check_position_and_mode_kept(FlightMode.HOLD)

    finish(tester)


@pytest.mark.gnss_failover
def test_ranking_moves_to_rtk_fixed_receiver_while_disarmed(
        tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, -1)
    tester.sleep_for(5)

    # Both meet the requirements, B also reports an RTK fixed solution
    tester.mark()
    tester.inject(wrong_injection(STANDBY, WrongPayload(fix_type=6)))
    tester.wait_for_resets(1, RANKING_HOLD_DISARMED_S + 4)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)

    # The selection stays on B while it is the only RTK fixed receiver
    tester.mark()
    tester.sleep_for(10)
    tester.check_resets_since_mark(0)
    tester.clear(STANDBY)


@pytest.mark.gnss_failover
def test_selected_receiver_update_rate_collapses_while_disarmed(
        tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, -1)
    tester.sleep_for(5)

    # One sample in ten of the 8 Hz simulated receiver. The selection takes a rate above about 1 Hz
    # as the receiver's own, but below it S has no usable sample for the fail time and fails. The
    # update rate doesn't rank.
    tester.mark()
    tester.inject(Injection(failure_type=FailureType.SLOW, instance=SELECTED, slow_divider=10))
    tester.wait_for_resets(1, SWITCH_TIMEOUT_S)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)
    tester.clear(SELECTED)


def set_dual_antenna_heading_on_s(tester: AutopilotTesterGnss) -> None:
    # S has a second antenna 0.5 m to the right of its main one
    tester.set_param_int("SENS_GNSS0_HDG", 2)
    tester.set_param_float("SENS_GNSS0_AUXX", 0.2)
    tester.set_param_float("SENS_GNSS0_AUXY", 0.5)


def check_heading_source_off(tester: AutopilotTesterGnss) -> None:
    """S is the heading source and fails with its position: position fails over to B, GNSS yaw
    stops without a yaw reset."""
    # GNSS yaw on top of position, velocity and height
    tester.set_param_int("EKF2_GPS_CTRL", 15)
    tester.sleep_for(10)  # heading settle and yaw alignment

    tester.takeoff_and_hold(FLIGHT_ALTITUDE_M)

    tester.mark()
    tester.inject_off(SELECTED)
    tester.wait_for_resets(1, SWITCH_TIMEOUT_S)
    tester.sleep_for(10)

    tester.check_resets_since_mark(1)
    tester.check_switch_events(1)
    tester.check_switch_reset(RECEIVER_S, RECEIVER_B, False)
    tester.check_no_yaw_reset_since_mark()
    tester.check_position_and_mode_kept(FlightMode.HOLD)


@pytest.mark.gnss_failover
def test_dual_antenna_heading_receiver_off(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)
    set_dual_antenna_heading_on_s(tester)

    check_heading_source_off(tester)
    finish(tester)
    tester.check_log("heading,reporting")


@pytest.mark.gnss_failover
def test_moving_base_off(tester_gnss: AutopilotTesterGnss) -> None:
    tester = tester_gnss
    prepare(tester, RECEIVER_S)

    # B is the rover of a moving base pair with S as its moving base: S's silence takes B's
    # heading with it
    tester.set_param_int("SENS_GNSS1_HDG", 1)

    check_heading_source_off(tester)
    finish(tester)
    tester.check_log("heading,reporting")


@pytest.mark.gnss_failover
def test_dual_antenna_heading_receiver_recovers(tester_gnss: AutopilotTesterGnss) -> None:
    # Without a preference the position stays on B when S recovers, while the heading returns with
    # S
    tester = tester_gnss
    prepare(tester, -1)
    set_dual_antenna_heading_on_s(tester)

    check_heading_source_off(tester)

    tester.clear(SELECTED)
    tester.mark()
    tester.sleep_for(30)
    tester.check_resets_since_mark(0)

    finish(tester)
    tester.check_log("heading,reporting")
