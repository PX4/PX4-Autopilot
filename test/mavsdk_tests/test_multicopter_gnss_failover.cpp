/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

// Failover between the two simulated receivers of the [gnss_failover] entry in configs/sih-sitl.json. Receiver 0 (S,
// failure instance 1) is selected, receiver 1 (B, failure instance 2) is the standby and biased against S, so the
// switch shows as an estimator reset by the difference of the biases. The baro is the height reference except where
// the height reset is under test, so that each case grades one reset.

#include "autopilot_tester_gnss.h"

#include <chrono>

using namespace std::chrono_literals;

namespace
{

constexpr float FLIGHT_ALTITUDE_M = 10.f;

// Failure instances, 1-based
constexpr int SELECTED = 1;
constexpr int STANDBY = 2;
constexpr int ALL = 0;

// Receivers, 0-based
constexpr int RECEIVER_S = 0;
constexpr int RECEIVER_B = 1;

// The selector replaces a silent receiver at its 2 s timeout and one without usable samples after 2 s; the reset
// follows with the estimator delay
constexpr auto SWITCH_TIMEOUT = 6s;

// After a switch the hold setpoint follows the reset, so the vehicle stays where it was
constexpr float HELD_HORIZONTAL_M = 1.f;
constexpr float HELD_VERTICAL_M = 0.7f;

// Without a primary, the selection moves to a receiver one ranking level higher after this long
constexpr auto RANKING_HOLD_ARMED = 10s;
constexpr auto RANKING_HOLD_DISARMED = 2s;

// While armed, a receiver the selection left because it failed is not picked again unless the current one fails;
// this is how long the tests watch for a return that must not happen
constexpr auto NO_RETURN_WATCH = 30s;

void prepare(AutopilotTesterGnss &tester, int preferred_receiver)
{
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_preferred_receiver(preferred_receiver);
	tester.set_height_source(AutopilotTester::HeightSource::Baro);
}

void finish(AutopilotTesterGnss &tester)
{
	tester.clear(ALL);
	tester.execute_rtl();
	tester.wait_until_disarmed(180s);
}

GnssFailover::Injection wrong_injection(int instance, const GnssFailover::WrongPayload &payload)
{
	GnssFailover::Injection injection{};
	injection.type = mavsdk::Failure::FailureType::Wrong;
	injection.instance = instance;
	injection.wrong = payload;
	return injection;
}

// Selected receiver fails while hovering in Hold: one switch to the standby, one reset by the receiver offset, and
// the vehicle keeps a valid position and its mode
void check_failover_in_hold(AutopilotTesterGnss &tester, const GnssFailover::Injection &injection)
{
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();
	tester.inject(injection);
	tester.wait_for_resets(1, SWITCH_TIMEOUT);
	tester.sleep_for(10s);

	CHECK(tester.resets_since_mark() == 1);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);
	tester.check_position_and_mode_kept(mavsdk::Telemetry::FlightMode::Hold);
}

} // namespace

TEST_CASE("GNSS failover - selected receiver off in Hold", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);

	GnssFailover::Injection injection{};
	injection.type = mavsdk::Failure::FailureType::Off;
	injection.instance = SELECTED;
	check_failover_in_hold(tester, injection);

	// The hold setpoint follows the reset, so the vehicle doesn't move
	tester.check_held_horizontally(HELD_HORIZONTAL_M);
	tester.check_held_vertically(HELD_VERTICAL_M);

	finish(tester);
}

TEST_CASE("GNSS failover - selected receiver loses its 3D fix in Hold", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);

	GnssFailover::WrongPayload payload{};
	payload.fix_type = 2;
	check_failover_in_hold(tester, wrong_injection(SELECTED, payload));
	finish(tester);
}

TEST_CASE("GNSS failover - selected receiver off on a mission leg", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.start_mission_leg(20., FLIGHT_ALTITUDE_M);
	tester.sleep_for(8s); // on the second leg, at cruise speed

	tester.mark();
	tester.inject_off(SELECTED);
	tester.wait_for_resets(1, SWITCH_TIMEOUT);
	tester.sleep_for(5s);

	CHECK(tester.resets_since_mark() == 1);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);
	tester.check_position_and_mode_kept(mavsdk::Telemetry::FlightMode::Mission);

	tester.wait_for_mission_finished(120s);
	CHECK(tester.resets_since_mark() == 1);

	finish(tester);
}

TEST_CASE("GNSS failover - height reset with GNSS as the height reference", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);
	tester.set_height_source(AutopilotTester::HeightSource::Gps);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();
	tester.inject_off(SELECTED);
	tester.wait_for_resets(1, SWITCH_TIMEOUT);
	tester.sleep_for(10s);

	// Horizontal and vertical reset, and the altitude setpoint follows the vertical one
	CHECK(tester.resets_since_mark() == 2);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, true);
	tester.check_held_vertically(HELD_VERTICAL_M);
	tester.check_position_and_mode_kept(mavsdk::Telemetry::FlightMode::Hold);

	finish(tester);
}

TEST_CASE("GNSS failover - return to the primary receiver on disarm", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();
	tester.inject_off(SELECTED);
	tester.wait_for_resets(1, SWITCH_TIMEOUT);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);
	tester.sleep_for(8s);

	// S recovers, but the selection stays on B while armed
	tester.clear(SELECTED);
	tester.mark();
	tester.sleep_for(NO_RETURN_WATCH);
	CHECK(tester.resets_since_mark() == 0);
	tester.check_position_and_mode_kept(mavsdk::Telemetry::FlightMode::Hold);

	// Disarmed, the primary is selected again as soon as it publishes. The mark goes first, since the reset can come
	// before the disarm shows on MAVLink.
	tester.mark();
	tester.execute_rtl();
	tester.wait_until_disarmed(180s);
	tester.wait_for_resets(1, 5s);
	tester.sleep_for(2s);
	CHECK(tester.resets_since_mark() == 1);
	tester.check_switch_reset(RECEIVER_B, RECEIVER_S, false);
	tester.clear(ALL);
}

TEST_CASE("GNSS failover - no return without a preferred receiver", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, -1);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();
	tester.inject_off(SELECTED);
	tester.wait_for_resets(1, SWITCH_TIMEOUT);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);
	tester.sleep_for(8s);

	// S recovers at the same ranking level as B, which keeps the selection
	tester.clear(SELECTED);
	tester.mark();
	tester.sleep_for(NO_RETURN_WATCH);
	CHECK(tester.resets_since_mark() == 0);
	tester.check_position_and_mode_kept(mavsdk::Telemetry::FlightMode::Hold);

	finish(tester);
}

TEST_CASE("GNSS failover - standby receiver off", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);

	// Two receivers required, landing when one is lost
	tester.set_param_int("SYS_HAS_NUM_GNSS", 2);
	tester.set_param_int("COM_GNSSLOSS_ACT", 2);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();
	tester.inject_off(STANDBY);
	tester.wait_for_mode_other_than(mavsdk::Telemetry::FlightMode::Hold, 10s);
	CHECK(tester.flight_mode() == mavsdk::Telemetry::FlightMode::Land);

	// The estimate stays on S throughout the landing
	tester.wait_until_disarmed(120s);
	CHECK(tester.resets_since_mark() == 0);
	CHECK(tester.gnss().position_ok());
	tester.clear(ALL);
}

TEST_CASE("GNSS failover - total loss and recovery on the standby", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();
	tester.inject_off(ALL);

	// Position goes invalid after EKF2_NOAID_TOUT, and the failsafe takes the vehicle out of Hold
	tester.wait_for_position_ok(false, 15s);
	tester.wait_for_mode_other_than(mavsdk::Telemetry::FlightMode::Hold, 10s);

	// B comes back alone, S stays off: fusion resumes on B
	tester.clear(STANDBY);
	tester.wait_for_position_ok(true, 20s);

	// Commander accepts Return once its own position checks pass again
	tester.sleep_for(5s);
	finish(tester);
}

TEST_CASE("GNSS failover - selected receiver toggling at 1 Hz", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();

	// S never stays silent for the 2 s timeout, but its availability drops by the margin within a few seconds
	const int64_t start_us = tester.gnss().vehicle_time_us();
	constexpr int64_t TOGGLE_DURATION_US = 20'000'000;
	bool off = false;

	while (tester.gnss().vehicle_time_us() - start_us < TOGGLE_DURATION_US) {
		off = !off;
		tester.inject_without_ack(off ? mavsdk::Failure::FailureType::Off : mavsdk::Failure::FailureType::Ok, SELECTED);
		tester.gnss().sleep(0.5);
	}

	tester.clear(SELECTED);

	// One switch to B, not on the first dropout, and no return while S kept failing
	CHECK(tester.resets_since_mark() == 1);
	const std::vector<GnssFailover::Reset> resets = tester.gnss().resets();
	REQUIRE(!resets.empty());
	CHECK(resets.back().vehicle_time_us - start_us > 2'000'000);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);

	// S stopped failing, but B hasn't failed: no return while armed
	tester.mark();
	tester.sleep_for(NO_RETURN_WATCH);
	CHECK(tester.resets_since_mark() == 0);

	finish(tester);
}

TEST_CASE("GNSS failover - selected receiver off without a preferred receiver", "[gnss_failover]")
{
	// Without a preference the first receiver to publish, S, is selected
	AutopilotTesterGnss tester;
	prepare(tester, -1);

	GnssFailover::Injection injection{};
	injection.type = mavsdk::Failure::FailureType::Off;
	injection.instance = SELECTED;
	check_failover_in_hold(tester, injection);
	finish(tester);
}

TEST_CASE("GNSS failover - selected receiver horizontal accuracy above the relaxed gate", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);

	GnssFailover::WrongPayload payload{};
	payload.fix_type = 0; // unchanged
	payload.eph = 60.f;   // relaxed in-flight gate: 50 m
	check_failover_in_hold(tester, wrong_injection(SELECTED, payload));
	finish(tester);
}

TEST_CASE("GNSS failover - selected receiver speed accuracy above the relaxed gate", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);

	GnssFailover::WrongPayload payload{};
	payload.fix_type = 0;           // unchanged
	payload.speed_accuracy = 12.f;  // relaxed in-flight gate: 10 m/s
	check_failover_in_hold(tester, wrong_injection(SELECTED, payload));
	finish(tester);
}

TEST_CASE("GNSS failover - ranking moves to a receiver that meets the requirements in flight", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, -1);
	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	// S stays usable (relaxed in-flight gate 50 m) but no longer meets GNSS_REQ_EPH; B does
	float required_eph = 0.f;
	REQUIRE(tester.get_param_float("GNSS_REQ_EPH", required_eph));
	GnssFailover::WrongPayload payload{};
	payload.fix_type = 0; // unchanged
	payload.eph = 3.f * required_eph;

	tester.mark();
	tester.inject(wrong_injection(SELECTED, payload));
	tester.sleep_for(RANKING_HOLD_ARMED - 2s);
	CHECK(tester.resets_since_mark() == 0);
	tester.wait_for_resets(1, 6s);
	tester.sleep_for(5s);
	CHECK(tester.resets_since_mark() == 1);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);
	tester.check_position_and_mode_kept(mavsdk::Telemetry::FlightMode::Hold);

	finish(tester);
}

TEST_CASE("GNSS failover - ranking moves to an RTK fixed receiver while disarmed", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, -1);
	tester.sleep_for(5s);

	// Both meet the requirements, B also reports an RTK fixed solution
	GnssFailover::WrongPayload payload{};
	payload.fix_type = 6;

	tester.mark();
	tester.inject(wrong_injection(STANDBY, payload));
	tester.wait_for_resets(1, RANKING_HOLD_DISARMED + 4s);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);

	// The selection stays on B while it is the only RTK fixed receiver
	tester.mark();
	tester.sleep_for(10s);
	CHECK(tester.resets_since_mark() == 0);
	tester.clear(STANDBY);
}

TEST_CASE("GNSS failover - selected receiver update rate collapses while disarmed", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, -1);
	tester.sleep_for(5s);

	// One sample in four arrives later than three times the usual interval, so S has no usable sample and fails;
	// the update rate doesn't rank
	GnssFailover::Injection injection{};
	injection.type = mavsdk::Failure::FailureType::Slow;
	injection.instance = SELECTED;
	injection.slow_divider = 4;

	tester.mark();
	tester.inject(injection);
	tester.wait_for_resets(1, SWITCH_TIMEOUT);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);
	tester.clear(SELECTED);
}

namespace
{

// S is the heading source and fails with its position: position fails over to B, GNSS yaw stops without a yaw reset
void check_heading_source_off(AutopilotTesterGnss &tester)
{
	// GNSS yaw on top of position, velocity and height
	tester.set_param_int("EKF2_GPS_CTRL", 15);
	tester.sleep_for(10s); // heading settle and yaw alignment

	tester.takeoff_and_hold(FLIGHT_ALTITUDE_M);

	tester.mark();
	tester.inject_off(SELECTED);
	tester.wait_for_resets(1, SWITCH_TIMEOUT);
	tester.sleep_for(10s);

	CHECK(tester.resets_since_mark() == 1);
	tester.check_switch_reset(RECEIVER_S, RECEIVER_B, false);
	tester.check_no_yaw_reset_since_mark();
	tester.check_position_and_mode_kept(mavsdk::Telemetry::FlightMode::Hold);
}

} // namespace

TEST_CASE("GNSS failover - dual antenna heading receiver off", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);

	// S has a second antenna 0.5 m to the right of its main one
	tester.set_param_int("SENS_GNSS0_HDG", 2);
	tester.set_param_float("SENS_GNSS0_AUXX", 0.2f);
	tester.set_param_float("SENS_GNSS0_AUXY", 0.5f);

	check_heading_source_off(tester);
	finish(tester);
}

TEST_CASE("GNSS failover - moving base off", "[gnss_failover]")
{
	AutopilotTesterGnss tester;
	prepare(tester, RECEIVER_S);

	// B is the rover of a moving base pair with S as its moving base: S's silence takes B's heading with it
	tester.set_param_int("SENS_GNSS1_HDG", 1);

	check_heading_source_off(tester);
	finish(tester);
}

TEST_CASE("GNSS failover - dual antenna heading receiver recovers", "[gnss_failover]")
{
	// Without a preference the position stays on B when S recovers, while the heading returns with S
	AutopilotTesterGnss tester;
	prepare(tester, -1);
	tester.set_param_int("SENS_GNSS0_HDG", 2);
	tester.set_param_float("SENS_GNSS0_AUXX", 0.2f);
	tester.set_param_float("SENS_GNSS0_AUXY", 0.5f);

	check_heading_source_off(tester);

	tester.clear(SELECTED);
	tester.mark();
	tester.sleep_for(30s);
	CHECK(tester.resets_since_mark() == 0);

	finish(tester);
}
