/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "autopilot_tester.h"
#include "math_helpers.h"
#include <iostream>
#include <future>
#include <thread>
#include <unistd.h>
#include <cmath>

std::string connection_url {"udp://"};
std::optional<float> speed_factor {std::nullopt};

AutopilotTester::AutopilotTester() :
	_real_time_report_thread([this]()
{
	report_speed_factor();
})
{
}

AutopilotTester::~AutopilotTester()
{
	_should_exit = true;
	_real_time_report_thread.join();
}

void AutopilotTester::connect(const std::string uri)
{
	ConnectionResult ret = _mavsdk.add_any_connection(uri);
	REQUIRE(ret == ConnectionResult::Success);

	std::cout << time_str() << "Waiting for system connect" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mavsdk.systems().size() > 0; }, std::chrono::seconds(25)));

	auto system = get_system();

	_action.reset(new Action(system));
	_failure.reset(new Failure(system));
	_info.reset(new Info(system));
	_manual_control.reset(new ManualControl(system));
	_mission.reset(new Mission(system));
	_mission_raw.reset(new MissionRaw(system));
	_offboard.reset(new Offboard(system));
	_param.reset(new Param(system));
	_telemetry.reset(new Telemetry(system));
	_mavlink_passthrough.reset(new MavlinkPassthrough(system));
}

void AutopilotTester::wait_until_ready()
{
	std::cout << time_str() << "Waiting for system to be ready (system health ok & able to arm)" << std::endl;

	// Wiat until the system is healthy
	CHECK(poll_condition_with_timeout(
	[this]() { return _telemetry->health_all_ok(); }, std::chrono::seconds(30)));

	// Note: There is a known bug in MAVSDK (https://github.com/mavlink/MAVSDK/issues/1852),
	// where `health_all_ok()` returning true doesn't actually mean vehicle is ready to accept
	// global position estimate as valid (due to hysteresis). This needs to be fixed properly.

	// However, this is mitigated by the `is_armable` check below as a side effect, since
	// when the vehicle considers global position to be valid, it will then allow arming

	// Wait until we can arm
	CHECK(poll_condition_with_timeout(
	[this]() {	return _telemetry->health().is_armable;	}, std::chrono::seconds(45)));
}

void AutopilotTester::store_home()
{
	request_ground_truth();
	std::cout << time_str() << "Waiting to get home position" << std::endl;
	CHECK(poll_condition_with_timeout(
	[this]() {
		_home = _telemetry->ground_truth();
		return std::isfinite(_home.latitude_deg) && std::isfinite(_home.longitude_deg);
	}, std::chrono::seconds(10)));
}

void AutopilotTester::check_home_within(float acceptance_radius_m)
{
	CHECK(ground_truth_horizontal_position_close_to(_home, acceptance_radius_m));
}

void AutopilotTester::check_home_not_within(float min_distance_m)
{
	CHECK(ground_truth_horizontal_position_far_from(_home, min_distance_m));
}

void AutopilotTester::set_takeoff_altitude(const float altitude_m)
{
	CHECK(Action::Result::Success == _action->set_takeoff_altitude(altitude_m));
	const auto result = _action->get_takeoff_altitude();
	CHECK(result.first == Action::Result::Success);
	CHECK(result.second == Approx(altitude_m));
}

void AutopilotTester::set_rtl_altitude(const float altitude_m)
{
	CHECK(Action::Result::Success == _action->set_return_to_launch_altitude(altitude_m));
	const auto result = _action->get_return_to_launch_altitude();
	CHECK(result.first == Action::Result::Success);
	CHECK(result.second == Approx(altitude_m));
}

void AutopilotTester::set_height_source(AutopilotTester::HeightSource height_source)
{
	switch (height_source) {
	case HeightSource::Baro:
		CHECK(_param->set_param_int("EKF2_HGT_REF", 0) == Param::Result::Success);
		break;

	case HeightSource::Gps:
		CHECK(_param->set_param_int("EKF2_HGT_REF", 1) == Param::Result::Success);
	}
}

void AutopilotTester::set_rc_loss_exception(AutopilotTester::RcLossException mask)
{
	switch (mask) {
	case RcLossException::Mission:
		CHECK(_param->set_param_int("COM_RCL_EXCEPT", 1 << 0) == Param::Result::Success);
		break;

	case RcLossException::Hold:
		CHECK(_param->set_param_int("COM_RCL_EXCEPT", 1 << 1) == Param::Result::Success);
		break;

	case RcLossException::Offboard:
		CHECK(_param->set_param_int("COM_RCL_EXCEPT", 1 << 2) == Param::Result::Success);
	}
}

void AutopilotTester::set_param_vt_fwd_thrust_en(int value)
{
	CHECK(_param->set_param_int("VT_FWD_THRUST_EN", value) == Param::Result::Success);
}

void AutopilotTester::arm()
{
	const auto result = _action->arm();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::takeoff()
{
	const auto result = _action->takeoff();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::land()
{
	const auto result = _action->land();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::transition_to_fixedwing()
{
	const auto result = _action->transition_to_fixedwing();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::transition_to_multicopter()
{
	const auto result = _action->transition_to_multicopter();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::wait_until_disarmed(std::chrono::seconds timeout_duration)
{
	REQUIRE(poll_condition_with_timeout(
	[this]() { return !_telemetry->armed(); }, timeout_duration));
}

void AutopilotTester::wait_until_hovering()
{
	wait_for_landed_state(Telemetry::LandedState::InAir, std::chrono::seconds(45));
}

void AutopilotTester::wait_until_altitude(float rel_altitude_m, std::chrono::seconds timeout, float delta)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_telemetry->subscribe_position([&prom, rel_altitude_m, delta, this](Telemetry::Position new_position) {
		if (fabs(rel_altitude_m - new_position.relative_altitude_m) <= delta) {
			_telemetry->subscribe_position(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}

void AutopilotTester::wait_until_fixedwing(std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _telemetry->vtol_state() == Telemetry::VtolState::Fw; }, timeout));
}

void AutopilotTester::prepare_square_mission(MissionOptions mission_options)
{
	const auto ct = get_coordinate_transformation();

	Mission::MissionPlan mission_plan {};
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0.}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, mission_options.leg_length_m},
					     mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({0., mission_options.leg_length_m}, mission_options, ct));

	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	REQUIRE(_mission->upload_mission(mission_plan) == Mission::Result::Success);
}

void AutopilotTester::prepare_straight_mission(MissionOptions mission_options)
{
	const auto ct = get_coordinate_transformation();

	Mission::MissionPlan mission_plan {};
	mission_plan.mission_items.push_back(create_mission_item({0, 0.}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({2 * mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({3 * mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({4 * mission_options.leg_length_m, 0}, mission_options, ct));

	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	REQUIRE(_mission->upload_mission(mission_plan) == Mission::Result::Success);
}

void AutopilotTester::execute_mission()
{
	std::promise<void> prom;
	auto fut = prom.get_future();


	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mission->start_mission() == Mission::Result::Success; }, std::chrono::seconds(3)));

	float speed_factor = 1.0f;

	if (_info != nullptr) {
		speed_factor = _info->get_speed_factor().second;
	}

	const float mission_finish_waiting_time_in_simulation_s = 500.f;
	float mission_finish_waiting_time_in_real_s = mission_finish_waiting_time_in_simulation_s / speed_factor;

	wait_for_mission_finished(std::chrono::seconds(static_cast<int>(mission_finish_waiting_time_in_real_s)));
}

void AutopilotTester::execute_mission_and_lose_gps()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorGps, Failure::FailureType::Off, 0) == Failure::Result::Success);

	// We expect that a blind land is performed.
	wait_for_flight_mode(Telemetry::FlightMode::Land, std::chrono::seconds(30));
}

void AutopilotTester::execute_mission_and_lose_mag()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorMag, Failure::FailureType::Off, 0) == Failure::Result::Success);

	// We except the mission to continue without mag just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto progress = _mission->mission_progress();
		return progress.current == progress.total;
	}, std::chrono::seconds(90)));
}

void AutopilotTester::execute_mission_and_lose_baro()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorBaro, Failure::FailureType::Off, 0) == Failure::Result::Success);

	// We except the mission to continue without baro just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto progress = _mission->mission_progress();
		return progress.current == progress.total;
	}, std::chrono::seconds(90)));
}

void AutopilotTester::execute_mission_and_get_baro_stuck()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorBaro, Failure::FailureType::Stuck, 0) == Failure::Result::Success);

	// We except the mission to continue with a stuck baro just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto progress = _mission->mission_progress();
		return progress.current == progress.total;
	}, std::chrono::seconds(90)));
}

void AutopilotTester::execute_mission_and_get_mag_stuck()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorMag, Failure::FailureType::Stuck, 0) == Failure::Result::Success);

	// We except the mission to continue with a stuck mag just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto progress = _mission->mission_progress();
		return progress.current == progress.total;
	}, std::chrono::seconds(120)));
}

CoordinateTransformation AutopilotTester::get_coordinate_transformation()
{
	const auto home = _telemetry->home();
	CHECK(std::isfinite(home.latitude_deg));
	CHECK(std::isfinite(home.longitude_deg));
	return CoordinateTransformation({home.latitude_deg, home.longitude_deg});
}

Mission::MissionItem  AutopilotTester::create_mission_item(
	const CoordinateTransformation::LocalCoordinate &local_coordinate,
	const MissionOptions &mission_options,
	const CoordinateTransformation &ct)
{
	auto mission_item = Mission::MissionItem{};
	const auto pos_north = ct.global_from_local(local_coordinate);
	mission_item.latitude_deg = pos_north.latitude_deg;
	mission_item.longitude_deg = pos_north.longitude_deg;
	mission_item.relative_altitude_m = mission_options.relative_altitude_m;
	mission_item.is_fly_through = mission_options.fly_through;
	return mission_item;
}

void AutopilotTester::load_qgc_mission_raw_and_move_here(const std::string &plan_file)
{
	auto import_result = _mission_raw->import_qgroundcontrol_mission(plan_file);
	REQUIRE(import_result.first == MissionRaw::Result::Success);

	move_mission_raw_here(import_result.second.mission_items);

	REQUIRE(_mission_raw->upload_mission(import_result.second.mission_items) == MissionRaw::Result::Success);
}

void AutopilotTester::execute_mission_raw()
{
	REQUIRE(_mission->start_mission() == Mission::Result::Success);

	float speed_factor = 1.0f;

	if (_info != nullptr) {
		speed_factor = _info->get_speed_factor().second;
	}

	const float waiting_time_simulation_time_s = 300.f; // currently this is tuned for the VTOL wind test
	float waiting_time_absolute_s = waiting_time_simulation_time_s / speed_factor;

	wait_for_mission_raw_finished(std::chrono::seconds(static_cast<int>(waiting_time_absolute_s)));
}

void AutopilotTester::execute_rtl()
{
	REQUIRE(Action::Result::Success == _action->return_to_launch());
}

void AutopilotTester::execute_land()
{
	REQUIRE(Action::Result::Success == _action->land());
}

void AutopilotTester::offboard_goto(const Offboard::PositionNedYaw &target, float acceptance_radius_m,
				    std::chrono::seconds timeout_duration)
{
	_offboard->set_position_ned(target);
	REQUIRE(_offboard->start() == Offboard::Result::Success);
	CHECK(poll_condition_with_timeout(
	[ = ]() { return estimated_position_close_to(target, acceptance_radius_m); }, timeout_duration));
	std::cout << time_str() << "Target position reached" << std::endl;
}

void AutopilotTester::check_mission_item_speed_above(int item_index, float min_speed_m_s)
{

	_telemetry->set_rate_velocity_ned(10);
	_telemetry->subscribe_velocity_ned([item_index, min_speed_m_s, this](Telemetry::VelocityNed velocity) {
		float horizontal = std::hypot(velocity.north_m_s, velocity.east_m_s);
		auto progress = _mission->mission_progress();

		if (progress.current == item_index) {
			CHECK(horizontal > min_speed_m_s);
		}
	});
}

void AutopilotTester::fly_forward_in_posctl()
{
	const unsigned manual_control_rate_hz = 50;

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	CHECK(_manual_control->start_position_control() == ManualControl::Result::Success);

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	store_home();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	wait_until_ready();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	arm();

	// Climb up for 5 seconds
	for (unsigned i = 0; i < 5 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 1.f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Fly forward for 10 seconds
	for (unsigned i = 0; i < 10 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.5f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Descend until disarmed
	for (unsigned i = 0; i < 60 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.0f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));

		if (!_telemetry->in_air()) {
			break;
		}
	}
}

void AutopilotTester::fly_forward_in_altctl()
{
	const unsigned manual_control_rate_hz = 50;

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	CHECK(_manual_control->start_altitude_control() == ManualControl::Result::Success);

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	store_home();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	wait_until_ready();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	arm();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Climb up for 5 seconds
	for (unsigned i = 0; i < 5 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 1.f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Fly forward for 10 seconds
	for (unsigned i = 0; i < 10 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.5f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Descend until disarmed
	for (unsigned i = 0; i < 60 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.0f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));

		if (!_telemetry->in_air()) {
			break;
		}
	}
}
void AutopilotTester::fly_forward_in_offboard_attitude()
{
	// This test does not depend on valid position estimate.
	// Wait for raw gps & stable attitude estimate
	CHECK(poll_condition_with_timeout(
	[this]() {
		auto attitude = _telemetry->attitude_euler();
		return _telemetry->raw_gps().altitude_ellipsoid_m > 0.f && fabsf(attitude.roll_deg) < 5.f
		       && fabsf(attitude.pitch_deg) < 5.f;
	}, std::chrono::seconds(20)));

	const float start_altitude_ellipsoid_m = _telemetry->raw_gps().altitude_ellipsoid_m;

	Offboard::Attitude attitude{};
	_offboard->set_attitude(attitude);
	REQUIRE(_offboard->start() == Offboard::Result::Success);

	// Wait until we can arm
	CHECK(poll_condition_with_timeout(
	[this]() {	return _telemetry->health().is_armable;	}, std::chrono::seconds(20)));
	arm();

	const unsigned offboard_rate_hz = 50;

	// Climb
	const float climb_altitude_m = 10.f;
	attitude.thrust_value = 0.8f;

	while (_telemetry->raw_gps().altitude_ellipsoid_m - start_altitude_ellipsoid_m < climb_altitude_m) {
		CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / offboard_rate_hz));
	}

	// Fly forward for 3s
	attitude.thrust_value = 0.8f;
	attitude.pitch_deg = -20.f;

	for (unsigned i = 0; i < 3 * offboard_rate_hz; ++i) {
		CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / offboard_rate_hz));
	}

	// Check attitude
	auto attitude_estimate = _telemetry->attitude_euler();
	CHECK(fabsf(attitude.roll_deg - attitude_estimate.roll_deg) < 5.f);
	CHECK(fabsf(attitude.pitch_deg - attitude_estimate.pitch_deg) < 5.f);

	// Descend
	attitude.thrust_value = 0.4f;
	attitude.pitch_deg = 0.f;

	for (unsigned i = 0; i < 6 * offboard_rate_hz; ++i) {
		CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / offboard_rate_hz));
	}

	attitude.thrust_value = 0.0f;
	CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
}

void AutopilotTester::start_checking_altitude(const float max_deviation_m)
{
	std::array<float, 3> initial_position = get_current_position_ned();
	float target_altitude = initial_position[2];

	_telemetry->subscribe_position([target_altitude, max_deviation_m, this](Telemetry::Position new_position) {
		const float current_deviation = fabs((-target_altitude) - new_position.relative_altitude_m);
		CHECK(current_deviation <= max_deviation_m);
	});
}

void AutopilotTester::stop_checking_altitude()
{
	_telemetry->subscribe_position(nullptr);
}

void AutopilotTester::check_tracks_mission_raw(float corridor_radius_m, bool reverse)
{
	auto mission_raw = _mission_raw->download_mission();
	CHECK(mission_raw.first == MissionRaw::Result::Success);

	auto mission_items = mission_raw.second;
	auto ct = get_coordinate_transformation();

	_telemetry->set_rate_position_velocity_ned(5);
	_telemetry->subscribe_position_velocity_ned([ct, mission_items, corridor_radius_m, reverse,
	    this](Telemetry::PositionVelocityNed position_velocity_ned) {
		auto progress = _mission_raw->mission_progress();


		std::function<std::array<float, 3>(std::vector<mavsdk::MissionRaw::MissionItem>, unsigned, mavsdk::geometry::CoordinateTransformation)>
		get_waypoint_for_sequence = [](std::vector<mavsdk::MissionRaw::MissionItem> mission_items, int sequence, auto ct) {
			for (auto waypoint : mission_items) {

				if (waypoint.seq == (uint32_t)sequence) {
					return get_local_mission_item_from_raw_item<float>(waypoint, ct);
				}
			}

			return  std::array<float, 3>({0.0f, 0.0f, 0.0f});
		};

		if (progress.current > 0 && progress.current < progress.total) {
			// Get shortest distance of current position to 3D line between previous and next waypoint

			std::array<float, 3> current { position_velocity_ned.position.north_m,
						       position_velocity_ned.position.east_m,
						       position_velocity_ned.position.down_m };
			std::array<float, 3> wp_prev = get_waypoint_for_sequence(mission_items,
						       reverse ? progress.current + 1 : progress.current - 1, ct);
			std::array<float, 3> wp_next = get_waypoint_for_sequence(mission_items, progress.current, ct);

			float distance_to_trajectory = point_to_line_distance(current, wp_prev, wp_next);

			CHECK(distance_to_trajectory < corridor_radius_m);
		}
	});
}

void AutopilotTester::check_mission_land_within(float acceptance_radius_m)
{
	auto mission_raw = _mission_raw->download_mission();
	CHECK(mission_raw.first == MissionRaw::Result::Success);

	// Get last mission item
	MissionRaw::MissionItem land_mission_item = mission_raw.second.back();
	bool is_landing_item = (land_mission_item.command == 85) || (land_mission_item.command == 21);
	CHECK(is_landing_item);
	Telemetry::GroundTruth land_coord{};
	land_coord.latitude_deg = static_cast<double>(land_mission_item.x) / 1E7;
	land_coord.longitude_deg = static_cast<double>(land_mission_item.y) / 1E7;

	CHECK(ground_truth_horizontal_position_close_to(land_coord, acceptance_radius_m));
}

void AutopilotTester::check_tracks_mission(float corridor_radius_m)
{
	auto mission = _mission->download_mission();
	CHECK(mission.first == Mission::Result::Success);

	std::vector<Mission::MissionItem> mission_items = mission.second.mission_items;
	auto ct = get_coordinate_transformation();

	_telemetry->set_rate_position_velocity_ned(5);
	_telemetry->subscribe_position_velocity_ned([ct, mission_items, corridor_radius_m,
	    this](Telemetry::PositionVelocityNed position_velocity_ned) {
		auto progress = _mission->mission_progress();

		if (progress.current > 0 && progress.current < progress.total) {
			// Get shortest distance of current position to 3D line between previous and next waypoint

			std::array<float, 3> current { position_velocity_ned.position.north_m,
						       position_velocity_ned.position.east_m,
						       position_velocity_ned.position.down_m };
			std::array<float, 3> wp_prev = get_local_mission_item<float>(mission_items[progress.current - 1], ct);
			std::array<float, 3> wp_next = get_local_mission_item<float>(mission_items[progress.current], ct);

			float distance_to_trajectory = point_to_line_distance(current, wp_prev, wp_next);

			CHECK(distance_to_trajectory < corridor_radius_m);
		}
	});
}

void AutopilotTester::check_current_altitude(float target_rel_altitude_m, float max_distance_m)
{
	CHECK(std::abs(_telemetry->position().relative_altitude_m - target_rel_altitude_m) <= max_distance_m);
}

void AutopilotTester::execute_rtl_when_reaching_mission_sequence(int sequence_number)
{
	start_and_wait_for_mission_sequence_raw(sequence_number);
	execute_rtl();
}

void AutopilotTester::send_custom_mavlink_command(const MavlinkPassthrough::CommandInt &command)
{
	_mavlink_passthrough->send_command_int(command);
}

void AutopilotTester::send_custom_mavlink_message(mavlink_message_t &message)
{
	_mavlink_passthrough->send_message(message);
}

void AutopilotTester::add_mavlink_message_callback(uint16_t message_id,
		std::function< void(const mavlink_message_t &)> callback)
{
	_mavlink_passthrough->subscribe_message_async(message_id, std::move(callback));
}

std::array<float, 3> AutopilotTester::get_current_position_ned()
{
	mavsdk::Telemetry::PositionVelocityNed position_velocity_ned = _telemetry->position_velocity_ned();
	std::array<float, 3> position_ned{position_velocity_ned.position.north_m, position_velocity_ned.position.east_m, position_velocity_ned.position.down_m};
	return position_ned;
}

void AutopilotTester::offboard_land()
{
	Offboard::VelocityNedYaw land_velocity;
	land_velocity.north_m_s = 0.0f;
	land_velocity.east_m_s = 0.0f;
	land_velocity.down_m_s = 1.0f;
	land_velocity.yaw_deg = 0.0f;
	_offboard->set_velocity_ned(land_velocity);
}

bool AutopilotTester::estimated_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m)
{
	Telemetry::PositionNed est_pos = _telemetry->position_velocity_ned().position;
	const float distance_m = std::sqrt(sq(est_pos.north_m - target_pos.north_m) +
					   sq(est_pos.east_m - target_pos.east_m) +
					   sq(est_pos.down_m - target_pos.down_m));
	const bool pass = distance_m < acceptance_radius_m;

	if (!pass) {
		std::cout << time_str() << "distance: " << distance_m << ", " << "acceptance: " << acceptance_radius_m << std::endl;
	}

	return  pass;
}

bool AutopilotTester::estimated_horizontal_position_close_to(const Offboard::PositionNedYaw &target_pos,
		float acceptance_radius_m)
{
	Telemetry::PositionNed est_pos = _telemetry->position_velocity_ned().position;
	return sq(est_pos.north_m - target_pos.north_m) +
	       sq(est_pos.east_m - target_pos.east_m) < sq(acceptance_radius_m);
}

void AutopilotTester::request_ground_truth()
{
	CHECK(_telemetry->set_rate_ground_truth(15) == Telemetry::Result::Success);
}

bool AutopilotTester::ground_truth_horizontal_position_close_to(const Telemetry::GroundTruth &target_pos,
		float acceptance_radius_m)
{
	CHECK(std::isfinite(target_pos.latitude_deg));
	CHECK(std::isfinite(target_pos.longitude_deg));
	using GlobalCoordinate = CoordinateTransformation::GlobalCoordinate;
	using LocalCoordinate = CoordinateTransformation::LocalCoordinate;
	CoordinateTransformation ct(GlobalCoordinate{target_pos.latitude_deg, target_pos.longitude_deg});

	Telemetry::GroundTruth current_pos = _telemetry->ground_truth();
	CHECK(std::isfinite(current_pos.latitude_deg));
	CHECK(std::isfinite(current_pos.longitude_deg));
	GlobalCoordinate global_current;
	global_current.latitude_deg = current_pos.latitude_deg;
	global_current.longitude_deg = current_pos.longitude_deg;
	LocalCoordinate local_pos = ct.local_from_global(global_current);
	const double distance_m = sqrt(sq(local_pos.north_m) + sq(local_pos.east_m));
	const bool pass = distance_m < acceptance_radius_m;

	if (!pass) {
		std::cout << time_str() << "target_pos.lat: " << target_pos.latitude_deg << std::endl;
		std::cout << time_str() << "target_pos.lon: " << target_pos.longitude_deg << std::endl;
		std::cout << time_str() << "current.lat: " << current_pos.latitude_deg << std::endl;
		std::cout << time_str() << "current.lon: " << current_pos.longitude_deg << std::endl;
		std::cout << time_str() << "Distance: " << distance_m << std::endl;
		std::cout << time_str() << "Acceptance radius: " << acceptance_radius_m << std::endl;
	}

	return pass;
}

bool AutopilotTester::ground_truth_horizontal_position_far_from(const Telemetry::GroundTruth &target_pos,
		float min_distance_m)
{
	CHECK(std::isfinite(target_pos.latitude_deg));
	CHECK(std::isfinite(target_pos.longitude_deg));
	using GlobalCoordinate = CoordinateTransformation::GlobalCoordinate;
	using LocalCoordinate = CoordinateTransformation::LocalCoordinate;
	CoordinateTransformation ct(GlobalCoordinate{target_pos.latitude_deg, target_pos.longitude_deg});

	Telemetry::GroundTruth current_pos = _telemetry->ground_truth();
	CHECK(std::isfinite(current_pos.latitude_deg));
	CHECK(std::isfinite(current_pos.longitude_deg));
	GlobalCoordinate global_current;
	global_current.latitude_deg = current_pos.latitude_deg;
	global_current.longitude_deg = current_pos.longitude_deg;
	LocalCoordinate local_pos = ct.local_from_global(global_current);
	const double distance_m = sqrt(sq(local_pos.north_m) + sq(local_pos.east_m));
	const bool pass = distance_m > min_distance_m;

	if (!pass) {
		std::cout << time_str() << "target_pos.lat: " << target_pos.latitude_deg << std::endl;
		std::cout << time_str() << "target_pos.lon: " << target_pos.longitude_deg << std::endl;
		std::cout << time_str() << "current.lat: " << current_pos.latitude_deg << std::endl;
		std::cout << time_str() << "current.lon: " << current_pos.longitude_deg << std::endl;
		std::cout << time_str() << "Distance: " << distance_m << std::endl;
		std::cout << time_str() << "Min distance: " << min_distance_m << std::endl;
	}

	return pass;
}

void AutopilotTester::start_and_wait_for_mission_sequence(int sequence_number)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_mission->subscribe_mission_progress([&prom, this, sequence_number](Mission::MissionProgress progress) {
		std::cout << time_str() << "Progress: " << progress.current << "/" << progress.total << std::endl;

		if (progress.current >= sequence_number) {
			_mission->subscribe_mission_progress(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(_mission->start_mission() == Mission::Result::Success);

	REQUIRE(fut.wait_for(std::chrono::seconds(60)) == std::future_status::ready);
}

void AutopilotTester::start_and_wait_for_mission_sequence_raw(int sequence_number)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_mission_raw->subscribe_mission_progress([&prom, this, sequence_number](MissionRaw::MissionProgress progress) {
		std::cout << time_str() << "Progress: " << progress.current << "/" << progress.total << std::endl;

		if (progress.current >= sequence_number) {
			_mission_raw->subscribe_mission_progress(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(_mission_raw->start_mission() == MissionRaw::Result::Success);

	REQUIRE(fut.wait_for(std::chrono::seconds(60)) == std::future_status::ready);
}

void AutopilotTester::wait_for_flight_mode(Telemetry::FlightMode flight_mode, std::chrono::seconds timeout)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_telemetry->subscribe_flight_mode([&prom, flight_mode, this](Telemetry::FlightMode new_flight_mode) {
		if (new_flight_mode == flight_mode) {
			_telemetry->subscribe_flight_mode(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}

void AutopilotTester::wait_for_landed_state(Telemetry::LandedState landed_state, std::chrono::seconds timeout)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_telemetry->subscribe_landed_state([&prom, landed_state, this](Telemetry::LandedState new_landed_state) {
		if (new_landed_state == landed_state) {
			_telemetry->subscribe_landed_state(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}

void AutopilotTester::wait_until_speed_lower_than(float speed, std::chrono::seconds timeout)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_telemetry->subscribe_position_velocity_ned([&prom, speed, this](Telemetry::PositionVelocityNed position_velocity_ned) {
		std::array<float, 3> current_velocity;
		current_velocity[0] = position_velocity_ned.velocity.north_m_s;
		current_velocity[1] = position_velocity_ned.velocity.east_m_s;
		current_velocity[2] = position_velocity_ned.velocity.down_m_s;
		const float current_speed = norm(current_velocity);

		if (current_speed <= speed) {
			_telemetry->subscribe_position_velocity_ned(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}

void AutopilotTester::wait_for_mission_finished(std::chrono::seconds timeout)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_mission->subscribe_mission_progress([&prom, this](Mission::MissionProgress progress) {
		if (progress.current == progress.total) {
			_mission->subscribe_mission_progress(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}

void AutopilotTester::wait_for_mission_raw_finished(std::chrono::seconds timeout)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();

	_mission_raw->subscribe_mission_progress([&prom, this](MissionRaw::MissionProgress progress) {
		if (progress.current == progress.total) {
			_mission_raw->subscribe_mission_progress(nullptr);
			prom.set_value();
		}
	});

	REQUIRE(fut.wait_for(timeout) == std::future_status::ready);
}

void AutopilotTester::move_mission_raw_here(std::vector<MissionRaw::MissionItem> &mission_items)
{
	const auto position = _telemetry->position();
	REQUIRE(std::isfinite(position.latitude_deg));
	REQUIRE(std::isfinite(position.longitude_deg));

	auto offset_x = mission_items[0].x - static_cast<int32_t>(1e7 * position.latitude_deg);
	auto offset_y = mission_items[0].y - static_cast<int32_t>(1e7 * position.longitude_deg);

	for (auto &item : mission_items) {
		if (item.frame == 3) { // MAV_FRAME_GLOBAL_RELATIVE_ALT
			item.x -= offset_x;
		}

		item.y -= offset_y;
	}
}

void AutopilotTester::report_speed_factor()
{
	// We check the exit flag more often than the speed factor.
	unsigned counter = 0;

	while (!_should_exit) {
		if (counter++ % 10 == 0) {
			if (_info != nullptr) {
				std::cout << "Current speed factor: " << _info->get_speed_factor().second ;

				if (speed_factor.has_value()) {
					std::cout << " (set: " << speed_factor.value() << ')';
				}

				std::cout << std::endl;
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void AutopilotTester::enable_fixedwing_mectrics()
{
	CHECK(getTelemetry()->set_rate_fixedwing_metrics(10.f) == Telemetry::Result::Success);
}

void AutopilotTester::check_airspeed_is_valid()
{
	// If the airspeed was invalidated during the flight, the airspeed is sent in the
	// telemetry is NAN and stays so with the default parameter settings.
	const Telemetry::FixedwingMetrics &metrics = getTelemetry()->fixedwing_metrics();
	REQUIRE(std::isfinite(metrics.airspeed_m_s));
}

void AutopilotTester::check_airspeed_is_invalid()
{
	// If the airspeed was invalidated during the flight, the airspeed is sent in the
	// telemetry is NAN and stays so with the default parameter settings.
	const Telemetry::FixedwingMetrics &metrics = getTelemetry()->fixedwing_metrics();
	std::cout << "Reported airspeed after failure: " << metrics.airspeed_m_s ;
	REQUIRE(!std::isfinite(metrics.airspeed_m_s));
}
