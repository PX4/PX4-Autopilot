/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "autopilot_tester_follow_me.h"

// #include <mavsdk/plugins/follow_me/follow_me.h>

#include "math_helpers.h"

#include <iostream>
#include <future>
#include <thread>
#include <unistd.h>
#include <cmath>
#include <random>


FollowTargetSimulator::FollowTargetSimulator(std::array<float, 3> initial_position_ned,
		mavsdk::Telemetry::GroundTruth home) :
	_position_ned(initial_position_ned), _home(home)
{
	_velocity_ned[0] = 0.0f;
	_velocity_ned[1] = 0.0f;
	_velocity_ned[2] = 0.0f;
}

FollowTargetSimulator::~FollowTargetSimulator()
{

}

void FollowTargetSimulator::update(float delta_t_s)
{
	const float velocity_m_s = 2.0;

	_velocity_ned[0] = velocity_m_s;
	_velocity_ned[1] = 0.0;

	_position_ned[0] += _velocity_ned[0] * delta_t_s;
	_position_ned[1] += _velocity_ned[1] * delta_t_s;
	_position_ned[2] += _velocity_ned[2] * delta_t_s;

	_udpate_count++;
}

std::array<double, 3> FollowTargetSimulator::get_position_global(bool add_noise)
{
	std::array<float, 3> pos_ned = _position_ned;

	if (add_noise) {
		unsigned seed = _udpate_count;
		std::default_random_engine generator(seed);
		std::normal_distribution<double> distribution(0.0, 1.0);
		pos_ned[0] += distribution(generator);
		pos_ned[1] += distribution(generator);
		pos_ned[2] += distribution(generator);
	}

	CHECK(std::isfinite(_home.latitude_deg));
	CHECK(std::isfinite(_home.longitude_deg));
	const auto ct = CoordinateTransformation({_home.latitude_deg, _home.longitude_deg});

	mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_coordinate{pos_ned[0], pos_ned[1]};
	mavsdk::geometry::CoordinateTransformation::GlobalCoordinate global_coordinate = ct.global_from_local(local_coordinate);
	std::array<double, 3> global_pos{global_coordinate.latitude_deg, global_coordinate.longitude_deg, pos_ned[2] + _home.absolute_altitude_m};
	return global_pos;
}

std::array<float, 3> FollowTargetSimulator::get_position_ned(bool add_noise)
{

	std::array<float, 3> pos_ned = _position_ned;

	if (add_noise) {
		unsigned seed = _udpate_count;
		std::default_random_engine generator(seed);
		std::normal_distribution<double> distribution(0.0, pos_noise_std);
		pos_ned[0] += distribution(generator);
		pos_ned[1] += distribution(generator);
		pos_ned[2] += distribution(generator);
	}

	return pos_ned;
}

std::array<float, 3> FollowTargetSimulator::get_velocity_ned_noisy()
{
	return get_velocity_ned(true);
}

std::array<float, 3> FollowTargetSimulator::get_velocity_ned_ground_truth()
{
	return get_velocity_ned(false);
}

std::array<float, 3> FollowTargetSimulator::get_velocity_ned(bool add_noise)
{
	std::array<float, 3> vel_ned = _velocity_ned;

	if (add_noise) {
		unsigned seed = _udpate_count;
		std::default_random_engine generator(seed);
		std::normal_distribution<double> distribution(0.0, vel_noise_std);

		vel_ned[0] += distribution(generator);
		vel_ned[1] += distribution(generator);
		vel_ned[2] += distribution(generator);
	}

	return vel_ned;
}

std::array<float, 3> FollowTargetSimulator::get_position_ned_noisy()
{
	return get_position_ned(true);
}

std::array<float, 3> FollowTargetSimulator::get_position_ground_truth_ned()
{
	return get_position_ned(false);
}

std::array<double, 3> FollowTargetSimulator::get_position_global_noisy()
{
	return get_position_global(true);
}

std::array<double, 3> FollowTargetSimulator::get_position_global_ground_truth()
{
	return get_position_global(false);
}

void FollowTargetSimulator::check_follow_angle(FollowMe::Config config, std::array<float, 3> drone_pos_ned,
		std::array<float, 3> target_pos_ned, float tolerance)
{
	// This check assumes that the target is travelling straight on the x-axis
	const float target_to_drone_offset_x = drone_pos_ned[0] - target_pos_ned[0];
	const float target_to_drone_offset_y = drone_pos_ned[1] - target_pos_ned[1];

	// Follow Angle is measured relative from the target's course (direction it is moving towards)
	const float target_to_drone_angle_expected_rad = config.follow_angle_deg * (M_PI / 180.0f);
	const float target_to_drone_offset_x_expected = config.follow_distance_m * cos(target_to_drone_angle_expected_rad);
	const float target_to_drone_offset_y_expected = config.follow_distance_m * sin(target_to_drone_angle_expected_rad);

	// Check that drone is following at an expected position within the tolerance error
	CHECK(fabsf(target_to_drone_offset_x - target_to_drone_offset_x_expected) < tolerance);
	CHECK(fabsf(target_to_drone_offset_y - target_to_drone_offset_y_expected) < tolerance);
}

void AutopilotTesterFollowMe::connect(const std::string uri)
{
	AutopilotTester::connect(uri);

	auto system = get_system();
	_follow_me.reset(new FollowMe(system));
}


void AutopilotTesterFollowMe::straight_line_test(const bool stream_velocity)
{
	// CONFIGURATION for the test
	const unsigned location_update_rate = 1; // [Hz] How often the GPS location update samples are generated
	const float position_error_tolerance = 11.0f; // [m] Position error tolerance in both X and Y direction
	const float follow_me_height_setting = 10.0f; // [m] Height above home position where the Drone will follow from

	// Start with simulated target on the same plane as drone's home position
	std::array<float, 3> start_location_ned = get_current_position_ned();
	FollowTargetSimulator target_simulator(start_location_ned, getHome());

	// Configure the Follow Me parameters
	FollowMe::Config config;
	config.follow_height_m = follow_me_height_setting;
	config.follow_distance_m = 8.0f;
	config.follow_angle_deg = 0.0f;
	// Ensure that tangential velocity control generates fast enough trajectory to be responsive
	config.max_tangential_vel_m_s = 15.0f;

	// Allow some time for mode switch
	sleep_for(std::chrono::milliseconds(1000));

	// Start Follow Me
	CHECK(FollowMe::Result::Success ==  _follow_me->start());

	// Allow some time for mode switch
	sleep_for(std::chrono::milliseconds(1000));

	// task loop
	bool target_moving = false;
	bool perform_checks = false;

	// Simulate generating 75 different target location updates
	for (unsigned location_update_idx = 0; location_update_idx < 75 * location_update_rate; ++location_update_idx) {
		std::array<float, 3> target_pos_ned_ground_truth = target_simulator.get_position_ground_truth_ned();
		std::array<float, 3> position_ned = get_current_position_ned();
		const float distance_to_target = norm(diff(target_pos_ned_ground_truth, position_ned));

		// poor-man's state machine
		if (location_update_idx < 5) {
			// Stream target location without moving

		} else if (location_update_idx == 5) {
			// Change config to Follow from 'Behind'
			perform_checks = false;
			config.follow_angle_deg = 180.0f;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (location_update_idx < 15) {
			// Move target for 10 samples and wait for steady state of drone
			target_moving = true;

		} else if (location_update_idx < 20) {
			// Perform positional checks in steady state for 5 samples
			perform_checks = true;

		} else if (location_update_idx == 20) {
			// Change config to follow from 'Front'
			perform_checks = false;
			config.follow_angle_deg = 0.0f;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (location_update_idx < 30) {
			// Move target for 10 samples and wait for steady state of drone

		} else if (location_update_idx < 35) {
			// Perform positional checks in steady state for 5 samples
			perform_checks = true;

		} else if (location_update_idx == 35) {
			// Change config to follow from 'Front right'
			perform_checks = false;
			config.follow_angle_deg = 45.0f;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (location_update_idx < 45) {
			// Move target for 10 samples and wait for steady state of drone

		} else if (location_update_idx < 55) {
			// Perform positional checks in steady state for 5 samples
			perform_checks = true;

		} else if (location_update_idx == 55) {
			// Change config to follow from 'Front Left'
			perform_checks = false;
			config.follow_angle_deg = -45.0f;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (location_update_idx < 65) {
			// Move target for 10 samples and wait for steady state of drone

		} else if (location_update_idx < 75) {
			// Perform positional checks in steady state for 10 samples
			perform_checks = true;
		}

		if (target_moving) {
			target_simulator.update(1.0f / location_update_rate);
		}

		if (perform_checks) {
			check_current_altitude(follow_me_height_setting);
			CHECK(distance_to_target <= config.follow_distance_m + position_error_tolerance);
			CHECK(distance_to_target >= config.follow_distance_m - position_error_tolerance);
			target_simulator.check_follow_angle(config, position_ned, target_pos_ned_ground_truth, position_error_tolerance);
		}

		// Construct follow-me message
		std::array<double, 3> global_coordinate = target_simulator.get_position_global_noisy();

		FollowMe::TargetLocation target_location{};
		target_location.latitude_deg = global_coordinate[0];
		target_location.longitude_deg = global_coordinate[1];
		target_location.absolute_altitude_m = global_coordinate[2];

		if (stream_velocity) {
			std::array<float, 3> target_vel_ned = target_simulator.get_velocity_ned_noisy();
			target_location.velocity_x_m_s = target_vel_ned[0];
			target_location.velocity_y_m_s = target_vel_ned[1];
			target_location.velocity_z_m_s = target_vel_ned[2];

		} else {
			target_location.velocity_x_m_s = NAN;
			target_location.velocity_y_m_s = NAN;
			target_location.velocity_z_m_s = NAN;
		}


		// Send message and check result
		CHECK(FollowMe::Result::Success == _follow_me->set_target_location(target_location));

		sleep_for(std::chrono::milliseconds(1000 / location_update_rate));
	}

	CHECK(FollowMe::Result::Success ==  _follow_me->stop());
}

void AutopilotTesterFollowMe::stream_velocity_only()
{
	const unsigned loop_update_rate = 1;
	const float position_tolerance = 4.0f;

	// Configure follow-me
	FollowMe::Config config;
	config.follow_angle_deg = 180.0f; // Follow from behind
	CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

	// Allow some time for mode switch
	sleep_for(std::chrono::milliseconds(1000));

	// Start Follow Me
	CHECK(FollowMe::Result::Success ==  _follow_me->start());

	// Allow some time for mode switch
	sleep_for(std::chrono::milliseconds(1000));

	std::array<float, 3> drone_initial_pos = get_current_position_ned();

	// Start streaming velocity only. The drone should not move.
	for (unsigned i = 0; i < 15 * loop_update_rate; i++) {
		FollowMe::TargetLocation target_location{};
		target_location.latitude_deg = NAN;
		target_location.longitude_deg = NAN;
		target_location.absolute_altitude_m = NAN;
		target_location.velocity_x_m_s = 2.0f;
		target_location.velocity_y_m_s = 1.0f;
		target_location.velocity_z_m_s = 0.5f;

		// Send message and check result
		CHECK(FollowMe::Result::Success == _follow_me->set_target_location(target_location));

		sleep_for(std::chrono::milliseconds(1000 / loop_update_rate));
	}

	// Check that drone is still close to initial position and has not moved much
	std::array<float, 3> drone_final_pos = get_current_position_ned();
	const float distance_travelled = norm(diff(drone_initial_pos, drone_final_pos));
	CHECK(distance_travelled < position_tolerance);
}

void AutopilotTesterFollowMe::rc_adjustment_test()
{
	// CONFIGURATION
	const unsigned loop_update_rate = 50; // [Hz]
	const float follow_height_setting = 10.0f; // [m]
	const float follow_angle_setting = 0.0f; // [deg]
	const float follow_distance_setting = 10.0f; // [m]

	// The constants below are copied from the "FlightTaskAutoFollowTarget.hpp" to get a reference point for
	// how much change a RC adjustment is expected to bring for each follow me parameters

	// [m/s] Speed with which the follow distance will be adjusted by when commanded with deflection via RC command
	static constexpr float FOLLOW_DISTANCE_USER_ADJUST_SPEED = 2.0;
	// [m/s] Speed with which the follow height will be adjusted by when commanded with deflection via RC command
	static constexpr float FOLLOW_HEIGHT_USER_ADJUST_SPEED = 1.5;
	// [rad/s] Angular rate with which the follow distance will be adjusted by when commanded with full deflection via RC command
	static constexpr float FOLLOW_ANGLE_USER_ADJUST_SPEED = 1.5;

	// Start with simulated target on the same plane as drone's home position
	std::array<float, 3> start_location_ned = get_current_position_ned();
	FollowTargetSimulator target_simulator(start_location_ned, getHome());

	// Set Follow Me parameters
	FollowMe::Config config;
	config.follow_height_m = follow_height_setting;
	config.follow_distance_m = follow_distance_setting;
	config.follow_angle_deg = follow_angle_setting;
	CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

	// [deg] Get a single sample of target's GPS coordinate
	const std::array<double, 3> target_global_coordinate = target_simulator.get_position_global_ground_truth();

	// Set TargetLocation as the sample to simulate a stationary target for a controlled RC adjustment test
	const FollowMe::TargetLocation target_location{};
	target_location.latitude_deg = target_global_coordinate[0];
	target_location.longitude_deg = target_global_coordinate[1];
	target_location.absolute_altitude_m = target_global_coordinate[2];

	// [m] Save the target's location in Local NED (in meters), assuming it is where drone is at in the beginning
	const std::array<float, 3> target_pos = get_current_position_ned();

	// Start Follow-me
	CHECK(FollowMe::Result::Success ==  _follow_me->start());
	std::array<float, 3> drone_initial_pos;

	// task loop
	for (unsigned i = 0; i <= 60 * loop_update_rate; ++i) {
		_follow_me->set_target_location(target_location);

		std::array<float, 3> current_drone_pos = get_current_position_ned();

		if (i < 5 * loop_update_rate) {
			// For 5 seconds, give time for the drone to go to it's initial following position (front)
			CHECK(getManualControl()->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);

		} else if (i == 5 * loop_update_rate) {
			// At 5 second mark, record the current drone position as initial position
			drone_initial_pos = get_current_position_ned();

		} else if (i < 8 * loop_update_rate) {
			// FOLLOW HEIGHT ADJUSTMENT
			// Command Throttle-up (Z = 1.0f) for 3 seconds
			CHECK(getManualControl()->set_manual_control_input(0.f, 0.f, 1.0f, 0.f) == ManualControl::Result::Success);

		} else if (i < 10 * loop_update_rate) {
			// Center the throttle for 2 seconds
			CHECK(getManualControl()->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);

		} else if (i == 10 * loop_update_rate) {
			// Check if altitude has increased at least half of the expected adjustment (Z is directed downwards, so flip the sign)
			CHECK(-(current_drone_pos[2] - drone_initial_pos[2]) > FOLLOW_HEIGHT_USER_ADJUST_SPEED * 3.0f * 0.5f);

		} else if (i < 13 * loop_update_rate) {
			// FOLLOW HEIGHT ADJUSTMENT
			// Command Pitch-down (= Forward) (X = 1.0f) for 3 seconds
			CHECK(getManualControl()->set_manual_control_input(1.0f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);

		} else if (i < 15 * loop_update_rate) {
			// Center the Pitch for 2 seconds
			CHECK(getManualControl()->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);

		} else if (i == 15 * loop_update_rate) {
			// Check if follow distance has increased at least half of the expected adjustment
			const float target_to_drone_x_diff = current_drone_pos[0] - target_pos[0];
			const float target_to_drone_y_diff = current_drone_pos[1] - target_pos[1];
			const float current_follow_distance = sqrt(sq(target_to_drone_x_diff) + sq(target_to_drone_y_diff));
			CHECK(current_follow_distance > follow_distance_setting + FOLLOW_DISTANCE_USER_ADJUST_SPEED * 3.0f * 0.5f);

		} else if (i < 18 * loop_update_rate) {
			// FOLLOW ANGLE ADJUSTMENT
			// Command Roll-right (=Rightwards) (Y = 1.0f) for 3 seconds
			CHECK(getManualControl()->set_manual_control_input(0.f, 1.0f, 0.5f, 0.f) == ManualControl::Result::Success);

		} else if (i < 20 * loop_update_rate) {
			// Center the Roll for 2 seconds
			CHECK(getManualControl()->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);

		} else if (i == 20 * loop_update_rate) {
			// Check if follow angle has increased at least half of the expected adjustment
			// Since Roll-right corresponds to the follow angle increasing in clockwise direction
			const float target_to_drone_x_diff = current_drone_pos[0] - target_pos[0];
			const float target_to_drone_y_diff = current_drone_pos[1] - target_pos[1];
			const float current_follow_angle_rad = atan2f(target_to_drone_y_diff, target_to_drone_x_diff);
			CHECK(current_follow_angle_rad > follow_angle_setting * M_PI / 180.0f + FOLLOW_ANGLE_USER_ADJUST_SPEED * 3.0f * 0.5f);
		}

		sleep_for(std::chrono::milliseconds(1000 / loop_update_rate));
	}
}
