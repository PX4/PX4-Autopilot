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
	const float x_dist_to_target = target_pos_ned[0] - drone_pos_ned[0];
	const float y_dist_to_target = target_pos_ned[1] - drone_pos_ned[1];

	switch (config.follow_direction) {
	case FollowMe::Config::FollowDirection::None:
		CHECK(x_dist_to_target < tolerance);
		CHECK(x_dist_to_target > -tolerance);
		CHECK(y_dist_to_target < tolerance);
		CHECK(y_dist_to_target > -tolerance);
		break;

	case FollowMe::Config::FollowDirection::Behind:
		CHECK(drone_pos_ned[0] < target_pos_ned[0]);
		CHECK(y_dist_to_target < tolerance);
		CHECK(y_dist_to_target > -tolerance);
		break;

	case FollowMe::Config::FollowDirection::Front:
		CHECK(drone_pos_ned[0] > target_pos_ned[0]);
		CHECK(y_dist_to_target < tolerance);
		CHECK(y_dist_to_target > -tolerance);
		break;

	case FollowMe::Config::FollowDirection::FrontRight:
		CHECK(drone_pos_ned[0] > target_pos_ned[0]);
		CHECK(drone_pos_ned[1] > target_pos_ned[1]);
		break;

	case FollowMe::Config::FollowDirection::FrontLeft:
		CHECK(drone_pos_ned[0] > target_pos_ned[0]);
		CHECK(drone_pos_ned[1] < target_pos_ned[1]);
		break;

	default:
		break;
	}
}

AutopilotTesterFollowMe::AutopilotTesterFollowMe() : AutopilotTester()
{

}

AutopilotTesterFollowMe::~AutopilotTesterFollowMe()
{

}

void AutopilotTesterFollowMe::connect(const std::string uri)
{
	AutopilotTester::connect(uri);

	auto system = get_system();
	_follow_me.reset(new FollowMe(system));
}


void AutopilotTesterFollowMe::straight_line_test(const float altitude_m, const bool stream_velocity)
{
	const unsigned location_update_rate = 1;
	const float position_tolerance = 4.0f;

	// Start with simulated target on the same plane as drone's home position
	std::array<float, 3> start_location_ned = get_current_position_ned();
	FollowTargetSimulator target_simulator(start_location_ned, getHome());

	// Configure Min height of the drone to be "20 meters" above home & Follow direction as "Front
	// right".
	FollowMe::Config config;
	config.min_height_m = altitude_m;
	config.follow_distance_m = 8.0f;

	// Allow some time for mode switch
	sleep_for(std::chrono::milliseconds(1000));

	// Start Follow Me
	CHECK(FollowMe::Result::Success ==  _follow_me->start());

	// Allow some time for mode switch
	sleep_for(std::chrono::milliseconds(1000));

	// task loop
	bool target_moving = false;
	bool perform_checks = false;

	for (unsigned i = 0; i < 75 * location_update_rate; ++i) {
		std::array<float, 3> target_pos_ned_ground_truth = target_simulator.get_position_ground_truth_ned();
		std::array<float, 3> position_ned = get_current_position_ned();
		const float distance_to_target = norm(diff(target_pos_ned_ground_truth, position_ned));

		// poor-man's state machine
		if (i < 5) {
			// Stream target location without moving

		} else if (i == 5) {
			// Change config
			perform_checks = false;
			config.follow_direction = FollowMe::Config::FollowDirection::Behind;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (i > 5 && i < 15) {
			// Move target and wait for steady state of drone
			target_moving = true;

		} else if (i >= 15 && i < 20) {
			// Perform positional checks in steady state
			perform_checks = true;

		} else if (i == 20) {
			// Change config
			perform_checks = false;
			config.follow_direction = FollowMe::Config::FollowDirection::Front;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (i > 20 && i < 30) {
			// Move target and wait for steady state of drone

		} else if (i >= 30 && i < 35) {
			// Perform positional checks in steady state
			perform_checks = true;

		} else if (i == 35) {
			// Change config
			perform_checks = false;
			config.follow_direction = FollowMe::Config::FollowDirection::FrontRight;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (i > 35 && i < 45) {
			// Move target and wait for steady state of drone


		} else if (i >= 45 && i < 55) {
			// Perform positional checks in steady state
			perform_checks = true;

		} else if (i == 55) {
			// Change config
			perform_checks = false;
			config.follow_direction = FollowMe::Config::FollowDirection::FrontLeft;
			CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

		} else if (i > 55 && i < 65) {
			// Move target and wait for steady state of drone

		} else if (i >= 65 && i < 75) {
			// Perform positional checks in steady state
			perform_checks = true;
		}

		if (target_moving) {
			target_simulator.update(1.0f / location_update_rate);
		}

		if (perform_checks) {
			check_current_altitude(altitude_m);
			CHECK(distance_to_target <= config.follow_distance_m + position_tolerance);
			CHECK(distance_to_target >= config.follow_distance_m - position_tolerance);
			target_simulator.check_follow_angle(config, position_ned, target_pos_ned_ground_truth, position_tolerance);
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
	config.follow_direction = FollowMe::Config::FollowDirection::Behind;
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

void AutopilotTesterFollowMe::rc_override_test(const float altitude_m)
{
	const unsigned loop_update_rate = 50;
	const float position_tolerance = 4.0f;

	// Start with simulated target on the same plane as drone's home position
	std::array<float, 3> start_location_ned = get_current_position_ned();
	FollowTargetSimulator target_simulator(start_location_ned, getHome());

	// Configure Min height of the drone to be "20 meters" above home & Follow direction as "Front
	// right".
	FollowMe::Config config;
	config.min_height_m = altitude_m;
	config.follow_distance_m = 8.0f;
	config.follow_direction = FollowMe::Config::FollowDirection::Behind;
	CHECK(FollowMe::Result::Success == _follow_me->set_config(config));

	// task loop
	std::array<float, 3> drone_initial_pos;

	for (unsigned i = 0; i <= 30 * loop_update_rate; ++i) {
		// Start streaming target data after x seconds to provide RC before switching to the flight task
		bool stream_follow_me_data = (i > 7 * loop_update_rate);

		// Deflect a stick for a short time only
		bool deflect_rc_sticks = (i > 10 * loop_update_rate && i <= 11 * loop_update_rate);

		// Switch to follow-me at this instant
		if (i == 5 * loop_update_rate) {
			// Start Follow Me
			CHECK(FollowMe::Result::Success ==  _follow_me->start());
		}

		// After approximately 10 seconds we would expect the drone to have stopped because of the RC stick input
		if (i == 20 * loop_update_rate) {
			drone_initial_pos = get_current_position_ned();
		}

		if (stream_follow_me_data) {
			target_simulator.update(1.0f / loop_update_rate);
			std::array<double, 3> global_coordinate = target_simulator.get_position_global_noisy();
			FollowMe::TargetLocation target_location{};
			target_location.latitude_deg = global_coordinate[0];
			target_location.longitude_deg = global_coordinate[1];
			target_location.absolute_altitude_m = global_coordinate[2];
			target_location.velocity_x_m_s = NAN;
			target_location.velocity_y_m_s = NAN;
			target_location.velocity_z_m_s = NAN;
			_follow_me->set_target_location(target_location);
		}

		if (deflect_rc_sticks) {
			CHECK(getManualControl()->set_manual_control_input(1.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);

		} else {
			CHECK(getManualControl()->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		}

		sleep_for(std::chrono::milliseconds(1000 / loop_update_rate));
	}

	std::array<float, 3> drone_final_pos = get_current_position_ned();
	const float distance_travelled = norm(diff(drone_initial_pos, drone_final_pos));
	CHECK(distance_travelled < position_tolerance);
}
