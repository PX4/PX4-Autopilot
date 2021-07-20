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

#pragma once

#include "autopilot_tester.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/follow_me/follow_me.h>


// Simulated a target moving on a straight line
class FollowTargetSimulator
{
public:
	FollowTargetSimulator(std::array<float, 3> initial_position_ned, mavsdk::Telemetry::GroundTruth home);
	~FollowTargetSimulator();

	// Integrate simulator by one time step
	// This moves the target on a line
	void update(float delta_t_s);

	// Retrieve noisy version of position state in NED coordinate frame
	// The noise is deterministic and changes whenever the update() function is called
	std::array<float, 3> get_position_ned_noisy();

	// Retrieve ground truth of position state in NED coordinate frame
	std::array<float, 3> get_position_ground_truth_ned();

	// Retrieve noisy version of velocity state in NED coordinate frame
	// The noise is deterministic and changes whenever the update() function is called
	std::array<float, 3> get_velocity_ned_noisy();

	// Retrieve ground truth of velocity state in NED coordinate frame
	std::array<float, 3> get_velocity_ned_ground_truth();

	// Retrieve noisy version of position state in global coordinate frame (lat, lon, alt)
	// The noise is deterministic and changes whenever the update() function is called
	std::array<double, 3> get_position_global_noisy();

	// Retrieve ground truth of position state in global coordinate frame (lat, lon, alt)
	std::array<double, 3> get_position_global_ground_truth();

	// Run checks whether the drone has the correct angle towards the target, specified by the follow-me configuration
	void check_follow_angle(FollowMe::Config config, std::array<float, 3> drone_pos_ned,
				std::array<float, 3> target_pos_ned, float tolerance);

private:
	// Retrieve estimate with the option to add deterministic gaussian noise
	//
	// @param add_noise: Add gaussian noise to the state. Noise is deterministic and changes with each inokation of update()
	std::array<double, 3> get_position_global(bool add_noise);

	// Retrieve estimate with the option to add deterministic gaussian noise
	//
	// @param add_noise: Add gaussian noise to the state. Noise is deterministic and changes with each inokation of update()
	std::array<float, 3> get_position_ned(bool add_noise);

	// Retrieve estimate with the option to add deterministic gaussian noise
	//
	// @param add_noise: Add gaussian noise to the state. Noise is deterministic and changes with each inokation of update()
	std::array<float, 3> get_velocity_ned(bool add_noise);

	std::array<float, 3> _position_ned;
	std::array<float, 3> _velocity_ned;
	mavsdk::Telemetry::GroundTruth _home;
	size_t _udpate_count = 0;

	const double pos_noise_std = 0.3;
	const double vel_noise_std = 0.1;

	std::unique_ptr<mavsdk::Telemetry> _telemetry{};
};

class AutopilotTesterFollowMe : public AutopilotTester
{
public:
	AutopilotTesterFollowMe() = default;
	~AutopilotTesterFollowMe() = default;
	void connect(const std::string uri);

	void straight_line_test(const float altitude_m, const bool stream_velocity);

	void stream_velocity_only();

	void rc_override_test(const float altitude_m);

private:
	std::unique_ptr<mavsdk::FollowMe> _follow_me{};
};
