/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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


#include "autopilot_tester_flow.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

using namespace mavsdk;
using namespace mavsdk::geometry;

namespace
{

const char *const axis_name[3] = {"north", "east", "down"};

// Cap on the recorded ground truth trajectory so that a hanging test cannot grow it without bound.
constexpr size_t MAX_GROUND_TRUTH_SAMPLES = 100000;

// MAVLink command IDs (kept as literals to match the style of the other testers and avoid pulling
// extra mavlink enum headers).
constexpr uint16_t CMD_DO_ORBIT = 34;

// ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER: the nose follows the circle center, so the
// heading turns by a full revolution per orbit.
constexpr float ORBIT_YAW_FACE_CENTER = 0.f;

// Heading of a quaternion given as [w, x, y, z].
float heading_from_quaternion(const float q[4])
{
	return std::atan2(2.f * (q[0] * q[3] + q[1] * q[2]),
			  1.f - 2.f * (q[2] * q[2] + q[3] * q[3]));
}

float wrap_pi(float angle_rad)
{
	while (angle_rad > static_cast<float>(M_PI)) {
		angle_rad -= 2.f * static_cast<float>(M_PI);
	}

	while (angle_rad < -static_cast<float>(M_PI)) {
		angle_rad += 2.f * static_cast<float>(M_PI);
	}

	return angle_rad;
}

} // namespace

void AutopilotTesterFlow::set_ekf_origin_to_home(std::chrono::seconds timeout)
{
	const Telemetry::GroundTruth &home = getHome();

	const uint8_t target_sysid = getMavlinkPassthrough()->get_target_sysid();
	const int32_t latitude = static_cast<int32_t>(home.latitude_deg * 1e7);
	const int32_t longitude = static_cast<int32_t>(home.longitude_deg * 1e7);
	const int32_t altitude = static_cast<int32_t>(home.absolute_altitude_m * 1e3f);

	const MavlinkPassthrough::Result result = getMavlinkPassthrough()->queue_message(
	[target_sysid, latitude, longitude, altitude](MavlinkAddress mavlink_address, uint8_t channel) {
		mavlink_message_t message;
		mavlink_msg_set_gps_global_origin_pack_chan(
			mavlink_address.system_id,
			mavlink_address.component_id,
			channel,
			&message,
			target_sysid,
			latitude,
			longitude,
			altitude,
			0 /* time_usec, unused */);
		return message;
	});

	// Extra parentheses stop Catch2 from decomposing the expression: stringifying
	// MavlinkPassthrough::Result pulls in an operator<< that the MAVSDK lib does not export.
	REQUIRE((result == MavlinkPassthrough::Result::Success));

	// The estimator only publishes a global position once it has an origin, so a finite global
	// position confirms the origin was taken.
	std::cout << time_str() << "Waiting for the EKF origin to be set" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		const Telemetry::Position position = getTelemetry()->position();
		return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg);
	}, timeout));
}

void AutopilotTesterFlow::wait_until_armable(std::chrono::seconds timeout)
{
	std::cout << time_str() << "Waiting for the vehicle to be armable" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return getTelemetry()->health().is_armable; }, timeout));
}

void AutopilotTesterFlow::command_orbit(LocalCoordinate center, float radius_m, float velocity_m_s, float rel_alt_m)
{
	const auto global = get_coordinate_transformation().global_from_local(center);

	MavlinkPassthrough::CommandInt cmd{};
	cmd.target_sysid = getMavlinkPassthrough()->get_target_sysid();
	cmd.target_compid = getMavlinkPassthrough()->get_target_compid();
	cmd.command = CMD_DO_ORBIT;
	cmd.frame = MAV_FRAME_GLOBAL_INT;
	cmd.param1 = radius_m; // radius, sign encodes direction (positive == clockwise)
	cmd.param2 = velocity_m_s; // tangential velocity
	cmd.param3 = ORBIT_YAW_FACE_CENTER;
	cmd.param4 = NAN; // number of orbits: unlimited
	cmd.x = static_cast<int32_t>(global.latitude_deg * 1e7);
	cmd.y = static_cast<int32_t>(global.longitude_deg * 1e7);
	cmd.z = getHome().absolute_altitude_m + rel_alt_m;

	send_custom_mavlink_command(cmd);
}

void AutopilotTesterFlow::wait_until_on_circle(LocalCoordinate center, float radius_m, float tolerance_m,
		std::chrono::seconds timeout)
{
	// Flying out to the circle only crosses the band once and then stays on it, so the first time
	// the vehicle is in the band it is established on it.
	std::cout << time_str() << "Waiting for the vehicle to get onto the circle" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this, center, radius_m, tolerance_m]() {
		const Telemetry::PositionNed position = getTelemetry()->position_velocity_ned().position;
		const double distance = std::hypot(position.north_m - center.north_m, position.east_m - center.east_m);
		return std::fabs(distance - radius_m) < tolerance_m;
	}, timeout));
}

void AutopilotTesterFlow::start_ground_truth_comparison(double rate_hz)
{
	CHECK(getTelemetry()->set_rate_position_velocity_ned(rate_hz) == Telemetry::Result::Success);
	CHECK(getTelemetry()->set_rate_ground_truth(rate_hz) == Telemetry::Result::Success);

	{
		std::lock_guard<std::mutex> lock(_velocity_mutex);
		_estimate_valid = false;
		_error_samples = 0;

		for (unsigned i = 0; i < kAxes; ++i) {
			_error_sum_m_s[i] = 0.0;
			_error_max_m_s[i] = 0.f;
		}

		_heading_valid = false;
		_heading_swept_rad = 0.0;
		_ground_truth_lla.clear();
	}

	_position_velocity_handle = getTelemetry()->subscribe_position_velocity_ned(
	[this](Telemetry::PositionVelocityNed sample) {
		std::lock_guard<std::mutex> lock(_velocity_mutex);
		_estimated_velocity_m_s[0] = sample.velocity.north_m_s;
		_estimated_velocity_m_s[1] = sample.velocity.east_m_s;
		_estimated_velocity_m_s[2] = sample.velocity.down_m_s;
		_estimate_valid = true;
	});

	_ground_truth_handle = getMavlinkPassthrough()->subscribe_message(
				       MAVLINK_MSG_ID_HIL_STATE_QUATERNION,
	[this](const mavlink_message_t &message) {
		mavlink_hil_state_quaternion_t ground_truth{};
		mavlink_msg_hil_state_quaternion_decode(&message, &ground_truth);

		// HIL_STATE_QUATERNION carries the ground truth velocity in cm/s.
		const float truth_m_s[kAxes] = {
			ground_truth.vx * 1e-2f,
			ground_truth.vy * 1e-2f,
			ground_truth.vz * 1e-2f
		};

		const float heading_rad = heading_from_quaternion(ground_truth.attitude_quaternion);

		std::lock_guard<std::mutex> lock(_velocity_mutex);

		if (_ground_truth_lla.size() < MAX_GROUND_TRUTH_SAMPLES) {
			_ground_truth_lla.emplace_back(ground_truth.lat * 1e-7, ground_truth.lon * 1e-7);
		}

		if (_heading_valid) {
			_heading_swept_rad += std::fabs(wrap_pi(heading_rad - _previous_heading_rad));
		}

		_previous_heading_rad = heading_rad;
		_heading_valid = true;

		if (!_estimate_valid) {
			return;
		}

		for (unsigned i = 0; i < kAxes; ++i) {
			const float error = std::fabs(_estimated_velocity_m_s[i] - truth_m_s[i]);
			_error_sum_m_s[i] += error;
			_error_max_m_s[i] = std::max(_error_max_m_s[i], error);
		}

		++_error_samples;
	});
}

void AutopilotTesterFlow::stop_ground_truth_comparison()
{
	getTelemetry()->unsubscribe_position_velocity_ned(_position_velocity_handle);
	getMavlinkPassthrough()->unsubscribe_message(MAVLINK_MSG_ID_HIL_STATE_QUATERNION, _ground_truth_handle);
}

void AutopilotTesterFlow::wait_until_heading_swept(float min_sweep_deg, std::chrono::seconds timeout)
{
	std::cout << time_str() << "Waiting for the heading to turn by " << min_sweep_deg << " deg" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this, min_sweep_deg]() { return heading_swept_deg() > min_sweep_deg; }, timeout));
}

float AutopilotTesterFlow::heading_swept_deg()
{
	std::lock_guard<std::mutex> lock(_velocity_mutex);
	return static_cast<float>(_heading_swept_rad) * 180.f / static_cast<float>(M_PI);
}

void AutopilotTesterFlow::check_ground_truth_on_circle(LocalCoordinate center, float radius_m, float tolerance_m)
{
	const CoordinateTransformation ct({getHome().latitude_deg, getHome().longitude_deg});

	std::lock_guard<std::mutex> lock(_velocity_mutex);

	REQUIRE(_ground_truth_lla.size() > 100);

	double min_distance = std::numeric_limits<double>::max();
	double max_distance = 0.0;

	for (const auto &lla : _ground_truth_lla) {
		const auto local = ct.local_from_global({lla.first, lla.second});
		const double distance = std::hypot(local.north_m - center.north_m, local.east_m - center.east_m);
		min_distance = std::min(min_distance, distance);
		max_distance = std::max(max_distance, distance);
	}

	std::cout << time_str() << "ground truth distance to the orbit center: " << min_distance << " m to "
		  << max_distance << " m (radius " << radius_m << " m)" << std::endl;

	CHECK(min_distance > radius_m - tolerance_m);
	CHECK(max_distance < radius_m + tolerance_m);
}

void AutopilotTesterFlow::check_velocity_error_below(float mean_tol_m_s, float max_tol_m_s)
{
	std::lock_guard<std::mutex> lock(_velocity_mutex);

	// Without samples the checks below would pass vacuously.
	REQUIRE(_error_samples > 100);

	for (unsigned i = 0; i < kAxes; ++i) {
		const float mean = static_cast<float>(_error_sum_m_s[i] / _error_samples);
		std::cout << time_str() << "velocity error " << axis_name[i] << ": mean " << mean
			  << " m/s, max " << _error_max_m_s[i] << " m/s (" << _error_samples << " samples)" << std::endl;
		CHECK(mean < mean_tol_m_s);
		CHECK(_error_max_m_s[i] < max_tol_m_s);
	}
}
