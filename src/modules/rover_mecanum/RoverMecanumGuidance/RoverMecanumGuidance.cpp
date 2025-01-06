/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "RoverMecanumGuidance.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

RoverMecanumGuidance::RoverMecanumGuidance(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_max_velocity_magnitude = _param_rm_max_speed.get();
	_rover_mecanum_guidance_status_pub.advertise();
}

void RoverMecanumGuidance::updateParams()
{
	ModuleParams::updateParams();
}

void RoverMecanumGuidance::computeGuidance(const float yaw, const int nav_state)
{
	updateSubscriptions();

	// Calculate desired velocity magnitude
	float desired_velocity_magnitude{_max_velocity_magnitude};
	const float distance_to_curr_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
					  _curr_wp(0), _curr_wp(1));

	if (_param_rm_max_jerk.get() > FLT_EPSILON && _param_rm_max_accel.get() > FLT_EPSILON
	    && _param_rm_miss_vel_gain.get() > FLT_EPSILON) {
		const float speed_reduction = math::constrain(_param_rm_miss_vel_gain.get() * math::interpolate(
						      M_PI_F - _waypoint_transition_angle, 0.f,
						      M_PI_F, 0.f, 1.f), 0.f, 1.f);
		desired_velocity_magnitude = math::trajectory::computeMaxSpeedFromDistance(_param_rm_max_jerk.get(),
					     _param_rm_max_accel.get(), distance_to_curr_wp, _max_velocity_magnitude * (1.f - speed_reduction));
		desired_velocity_magnitude = math::constrain(desired_velocity_magnitude, -_max_velocity_magnitude,
					     _max_velocity_magnitude);
	}


	// Calculate heading error
	const float desired_heading = _pure_pursuit.calcDesiredHeading(_curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
				      desired_velocity_magnitude);
	const float heading_error = matrix::wrap_pi(desired_heading - yaw);

	// Catch return to launch
	const float distance_to_next_wp = get_distance_to_next_waypoint(_curr_pos(0), _curr_pos(1),
					  _next_wp(0), _next_wp(1));

	if (nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		_mission_finished = distance_to_next_wp < _param_nav_acc_rad.get();
	}

	// Guidance logic
	Vector2f desired_velocity(0.f, 0.f);

	if (!_mission_finished && distance_to_curr_wp > _param_nav_acc_rad.get()) {
		desired_velocity = desired_velocity_magnitude * Vector2f(cosf(heading_error), sinf(heading_error));
	}

	// Timestamp
	hrt_abstime timestamp = hrt_absolute_time();

	// Publish mecanum controller status (logging)
	rover_mecanum_guidance_status_s rover_mecanum_guidance_status{};
	rover_mecanum_guidance_status.timestamp = timestamp;
	rover_mecanum_guidance_status.lookahead_distance = _pure_pursuit.getLookaheadDistance();
	rover_mecanum_guidance_status.heading_error = heading_error;
	rover_mecanum_guidance_status.desired_speed = desired_velocity_magnitude;
	_rover_mecanum_guidance_status_pub.publish(rover_mecanum_guidance_status);

	// Publish setpoints
	rover_mecanum_setpoint_s rover_mecanum_setpoint{};
	rover_mecanum_setpoint.timestamp = timestamp;
	rover_mecanum_setpoint.forward_speed_setpoint = desired_velocity(0);
	rover_mecanum_setpoint.forward_speed_setpoint_normalized = NAN;
	rover_mecanum_setpoint.lateral_speed_setpoint = desired_velocity(1);
	rover_mecanum_setpoint.lateral_speed_setpoint_normalized = NAN;
	rover_mecanum_setpoint.yaw_rate_setpoint = NAN;
	rover_mecanum_setpoint.speed_diff_setpoint_normalized = NAN;
	rover_mecanum_setpoint.yaw_setpoint = _desired_yaw;
	_rover_mecanum_setpoint_pub.publish(rover_mecanum_setpoint);
}

void RoverMecanumGuidance::updateSubscriptions()
{
	if (_vehicle_global_position_sub.updated()) {
		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);
		_curr_pos = Vector2d(vehicle_global_position.lat, vehicle_global_position.lon);
	}

	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);

		if (!_global_ned_proj_ref.isInitialized()
		    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(local_position.ref_lat, local_position.ref_lon, local_position.ref_timestamp);
		}

		_curr_pos_ned = Vector2f(local_position.x, local_position.y);
	}

	if (_position_setpoint_triplet_sub.updated()) {
		updateWaypoints();
	}

	if (_mission_result_sub.updated()) {
		mission_result_s mission_result{};
		_mission_result_sub.copy(&mission_result);
		_mission_finished = mission_result.finished;
	}

	if (_home_position_sub.updated()) {
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);
		_home_position = Vector2d(home_position.lat, home_position.lon);
	}
}

void RoverMecanumGuidance::updateWaypoints()
{
	position_setpoint_triplet_s position_setpoint_triplet{};
	_position_setpoint_triplet_sub.copy(&position_setpoint_triplet);

	// Global waypoint coordinates
	if (position_setpoint_triplet.current.valid && PX4_ISFINITE(position_setpoint_triplet.current.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.current.lon)) {
		_curr_wp = Vector2d(position_setpoint_triplet.current.lat, position_setpoint_triplet.current.lon);

	} else {
		_curr_wp = Vector2d(0, 0);
	}

	if (position_setpoint_triplet.previous.valid && PX4_ISFINITE(position_setpoint_triplet.previous.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.previous.lon)) {
		_prev_wp = Vector2d(position_setpoint_triplet.previous.lat, position_setpoint_triplet.previous.lon);

	} else {
		_prev_wp = _curr_pos;
	}

	if (position_setpoint_triplet.next.valid && PX4_ISFINITE(position_setpoint_triplet.next.lat)
	    && PX4_ISFINITE(position_setpoint_triplet.next.lon)) {
		_next_wp = Vector2d(position_setpoint_triplet.next.lat, position_setpoint_triplet.next.lon);

	} else {
		_next_wp = _home_position;
	}

	// NED waypoint coordinates
	_curr_wp_ned = _global_ned_proj_ref.project(_curr_wp(0), _curr_wp(1));
	_prev_wp_ned = _global_ned_proj_ref.project(_prev_wp(0), _prev_wp(1));
	_next_wp_ned = _global_ned_proj_ref.project(_next_wp(0), _next_wp(1));

	// Distances
	const Vector2f curr_to_next_wp_ned = _next_wp_ned - _curr_wp_ned;
	const Vector2f curr_to_prev_wp_ned = _prev_wp_ned - _curr_wp_ned;
	float cosin = curr_to_prev_wp_ned.unit_or_zero() * curr_to_next_wp_ned.unit_or_zero();
	cosin = math::constrain<float>(cosin, -1.f, 1.f); // Protect against float precision problem
	_waypoint_transition_angle = acosf(cosin);

	// Waypoint cruising speed
	_max_velocity_magnitude = position_setpoint_triplet.current.cruising_speed > FLT_EPSILON ? math::constrain(
					  position_setpoint_triplet.current.cruising_speed, 0.f,
					  _param_rm_max_speed.get()) : _param_rm_max_speed.get();

	// Waypoint yaw setpoint
	if (PX4_ISFINITE(position_setpoint_triplet.current.yaw)) {
		_desired_yaw = position_setpoint_triplet.current.yaw;
	}
}
