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

#include "ManualMode.hpp"

using namespace time_literals;

ManualMode::ManualMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_velocity_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
}

void ManualMode::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
}

void ManualMode::manual()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = hrt_absolute_time();
	rover_steering_setpoint.normalized_steering_angle = manual_control_setpoint.roll;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
}

void ManualMode::acro()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	rover_rate_setpoint_s rover_rate_setpoint{};
	rover_rate_setpoint.timestamp = hrt_absolute_time();
	rover_rate_setpoint.yaw_rate_setpoint = matrix::sign(manual_control_setpoint.throttle) * math::interpolate<float>
						(manual_control_setpoint.roll, -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);
	_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
}

void ManualMode::stab()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	if (fabsf(manual_control_setpoint.roll) > FLT_EPSILON
	    || fabsf(rover_throttle_setpoint.throttle_body_x) < FLT_EPSILON) {
		_stab_yaw_setpoint = NAN;

		// Rate control
		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = hrt_absolute_time();
		rover_rate_setpoint.yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
							_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);;
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

		// Set uncontrolled setpoint invalid
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = NAN;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

	} else { // Heading control
		if (!PX4_ISFINITE(_stab_yaw_setpoint)) {
			_stab_yaw_setpoint = _vehicle_yaw;
		}

		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = _stab_yaw_setpoint;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
	}
}

void ManualMode::position()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);

		if (!_global_ned_proj_ref.isInitialized()
		    || (_global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp)) {
			_global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
							   vehicle_local_position.ref_timestamp);
		}

		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
	}

	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	const float speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
				     -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());

	if (fabsf(manual_control_setpoint.roll) > FLT_EPSILON
	    || fabsf(speed_setpoint) < FLT_EPSILON) {
		_pos_ctl_course_direction = Vector2f(NAN, NAN);

		// Speed control
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = hrt_absolute_time();
		rover_velocity_setpoint.speed = speed_setpoint;
		rover_velocity_setpoint.bearing = NAN;
		rover_velocity_setpoint.yaw = NAN;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

		// Rate control
		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = hrt_absolute_time();
		rover_rate_setpoint.yaw_rate_setpoint = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
							_param_ro_yaw_stick_dz.get()), -1.f, 1.f, -_max_yaw_rate, _max_yaw_rate);;
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

		// Set uncontrolled setpoints invalid
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = NAN;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = NAN;
		rover_position_setpoint.position_ned[1] = NAN;
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else { // Course control
		if (!_pos_ctl_course_direction.isAllFinite()) {
			_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
			_pos_ctl_start_position_ned = _curr_pos_ned;
		}

		// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
		const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
		const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_pp_lookahd_max.get();
		const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(speed_setpoint) *
						     vector_scaling * _pos_ctl_course_direction;
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.position_ned[0] = target_waypoint_ned(0);
		rover_position_setpoint.position_ned[1] = target_waypoint_ned(1);
		rover_position_setpoint.start_ned[0] = _pos_ctl_start_position_ned(0);
		rover_position_setpoint.start_ned[1] = _pos_ctl_start_position_ned(1);
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.cruising_speed = speed_setpoint;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}
}

void ManualMode::reset()
{
	_stab_yaw_setpoint = NAN;
	_pos_ctl_course_direction = Vector2f(NAN, NAN);
	_pos_ctl_start_position_ned = Vector2f(NAN, NAN);
	_curr_pos_ned = Vector2f(NAN, NAN);
}
