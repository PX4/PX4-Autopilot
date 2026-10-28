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

#include "DifferentialManualMode.hpp"

using namespace time_literals;

DifferentialManualMode::DifferentialManualMode(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_speed_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
}

void DifferentialManualMode::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
}

void DifferentialManualMode::manual()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);
	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = hrt_absolute_time();
	rover_steering_setpoint.normalized_steering_setpoint = _param_rd_yaw_stk_gain.get() * math::superexpo<float>
			(manual_control_setpoint.roll, _param_ro_yaw_expo.get(), _param_ro_yaw_supexpo.get());
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.throttle;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
}

void DifferentialManualMode::manualParking()
{
	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	const float max_throttle = _param_ro_park_thr_max.get();
	const float throttle_left = max_throttle * manual_control_setpoint.throttle;
	const float throttle_right = max_throttle * manual_control_setpoint.pitch;

	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = hrt_absolute_time();
	rover_throttle_setpoint.throttle_body_x = (throttle_left + throttle_right) / 2.f;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = hrt_absolute_time();
	rover_steering_setpoint.normalized_steering_setpoint = (throttle_left - throttle_right) / 2.f;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
}

void DifferentialManualMode::acro()
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
	rover_rate_setpoint.yaw_rate_setpoint = _max_yaw_rate * math::superexpo<float>(manual_control_setpoint.roll,
						_param_ro_yaw_expo.get(), _param_ro_yaw_supexpo.get());
	_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
}

void DifferentialManualMode::stab()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
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
		rover_rate_setpoint.yaw_rate_setpoint = _max_yaw_rate * math::superexpo<float>(math::deadzone(
				manual_control_setpoint.roll, _param_ro_yaw_stick_dz.get()), _param_ro_yaw_expo.get(), _param_ro_yaw_supexpo.get());
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

void DifferentialManualMode::position()
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		matrix::Quatf vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
	}

	manual_control_setpoint_s manual_control_setpoint{};
	_manual_control_setpoint_sub.copy(&manual_control_setpoint);

	const float speed_setpoint = math::interpolate<float>(manual_control_setpoint.throttle,
				     -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());

	if (fabsf(manual_control_setpoint.roll) > FLT_EPSILON
	    || fabsf(speed_setpoint) < FLT_EPSILON) {
		_pos_ctl_bearing = NAN;

		// Speed control
		rover_speed_setpoint_s rover_speed_setpoint{};
		rover_speed_setpoint.timestamp = hrt_absolute_time();
		rover_speed_setpoint.speed_body_x = speed_setpoint;
		_rover_speed_setpoint_pub.publish(rover_speed_setpoint);

		// Rate control
		rover_rate_setpoint_s rover_rate_setpoint{};
		rover_rate_setpoint.timestamp = hrt_absolute_time();
		rover_rate_setpoint.yaw_rate_setpoint = _max_yaw_rate * math::superexpo<float>(math::deadzone(
				manual_control_setpoint.roll, _param_ro_yaw_stick_dz.get()), _param_ro_yaw_expo.get(), _param_ro_yaw_supexpo.get());
		_rover_rate_setpoint_pub.publish(rover_rate_setpoint);

		// Set uncontrolled setpoints invalid
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = hrt_absolute_time();
		rover_attitude_setpoint.yaw_setpoint = NAN;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.target_ned[0] = NAN;
		rover_position_setpoint.target_ned[1] = NAN;
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else { // Follow an endless linear desired track
		if (!PX4_ISFINITE(_pos_ctl_bearing)) {
			// The bearing points backwards when driving in reverse
			_pos_ctl_bearing = speed_setpoint > 0.f ? _vehicle_yaw : matrix::wrap_pi(_vehicle_yaw + M_PI_F);
			_pos_ctl_start_position_ned = _curr_pos_ned;
		}

		// Hold the linear desired track through the start position: the target is kept beyond the maximum lookahead, so it is never reached
		const Vector2f bearing_direction(cosf(_pos_ctl_bearing), sinf(_pos_ctl_bearing));
		const float distance_along_track = fabsf((_curr_pos_ned - _pos_ctl_start_position_ned) * bearing_direction);
		const Vector2f target_ned = _pos_ctl_start_position_ned + (distance_along_track + _param_pp_lookahd_max.get()) *
					    bearing_direction;

		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = hrt_absolute_time();
		rover_position_setpoint.target_ned[0] = target_ned(0);
		rover_position_setpoint.target_ned[1] = target_ned(1);
		rover_position_setpoint.start_ned[0] = _pos_ctl_start_position_ned(0);
		rover_position_setpoint.start_ned[1] = _pos_ctl_start_position_ned(1);
		rover_position_setpoint.cruising_speed = speed_setpoint;
		// Arriving at the target with the cruising speed keeps the rover from braking for it
		rover_position_setpoint.arrival_speed = fabsf(speed_setpoint);
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);
	}
}

void DifferentialManualMode::reset()
{
	_stab_yaw_setpoint = NAN;
	_pos_ctl_bearing = NAN;
	_pos_ctl_start_position_ned = Vector2f(NAN, NAN);
	_curr_pos_ned = Vector2f(NAN, NAN);
}
