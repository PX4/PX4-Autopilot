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

#include "MecanumSpeedControl.hpp"

using namespace time_literals;

MecanumSpeedControl::MecanumSpeedControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_speed_status_pub.advertise();
	updateParams();
}

void MecanumSpeedControl::updateParams()
{
	ModuleParams::updateParams();

	// Set up PID controllers
	_pid_speed_x.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed_x.setIntegralLimit(1.f);
	_pid_speed_x.setOutputLimit(1.f);
	_pid_speed_y.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed_y.setIntegralLimit(1.f);
	_pid_speed_y.setOutputLimit(1.f);

	// Set up slew rates
	if (_param_ro_accel_limit.get() > FLT_EPSILON) {
		_adjusted_speed_x_setpoint.setSlewRate(_param_ro_accel_limit.get());
		_adjusted_speed_y_setpoint.setSlewRate(_param_ro_accel_limit.get());
	}
}

void MecanumSpeedControl::updateSpeedControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 10_ms) * 1e-6f;

	updateSubscriptions();

	// Throttle Setpoints
	if (PX4_ISFINITE(_speed_x_setpoint) && PX4_ISFINITE(_speed_y_setpoint)) {
		Vector2f speed_setpoint = calcSpeedSetpoint();
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = _timestamp;

		rover_throttle_setpoint.throttle_body_x = RoverControl::speedControl(_adjusted_speed_x_setpoint, _pid_speed_x,
				speed_setpoint(0), _vehicle_speed_body_x, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
				_param_ro_max_thr_speed.get(), dt);
		rover_throttle_setpoint.throttle_body_y = RoverControl::speedControl(_adjusted_speed_y_setpoint, _pid_speed_y,
				speed_setpoint(1), _vehicle_speed_body_y, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
				_param_ro_max_thr_speed.get(), dt);
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

	}

	// Publish position controller status (logging only)
	rover_speed_status_s rover_speed_status;
	rover_speed_status.timestamp = _timestamp;
	rover_speed_status.measured_speed_body_x = _vehicle_speed_body_x;
	rover_speed_status.adjusted_speed_body_x_setpoint = _adjusted_speed_x_setpoint.getState();
	rover_speed_status.measured_speed_body_y = _vehicle_speed_body_y;
	rover_speed_status.adjusted_speed_body_y_setpoint = _adjusted_speed_y_setpoint.getState();
	rover_speed_status.pid_throttle_body_x_integral = _pid_speed_x.getIntegral();
	rover_speed_status.pid_throttle_body_y_integral = _pid_speed_y.getIntegral();
	_rover_speed_status_pub.publish(rover_speed_status);
}

void MecanumSpeedControl::updateSubscriptions()
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
		const Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_speed_body_x = fabsf(velocity_in_body_frame(0)) > _param_ro_speed_th.get() ? velocity_in_body_frame(0) : 0.f;
		_vehicle_speed_body_y = fabsf(velocity_in_body_frame(1)) > _param_ro_speed_th.get() ? velocity_in_body_frame(1) : 0.f;
	}

	if (_rover_speed_setpoint_sub.updated()) {
		rover_speed_setpoint_s rover_speed_setpoint;
		_rover_speed_setpoint_sub.copy(&rover_speed_setpoint);
		_speed_x_setpoint = rover_speed_setpoint.speed_body_x;
		_speed_y_setpoint = rover_speed_setpoint.speed_body_y;
	}
}

Vector2f MecanumSpeedControl::calcSpeedSetpoint()
{
	if (_rover_steering_setpoint_sub.updated()) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		_rover_steering_setpoint_sub.copy(&rover_steering_setpoint);
		_normalized_speed_diff = rover_steering_setpoint.normalized_steering_setpoint;
	}

	Vector2f speed_setpoint = Vector2f(_speed_x_setpoint, _speed_y_setpoint);

	float speed_x_setpoint_normalized = math::interpolate<float>(_speed_x_setpoint,
					    -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

	float speed_y_setpoint_normalized = math::interpolate<float>(_speed_y_setpoint,
					    -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

	const float total_speed = fabsf(speed_x_setpoint_normalized) + fabsf(speed_y_setpoint_normalized) + fabsf(
					  _normalized_speed_diff);

	if (total_speed > 1.f) {
		const float theta = atan2f(fabsf(speed_y_setpoint_normalized), fabsf(speed_x_setpoint_normalized));
		const float magnitude = (1.f - fabsf(_normalized_speed_diff)) / (sinf(theta) + cosf(theta));
		const float normalization = 1.f / (sqrtf(powf(speed_x_setpoint_normalized,
						   2.f) + powf(speed_y_setpoint_normalized, 2.f)));
		speed_x_setpoint_normalized *= magnitude * normalization;
		speed_y_setpoint_normalized *= magnitude * normalization;
		speed_setpoint(0) = math::interpolate<float>(speed_x_setpoint_normalized, -1.f, 1.f,
				    -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
		speed_setpoint(1) = math::interpolate<float>(speed_y_setpoint_normalized, -1.f, 1.f,
				    -_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
	}

	speed_setpoint(0) = fabsf(speed_setpoint(0)) > _param_ro_speed_th.get() ? speed_setpoint(0) : 0.f;
	speed_setpoint(1) = fabsf(speed_setpoint(1)) > _param_ro_speed_th.get() ? speed_setpoint(1) : 0.f;

	return speed_setpoint;
}

bool MecanumSpeedControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
		events::send<float>(events::ID("mecanum_vel_control_conf_invalid_speed_lim"), events::Log::Error,
				    "Invalid configuration of necessary parameter RO_SPEED_LIM", _param_ro_speed_limit.get());

	}

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON && _param_ro_speed_p.get() < FLT_EPSILON) {
		ret = false;
		events::send<float, float>(events::ID("mecanum_vel_control_conf_invalid_speed_control"), events::Log::Error,
					   "Invalid configuration for speed control: Neither feed forward (RO_MAX_THR_SPEED) nor feedback (RO_SPEED_P) is setup",
					   _param_ro_max_thr_speed.get(),
					   _param_ro_speed_p.get());
	}

	return ret;
}
