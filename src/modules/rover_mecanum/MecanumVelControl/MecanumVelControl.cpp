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

#include "MecanumVelControl.hpp"

using namespace time_literals;

MecanumVelControl::MecanumVelControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_rover_velocity_setpoint_pub.advertise();
	_rover_velocity_status_pub.advertise();
	updateParams();
}

void MecanumVelControl::updateParams()
{
	ModuleParams::updateParams();
	_pid_speed_x.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed_x.setIntegralLimit(1.f);
	_pid_speed_x.setOutputLimit(1.f);
	_pid_speed_y.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed_y.setIntegralLimit(1.f);
	_pid_speed_y.setOutputLimit(1.f);

	if (_param_ro_accel_limit.get() > FLT_EPSILON) {
		_speed_x_setpoint.setSlewRate(_param_ro_accel_limit.get());
		_speed_y_setpoint.setSlewRate(_param_ro_accel_limit.get());
	}
}

void MecanumVelControl::updateVelControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	if ((_vehicle_control_mode.flag_control_velocity_enabled) && _vehicle_control_mode.flag_armed && runSanityChecks()) {
		if (_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard Velocity Control
			generateVelocitySetpoint();
		}

		generateAttitudeAndThrottleSetpoint();

	} else { // Reset controller and slew rate when velocity control is not active
		_pid_speed_x.resetIntegral();
		_speed_x_setpoint.setForcedValue(0.f);
		_pid_speed_y.resetIntegral();
		_speed_y_setpoint.setForcedValue(0.f);
	}

	// Publish position controller status (logging only)
	rover_velocity_status_s rover_velocity_status;
	rover_velocity_status.timestamp = _timestamp;
	rover_velocity_status.measured_speed_body_x = _vehicle_speed_body_x;
	rover_velocity_status.adjusted_speed_body_x_setpoint = _speed_x_setpoint.getState();
	rover_velocity_status.measured_speed_body_y = _vehicle_speed_body_y;
	rover_velocity_status.adjusted_speed_body_y_setpoint = _speed_y_setpoint.getState();
	rover_velocity_status.pid_throttle_body_x_integral = _pid_speed_x.getIntegral();
	rover_velocity_status.pid_throttle_body_y_integral = _pid_speed_y.getIntegral();
	_rover_velocity_status_pub.publish(rover_velocity_status);
}

void MecanumVelControl::updateSubscriptions()
{
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

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

}
void MecanumVelControl::generateVelocitySetpoint()
{
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	if (_offboard_control_mode_sub.updated()) {
		_offboard_control_mode_sub.copy(&_offboard_control_mode);
	}

	const bool offboard_vel_control = _offboard_control_mode.velocity && !_offboard_control_mode.position;

	const Vector2f velocity_in_local_frame(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);

	if (offboard_vel_control && velocity_in_local_frame.isAllFinite()) {
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = velocity_in_local_frame.norm();
		rover_velocity_setpoint.bearing = atan2f(velocity_in_local_frame(1), velocity_in_local_frame(0));
		rover_velocity_setpoint.yaw = _vehicle_yaw;
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	}
}

void MecanumVelControl::generateAttitudeAndThrottleSetpoint()
{
	if (_rover_velocity_setpoint_sub.updated()) {
		_rover_velocity_setpoint_sub.copy(&_rover_velocity_setpoint);
	}

	// Attitude Setpoint
	if (PX4_ISFINITE(_rover_velocity_setpoint.yaw)) {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		rover_attitude_setpoint.yaw_setpoint = _rover_velocity_setpoint.yaw;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
		_last_attitude_setpoint_update = _timestamp;

	} else {
		rover_attitude_setpoint_s rover_attitude_setpoint{};
		rover_attitude_setpoint.timestamp = _timestamp;
		rover_attitude_setpoint.yaw_setpoint = _vehicle_yaw;
		_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);
		_last_attitude_setpoint_update = _timestamp;
	}

	// Throttle Setpoint
	float speed_body_x_setpoint{0.f};
	float speed_body_y_setpoint{0.f};

	if (fabsf(_rover_velocity_setpoint.speed) > FLT_EPSILON) {
		const Vector3f velocity_in_local_frame(_rover_velocity_setpoint.speed * cosf(
				_rover_velocity_setpoint.bearing),
						       _rover_velocity_setpoint.speed * sinf(_rover_velocity_setpoint.bearing), 0.f);
		const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		speed_body_x_setpoint = velocity_in_body_frame(0);
		speed_body_y_setpoint = velocity_in_body_frame(1);

	}

	if (_param_ro_max_thr_speed.get() > FLT_EPSILON) { // Adjust speed setpoints if infeasible
		if (_rover_steering_setpoint_sub.updated()) {
			_rover_steering_setpoint_sub.copy(&_rover_steering_setpoint);
		}

		float speed_body_x_setpoint_normalized = math::interpolate<float>(speed_body_x_setpoint,
				-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

		float speed_body_y_setpoint_normalized = math::interpolate<float>(speed_body_y_setpoint,
				-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

		const float total_speed = fabsf(speed_body_x_setpoint_normalized) + fabsf(speed_body_y_setpoint_normalized) + fabsf(
						  _rover_steering_setpoint.normalized_speed_diff);

		if (total_speed > 1.f) {
			const float theta = atan2f(fabsf(speed_body_y_setpoint_normalized), fabsf(speed_body_x_setpoint_normalized));
			const float magnitude = (1.f - fabsf(_rover_steering_setpoint.normalized_speed_diff)) / (sinf(theta) + cosf(theta));
			const float normalization = 1.f / (sqrtf(powf(speed_body_x_setpoint_normalized,
							   2.f) + powf(speed_body_y_setpoint_normalized, 2.f)));
			speed_body_x_setpoint_normalized *= magnitude * normalization;
			speed_body_y_setpoint_normalized *= magnitude * normalization;
			speed_body_x_setpoint = math::interpolate<float>(speed_body_x_setpoint_normalized, -1.f, 1.f,
						-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
			speed_body_y_setpoint = math::interpolate<float>(speed_body_y_setpoint_normalized, -1.f, 1.f,
						-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
		}
	}

	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = _timestamp;
	speed_body_x_setpoint = fabsf(speed_body_x_setpoint) > _param_ro_speed_th.get() ? speed_body_x_setpoint : 0.f;
	speed_body_y_setpoint = fabsf(speed_body_y_setpoint) > _param_ro_speed_th.get() ? speed_body_y_setpoint : 0.f;
	rover_throttle_setpoint.throttle_body_x = RoverControl::speedControl(_speed_x_setpoint, _pid_speed_x,
			speed_body_x_setpoint, _vehicle_speed_body_x, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
			_param_ro_max_thr_speed.get(), _dt);
	rover_throttle_setpoint.throttle_body_y = RoverControl::speedControl(_speed_y_setpoint, _pid_speed_y,
			speed_body_y_setpoint, _vehicle_speed_body_y, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
			_param_ro_max_thr_speed.get(), _dt);
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);

}

bool MecanumVelControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float>(events::ID("mecanum_vel_control_conf_invalid_speed_lim"), events::Log::Error,
					    "Invalid configuration of necessary parameter RO_SPEED_LIM", _param_ro_speed_limit.get());
		}

	}

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON && _param_ro_speed_p.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float, float>(events::ID("mecanum_vel_control_conf_invalid_speed_control"), events::Log::Error,
						   "Invalid configuration for speed control: Neither feed forward (RO_MAX_THR_SPEED) nor feedback (RO_SPEED_P) is setup",
						   _param_ro_max_thr_speed.get(),
						   _param_ro_speed_p.get());
		}
	}

	_prev_param_check_passed = ret;
	return ret;
}
