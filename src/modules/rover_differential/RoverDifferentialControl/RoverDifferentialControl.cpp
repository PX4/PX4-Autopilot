/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "RoverDifferentialControl.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

RoverDifferentialControl::RoverDifferentialControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_differential_status_pub.advertise();
	pid_init(&_pid_yaw_rate, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_pid_throttle, PID_MODE_DERIVATIV_NONE, 0.001f);
	pid_init(&_pid_yaw, PID_MODE_DERIVATIV_NONE, 0.001f);
}

void RoverDifferentialControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_rd_max_yaw_rate.get() * M_DEG_TO_RAD_F;
	_max_yaw_accel = _param_rd_max_yaw_accel.get() * M_DEG_TO_RAD_F;

	// Update PID
	pid_set_parameters(&_pid_yaw_rate,
			   _param_rd_yaw_rate_p.get(), // Proportional gain
			   _param_rd_yaw_rate_i.get(), // Integral gain
			   0.f, // Derivative gain
			   1.f, // Integral limit
			   1.f); // Output limit
	pid_set_parameters(&_pid_throttle,
			   _param_rd_p_gain_speed.get(), // Proportional gain
			   _param_rd_i_gain_speed.get(), // Integral gain
			   0.f, // Derivative gain
			   1.f, // Integral limit
			   1.f); // Output limit
	pid_set_parameters(&_pid_yaw,
			   _param_rd_p_gain_yaw.get(),  // Proportional gain
			   _param_rd_i_gain_yaw.get(),  // Integral gain
			   0.f,  // Derivative gain
			   _max_yaw_rate,  // Integral limit
			   _max_yaw_rate);  // Output limit

	// Update slew rates
	if (_max_yaw_rate > FLT_EPSILON) {
		_yaw_setpoint_with_yaw_rate_limit.setSlewRate(_max_yaw_rate);
	}

	if (_max_yaw_accel > FLT_EPSILON) {
		_yaw_rate_with_accel_limit.setSlewRate(_max_yaw_accel);
	}

}

void RoverDifferentialControl::computeMotorCommands(const float vehicle_yaw, const float vehicle_yaw_rate,
		const float vehicle_forward_speed)
{
	// Timestamps
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// Update differential setpoint
	_rover_differential_setpoint_sub.update(&_rover_differential_setpoint);

	// Closed loop yaw control (Overrides yaw rate setpoint)
	if (PX4_ISFINITE(_rover_differential_setpoint.yaw_setpoint)) {
		_yaw_setpoint_with_yaw_rate_limit.update(matrix::wrap_pi(_rover_differential_setpoint.yaw_setpoint), dt);
		_rover_differential_status.adjusted_yaw_setpoint = matrix::wrap_pi(_yaw_setpoint_with_yaw_rate_limit.getState());
		const float heading_error = matrix::wrap_pi(_yaw_setpoint_with_yaw_rate_limit.getState() - vehicle_yaw);
		_rover_differential_setpoint.yaw_rate_setpoint = pid_calculate(&_pid_yaw, heading_error, 0, 0, dt);
		_rover_differential_status.clyaw_yaw_rate_setpoint = _rover_differential_setpoint.yaw_rate_setpoint;

	} else {
		_yaw_setpoint_with_yaw_rate_limit.setForcedValue(vehicle_yaw);
	}

	// Yaw rate control
	float speed_diff_normalized{0.f};

	if (PX4_ISFINITE(_rover_differential_setpoint.yaw_rate_setpoint)) { // Closed loop yaw rate control
		speed_diff_normalized = calcNormalizedSpeedDiff(_rover_differential_setpoint.yaw_rate_setpoint, vehicle_yaw_rate,
					_param_rd_max_thr_yaw_r.get(), _max_yaw_accel, _param_rd_wheel_track.get(), dt, _yaw_rate_with_accel_limit,
					_pid_yaw_rate, false);

	} else { // Use normalized setpoint
		speed_diff_normalized = calcNormalizedSpeedDiff(_rover_differential_setpoint.speed_diff_setpoint_normalized,
					vehicle_yaw_rate,
					_param_rd_max_thr_yaw_r.get(), _max_yaw_accel, _param_rd_wheel_track.get(), dt, _yaw_rate_with_accel_limit,
					_pid_yaw_rate, true);
	}

	// Speed control
	float forward_speed_normalized{0.f};

	if (PX4_ISFINITE(_rover_differential_setpoint.forward_speed_setpoint)) {
		forward_speed_normalized = calcNormalizedSpeedSetpoint(_rover_differential_setpoint.forward_speed_setpoint,
					   vehicle_forward_speed, _param_rd_max_thr_spd.get(), _forward_speed_setpoint_with_accel_limit, _pid_throttle,
					   _param_rd_max_accel.get(), _param_rd_max_decel.get(), dt, false);

	} else if (PX4_ISFINITE(_rover_differential_setpoint.forward_speed_setpoint_normalized)) { // Use normalized setpoint
		forward_speed_normalized = calcNormalizedSpeedSetpoint(_rover_differential_setpoint.forward_speed_setpoint_normalized,
					   vehicle_forward_speed, _param_rd_max_thr_spd.get(), _forward_speed_setpoint_with_accel_limit, _pid_throttle,
					   _param_rd_max_accel.get(), _param_rd_max_decel.get(), dt, true);

	}

	// Publish rover differential status (logging)
	_rover_differential_status.timestamp = _timestamp;
	_rover_differential_status.measured_forward_speed = vehicle_forward_speed;
	_rover_differential_status.measured_yaw = vehicle_yaw;
	_rover_differential_status.measured_yaw_rate = vehicle_yaw_rate;
	_rover_differential_status.pid_yaw_rate_integral = _pid_yaw_rate.integral;
	_rover_differential_status.pid_throttle_integral = _pid_throttle.integral;
	_rover_differential_status.pid_yaw_integral = _pid_yaw.integral;
	_rover_differential_status_pub.publish(_rover_differential_status);

	// Publish to motors
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	computeInverseKinematics(forward_speed_normalized, speed_diff_normalized).copyTo(actuator_motors.control);
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);
}

float RoverDifferentialControl::calcNormalizedSpeedDiff(const float yaw_rate_setpoint, const float vehicle_yaw_rate,
		const float max_thr_yaw_r,
		const float max_yaw_accel, const float wheel_track, const float dt, SlewRate<float> &yaw_rate_with_accel_limit,
		PID_t &pid_yaw_rate, const bool normalized)
{
	float slew_rate_normalization{1.f};

	if (normalized) { // Slew rate needs to be normalized if the setpoint is normalized
		slew_rate_normalization = max_thr_yaw_r > FLT_EPSILON ? max_thr_yaw_r : 0.f;
	}

	if (max_yaw_accel > FLT_EPSILON && slew_rate_normalization > FLT_EPSILON) {
		yaw_rate_with_accel_limit.setSlewRate(max_yaw_accel / slew_rate_normalization);
		yaw_rate_with_accel_limit.update(yaw_rate_setpoint, dt);

	} else {
		yaw_rate_with_accel_limit.setForcedValue(yaw_rate_setpoint);
	}

	// Transform yaw rate into speed difference
	float speed_diff_normalized{0.f};

	if (normalized) {
		speed_diff_normalized = yaw_rate_with_accel_limit.getState();

	} else {
		_rover_differential_status.adjusted_yaw_rate_setpoint = yaw_rate_with_accel_limit.getState();

		if (wheel_track > FLT_EPSILON && max_thr_yaw_r > FLT_EPSILON) { // Feedforward
			const float speed_diff = yaw_rate_with_accel_limit.getState() * wheel_track /
						 2.f;
			speed_diff_normalized = math::interpolate<float>(speed_diff, -max_thr_yaw_r,
						max_thr_yaw_r, -1.f, 1.f);
		}

		speed_diff_normalized += pid_calculate(&pid_yaw_rate, yaw_rate_with_accel_limit.getState(), vehicle_yaw_rate, 0,
						       dt); // Feedback
	}

	return math::constrain(speed_diff_normalized, -1.f, 1.f);

}

float RoverDifferentialControl::calcNormalizedSpeedSetpoint(const float forward_speed_setpoint,
		const float vehicle_forward_speed, const float max_thr_spd, SlewRate<float> &forward_speed_setpoint_with_accel_limit,
		PID_t &pid_throttle, const float max_accel, const float max_decel, const float dt, const bool normalized)
{
	float slew_rate_normalization{1.f};

	if (normalized) { // Slew rate needs to be normalized if the setpoint is normalized
		slew_rate_normalization = max_thr_spd > FLT_EPSILON ? max_thr_spd : 0.f;
	}

	// Apply acceleration and deceleration limit
	if (fabsf(forward_speed_setpoint) >= fabsf(forward_speed_setpoint_with_accel_limit.getState())) {
		if (max_accel > FLT_EPSILON && slew_rate_normalization > FLT_EPSILON) {
			forward_speed_setpoint_with_accel_limit.setSlewRate(max_accel / slew_rate_normalization);
			forward_speed_setpoint_with_accel_limit.update(forward_speed_setpoint, dt);

		} else {
			forward_speed_setpoint_with_accel_limit.setForcedValue(forward_speed_setpoint);

		}

	} else if (max_decel > FLT_EPSILON && slew_rate_normalization > FLT_EPSILON) {
		forward_speed_setpoint_with_accel_limit.setSlewRate(max_decel / slew_rate_normalization);
		forward_speed_setpoint_with_accel_limit.update(forward_speed_setpoint, dt);

	} else {
		forward_speed_setpoint_with_accel_limit.setForcedValue(forward_speed_setpoint);
	}

	// Calculate normalized forward speed setpoint
	float forward_speed_normalized{0.f};

	if (normalized) {
		forward_speed_normalized = _forward_speed_setpoint_with_accel_limit.getState();

	} else { // Closed loop speed control
		_rover_differential_status.adjusted_forward_speed_setpoint = _forward_speed_setpoint_with_accel_limit.getState();

		if (_param_rd_max_thr_spd.get() > FLT_EPSILON) { // Feedforward
			forward_speed_normalized = math::interpolate<float>(_forward_speed_setpoint_with_accel_limit.getState(),
						   -max_thr_spd, max_thr_spd,
						   -1.f, 1.f);
		}

		forward_speed_normalized += pid_calculate(&pid_throttle, _forward_speed_setpoint_with_accel_limit.getState(),
					    vehicle_forward_speed, 0, dt); // Feedback
	}

	return math::constrain(forward_speed_normalized, -1.f, 1.f);

}

matrix::Vector2f RoverDifferentialControl::computeInverseKinematics(float forward_speed_normalized,
		const float speed_diff_normalized)
{
	float max_motor_command = fabsf(forward_speed_normalized) + fabsf(speed_diff_normalized);

	if (max_motor_command > 1.0f) { // Prioritize yaw rate if a normalized motor command exceeds limit of 1
		float excess = fabsf(max_motor_command - 1.0f);
		forward_speed_normalized -= sign(forward_speed_normalized) * excess;
	}

	// Calculate the left and right wheel speeds
	return Vector2f(forward_speed_normalized - speed_diff_normalized,
			forward_speed_normalized + speed_diff_normalized);
}

void RoverDifferentialControl::resetControllers()
{
	pid_reset_integral(&_pid_throttle);
	pid_reset_integral(&_pid_yaw_rate);
	pid_reset_integral(&_pid_yaw);
}
