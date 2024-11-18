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

#include "RoverMecanumControl.hpp"

#include <mathlib/math/Limits.hpp>

using namespace matrix;
using namespace time_literals;

RoverMecanumControl::RoverMecanumControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
	_rover_mecanum_status_pub.advertise();
}

void RoverMecanumControl::updateParams()
{
	ModuleParams::updateParams();

	_pid_yaw_rate.setGains(_param_rm_yaw_rate_p.get(), _param_rm_yaw_rate_i.get(), 0.f);
	_pid_yaw_rate.setIntegralLimit(1.f);
	_pid_yaw_rate.setOutputLimit(1.f);

	_pid_forward_throttle.setGains(_param_rm_p_gain_speed.get(), _param_rm_i_gain_speed.get(), 0.f);
	_pid_forward_throttle.setIntegralLimit(1.f);
	_pid_forward_throttle.setOutputLimit(1.f);

	_pid_lateral_throttle.setGains(_param_rm_p_gain_speed.get(), _param_rm_i_gain_speed.get(), 0.f);
	_pid_lateral_throttle.setIntegralLimit(1.f);
	_pid_lateral_throttle.setOutputLimit(1.f);

	_max_yaw_rate = _param_rm_max_yaw_rate.get() * M_DEG_TO_RAD_F;
	_max_yaw_accel = _param_rm_max_yaw_accel.get() * M_DEG_TO_RAD_F;
	_pid_yaw.setGains(_param_rm_p_gain_yaw.get(), _param_rm_i_gain_yaw.get(), 0.f);
	_pid_yaw.setIntegralLimit(_max_yaw_rate);
	_pid_yaw.setOutputLimit(_max_yaw_rate);

	// Update slew rates
	if (_max_yaw_rate > FLT_EPSILON) {
		_yaw_setpoint_with_yaw_rate_limit.setSlewRate(_max_yaw_rate);
	}

	if (_max_yaw_accel > FLT_EPSILON) {
		_yaw_rate_with_accel_limit.setSlewRate(_max_yaw_accel);
	}
}

void RoverMecanumControl::computeMotorCommands(const float vehicle_yaw, const float vehicle_yaw_rate,
		const float vehicle_forward_speed, const float vehicle_lateral_speed)
{
	// Timestamps
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	// Update mecanum setpoint
	_rover_mecanum_setpoint_sub.update(&_rover_mecanum_setpoint);

	// Closed loop yaw control
	if (PX4_ISFINITE(_rover_mecanum_setpoint.yaw_setpoint)) {
		_yaw_setpoint_with_yaw_rate_limit.update(matrix::wrap_pi(_rover_mecanum_setpoint.yaw_setpoint), dt);
		_rover_mecanum_status.adjusted_yaw_setpoint = matrix::wrap_pi(_yaw_setpoint_with_yaw_rate_limit.getState());
		_pid_yaw.setSetpoint(
			matrix::wrap_pi(_yaw_setpoint_with_yaw_rate_limit.getState() -
					vehicle_yaw));  // error as setpoint to take care of wrapping
		_rover_mecanum_setpoint.yaw_rate_setpoint = _pid_yaw.update(0.f, dt);
		_rover_mecanum_status.clyaw_yaw_rate_setpoint = _rover_mecanum_setpoint.yaw_rate_setpoint;

	} else {
		_pid_yaw.resetIntegral();
		_yaw_setpoint_with_yaw_rate_limit.setForcedValue(vehicle_yaw);
	}

	// Yaw rate control
	float speed_diff_normalized{0.f};

	if (PX4_ISFINITE(_rover_mecanum_setpoint.yaw_rate_setpoint)) { // Closed loop yaw rate control
		speed_diff_normalized = calcNormalizedSpeedDiff(_rover_mecanum_setpoint.yaw_rate_setpoint, vehicle_yaw_rate,
					_param_rm_max_thr_yaw_r.get(), _max_yaw_accel, _param_rm_wheel_track.get(), dt, _yaw_rate_with_accel_limit,
					_pid_yaw_rate, false);
		_rover_mecanum_status.adjusted_yaw_rate_setpoint = _yaw_rate_with_accel_limit.getState();

	} else { // Use normalized setpoint
		speed_diff_normalized = calcNormalizedSpeedDiff(_rover_mecanum_setpoint.speed_diff_setpoint_normalized,
					vehicle_yaw_rate,
					_param_rm_max_thr_yaw_r.get(), _max_yaw_accel, _param_rm_wheel_track.get(), dt, _yaw_rate_with_accel_limit,
					_pid_yaw_rate, true);
	}

	// Speed control
	float forward_speed_normalized{0.f};
	float lateral_speed_normalized{0.f};

	if (PX4_ISFINITE(_rover_mecanum_setpoint.forward_speed_setpoint)
	    && PX4_ISFINITE(_rover_mecanum_setpoint.lateral_speed_setpoint)) { // Closed loop speed control
		forward_speed_normalized = calcNormalizedSpeedSetpoint(_rover_mecanum_setpoint.forward_speed_setpoint,
					   vehicle_forward_speed, _param_rm_max_thr_spd.get(), _forward_speed_setpoint_with_accel_limit, _pid_forward_throttle,
					   _param_rm_max_accel.get(), _param_rm_max_decel.get(), dt, false);
		lateral_speed_normalized = calcNormalizedSpeedSetpoint(_rover_mecanum_setpoint.lateral_speed_setpoint,
					   vehicle_lateral_speed, _param_rm_max_thr_spd.get(), _lateral_speed_setpoint_with_accel_limit, _pid_lateral_throttle,
					   _param_rm_max_accel.get(), _param_rm_max_decel.get(), dt, false);
		_rover_mecanum_status.adjusted_forward_speed_setpoint = _forward_speed_setpoint_with_accel_limit.getState();
		_rover_mecanum_status.adjusted_lateral_speed_setpoint = _lateral_speed_setpoint_with_accel_limit.getState();


	} else if (PX4_ISFINITE(_rover_mecanum_setpoint.forward_speed_setpoint_normalized)
		   && PX4_ISFINITE(_rover_mecanum_setpoint.lateral_speed_setpoint_normalized)) { // Use normalized setpoint
		forward_speed_normalized = calcNormalizedSpeedSetpoint(_rover_mecanum_setpoint.forward_speed_setpoint_normalized,
					   vehicle_forward_speed, _param_rm_max_thr_spd.get(), _forward_speed_setpoint_with_accel_limit, _pid_forward_throttle,
					   _param_rm_max_accel.get(), _param_rm_max_decel.get(), dt, true);
		lateral_speed_normalized = calcNormalizedSpeedSetpoint(_rover_mecanum_setpoint.lateral_speed_setpoint_normalized,
					   vehicle_lateral_speed, _param_rm_max_thr_spd.get(), _lateral_speed_setpoint_with_accel_limit, _pid_lateral_throttle,
					   _param_rm_max_accel.get(), _param_rm_max_decel.get(), dt, true);

	}

	// Publish rover mecanum status (logging)
	_rover_mecanum_status.timestamp = _timestamp;
	_rover_mecanum_status.measured_forward_speed = vehicle_forward_speed;
	_rover_mecanum_status.measured_lateral_speed = vehicle_lateral_speed;
	_rover_mecanum_status.measured_yaw_rate = vehicle_yaw_rate;
	_rover_mecanum_status.measured_yaw = vehicle_yaw;
	_rover_mecanum_status.pid_yaw_rate_integral = _pid_yaw_rate.getIntegral();
	_rover_mecanum_status.pid_yaw_integral = _pid_yaw.getIntegral();
	_rover_mecanum_status.pid_forward_throttle_integral = _pid_forward_throttle.getIntegral();
	_rover_mecanum_status.pid_lateral_throttle_integral = _pid_lateral_throttle.getIntegral();
	_rover_mecanum_status_pub.publish(_rover_mecanum_status);

	// Publish to motors
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	computeInverseKinematics(forward_speed_normalized, lateral_speed_normalized,
				 speed_diff_normalized).copyTo(actuator_motors.control);
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);

}

float RoverMecanumControl::calcNormalizedSpeedDiff(const float yaw_rate_setpoint, const float vehicle_yaw_rate,
		const float max_thr_yaw_r,
		const float max_yaw_accel, const float wheel_track, const float dt, SlewRate<float> &yaw_rate_with_accel_limit,
		PID &pid_yaw_rate, const bool normalized)
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
		if (wheel_track > FLT_EPSILON && max_thr_yaw_r > FLT_EPSILON) { // Feedforward
			const float speed_diff = yaw_rate_with_accel_limit.getState() * wheel_track /
						 2.f;
			speed_diff_normalized = math::interpolate<float>(speed_diff, -max_thr_yaw_r,
						max_thr_yaw_r, -1.f, 1.f);
		}

		_pid_yaw_rate.setSetpoint(yaw_rate_with_accel_limit.getState());
		speed_diff_normalized = math::constrain(speed_diff_normalized +
							_pid_yaw_rate.update(vehicle_yaw_rate, dt),
							-1.f, 1.f); // Feedback


	}

	return math::constrain(speed_diff_normalized, -1.f, 1.f);

}

float RoverMecanumControl::calcNormalizedSpeedSetpoint(const float speed_setpoint,
		const float vehicle_speed, const float max_thr_spd, SlewRate<float> &speed_setpoint_with_accel_limit,
		PID &pid_throttle, const float max_accel, const float max_decel, const float dt, const bool normalized)
{
	float slew_rate_normalization{1.f};

	if (normalized) { // Slew rate needs to be normalized if the setpoint is normalized
		slew_rate_normalization = max_thr_spd > FLT_EPSILON ? max_thr_spd : 0.f;
	}

	// Apply acceleration and deceleration limit
	if (fabsf(speed_setpoint) >= fabsf(speed_setpoint_with_accel_limit.getState())) {
		if (max_accel > FLT_EPSILON && slew_rate_normalization > FLT_EPSILON) {
			speed_setpoint_with_accel_limit.setSlewRate(max_accel / slew_rate_normalization);
			speed_setpoint_with_accel_limit.update(speed_setpoint, dt);

		} else {
			speed_setpoint_with_accel_limit.setForcedValue(speed_setpoint);

		}

	} else if (max_decel > FLT_EPSILON && slew_rate_normalization > FLT_EPSILON) {
		speed_setpoint_with_accel_limit.setSlewRate(max_decel / slew_rate_normalization);
		speed_setpoint_with_accel_limit.update(speed_setpoint, dt);

	} else {
		speed_setpoint_with_accel_limit.setForcedValue(speed_setpoint);
	}

	// Calculate normalized forward speed setpoint
	float forward_speed_normalized{0.f};

	if (normalized) {
		forward_speed_normalized = speed_setpoint_with_accel_limit.getState();

	} else { // Closed loop speed control

		if (_param_rm_max_thr_spd.get() > FLT_EPSILON) { // Feedforward
			forward_speed_normalized = math::interpolate<float>(speed_setpoint_with_accel_limit.getState(),
						   -max_thr_spd, max_thr_spd,
						   -1.f, 1.f);
		}

		pid_throttle.setSetpoint(speed_setpoint_with_accel_limit.getState());
		forward_speed_normalized += pid_throttle.update(vehicle_speed, dt); // Feedback

	}

	return math::constrain(forward_speed_normalized, -1.f, 1.f);

}

matrix::Vector4f RoverMecanumControl::computeInverseKinematics(float forward_speed_normalized,
		float lateral_speed_normalized,
		float speed_diff)
{
	const float total_speed = fabsf(forward_speed_normalized) + fabsf(lateral_speed_normalized) + fabsf(speed_diff);

	if (total_speed > 1.f) { // Adjust speed setpoints if infeasible
		const float theta = atan2f(fabsf(lateral_speed_normalized), fabsf(forward_speed_normalized));
		const float magnitude = (1.f - fabsf(speed_diff)) / (sinf(theta) + cosf(theta));
		const float normalization = 1.f / (sqrtf(powf(forward_speed_normalized, 2.f) + powf(lateral_speed_normalized, 2.f)));
		forward_speed_normalized *= magnitude * normalization;
		lateral_speed_normalized *= magnitude * normalization;

	}

	// Calculate motor commands
	const float input_data[3] = {forward_speed_normalized, lateral_speed_normalized, speed_diff};
	const Matrix<float, 3, 1> input(input_data);
	const float m_data[12] = {1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f, 1.f, -1.f, 1.f, -1.f, 1.f};
	const Matrix<float, 4, 3> m(m_data);
	const Vector4f motor_commands = m * input;

	return motor_commands;
}

void RoverMecanumControl::resetControllers()
{
	_pid_forward_throttle.resetIntegral();
	_pid_lateral_throttle.resetIntegral();
	_pid_yaw_rate.resetIntegral();
	_pid_yaw.resetIntegral();
}
