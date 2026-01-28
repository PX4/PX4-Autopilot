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

#include "MecanumActControl.hpp"

using namespace time_literals;

MecanumActControl::MecanumActControl(ModuleParams *parent) : ModuleParams(parent)
{
	updateParams();
}

void MecanumActControl::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ro_accel_limit.get() > FLT_EPSILON && _param_ro_max_thr_speed.get() > FLT_EPSILON) {
		_adjusted_throttle_x_setpoint.setSlewRate(_param_ro_accel_limit.get() / _param_ro_max_thr_speed.get());
		_adjusted_throttle_y_setpoint.setSlewRate(_param_ro_accel_limit.get() / _param_ro_max_thr_speed.get());
	}
}

void MecanumActControl::updateActControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	const float dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 10_ms) * 1e-6f;

	// Motor control
	if (_rover_throttle_setpoint_sub.updated()) {
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		_rover_throttle_setpoint_sub.copy(&rover_throttle_setpoint);
		_throttle_x_setpoint = rover_throttle_setpoint.throttle_body_x;
		_throttle_y_setpoint = rover_throttle_setpoint.throttle_body_y;
	}

	if (_rover_steering_setpoint_sub.updated()) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		_rover_steering_setpoint_sub.copy(&rover_steering_setpoint);
		_speed_diff_setpoint = rover_steering_setpoint.normalized_steering_setpoint;
	}

	if (PX4_ISFINITE(_throttle_x_setpoint) && PX4_ISFINITE(_throttle_y_setpoint) && PX4_ISFINITE(_speed_diff_setpoint)) {
		actuator_motors_s actuator_motors_sub{};
		_actuator_motors_sub.copy(&actuator_motors_sub);
		const float current_throttle_x = (actuator_motors_sub.control[0] + actuator_motors_sub.control[1]) / 2.f;
		const float current_throttle_y = (actuator_motors_sub.control[2] - actuator_motors_sub.control[0]) / 2.f;
		const float adjusted_throttle_x_setpoint = RoverControl::throttleControl(_adjusted_throttle_x_setpoint,
				_throttle_x_setpoint, current_throttle_x, _param_ro_accel_limit.get(),
				_param_ro_decel_limit.get(), _param_ro_max_thr_speed.get(), dt);
		const float adjusted_throttle_y_setpoint = RoverControl::throttleControl(_adjusted_throttle_y_setpoint,
				_throttle_y_setpoint, current_throttle_y, _param_ro_accel_limit.get(),
				_param_ro_decel_limit.get(), _param_ro_max_thr_speed.get(), dt);
		actuator_motors_s actuator_motors{};
		actuator_motors.reversible_flags = _param_r_rev.get();
		computeInverseKinematics(adjusted_throttle_x_setpoint, adjusted_throttle_y_setpoint,
					 _speed_diff_setpoint).copyTo(actuator_motors.control);
		actuator_motors.timestamp = _timestamp;
		_actuator_motors_pub.publish(actuator_motors);

	}

}

Vector4f MecanumActControl::computeInverseKinematics(float throttle_body_x, float throttle_body_y,
		const float speed_diff_normalized)
{
	const float total_speed = fabsf(throttle_body_x) + fabsf(throttle_body_y) + fabsf(speed_diff_normalized);

	if (total_speed > 1.f) { // Adjust speed setpoints if infeasible
		const float theta = atan2f(fabsf(throttle_body_y), fabsf(throttle_body_x));
		const float magnitude = (1.f - fabsf(speed_diff_normalized)) / (sinf(theta) + cosf(theta));
		const float normalization = 1.f / (sqrtf(powf(throttle_body_x, 2.f) + powf(throttle_body_y, 2.f)));
		throttle_body_x *= magnitude * normalization;
		throttle_body_y *= magnitude * normalization;

	}

	// Calculate motor commands
	const float input_data[3] = {throttle_body_x, throttle_body_y, speed_diff_normalized};
	const Matrix<float, 3, 1> input(input_data);
	const float m_data[12] = {1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f, 1.f, -1.f, 1.f, -1.f, 1.f};
	const Matrix<float, 4, 3> m(m_data);
	const Vector4f motor_commands = m * input;

	return motor_commands;
}

void MecanumActControl::stopVehicle()
{
	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	actuator_motors.control[0] = 0.f;
	actuator_motors.control[1] = 0.f;
	actuator_motors.control[2] = 0.f;
	actuator_motors.control[3] = 0.f;
	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);
}
