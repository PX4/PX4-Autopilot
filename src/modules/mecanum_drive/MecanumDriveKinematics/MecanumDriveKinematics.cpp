/****************************************************************************
 *
 *   Copyright (C) 2023-2024 PX4 Development Team. All rights reserved.
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

#include "MecanumDriveKinematics.hpp"

#include <mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

MecanumDriveKinematics::MecanumDriveKinematics(ModuleParams *parent) : ModuleParams(parent)
{}

void MecanumDriveKinematics::allocate()
{
	hrt_abstime now = hrt_absolute_time();

	if (_mecanum_drive_control_output_sub.updated()) {
		_mecanum_drive_control_output_sub.copy(&_mecanum_drive_control_output);
	}

	const bool setpoint_timeout = (_mecanum_drive_control_output.timestamp + 100_ms) < now;

	Vector4f wheel_speeds =
		computeInverseKinematics(_mecanum_drive_control_output.speed[0], _mecanum_drive_control_output.speed[1],
					 _mecanum_drive_control_output.yaw_rate);

	// printf("wheel_speeds: %f, %f, %f, %f\n", (double)wheel_speeds(0), (double)wheel_speeds(1), (double)wheel_speeds(2),
	//        (double)wheel_speeds(3));

	if (!_armed || setpoint_timeout) {
		wheel_speeds = {}; // stop
	}

	wheel_speeds = matrix::constrain(wheel_speeds, -1.f, 1.f);

	// printf("wheel_speeds after all checks: %f, %f, %f, %f\n", (double)wheel_speeds(0), (double)wheel_speeds(1),
	//        (double)wheel_speeds(2),
	//        (double)wheel_speeds(3));

	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get(); // should be 15 see rc.rover_mecanum_defaults
	wheel_speeds.copyTo(actuator_motors.control);
	actuator_motors.timestamp = now;
	_actuator_motors_pub.publish(actuator_motors);
}

matrix::Vector4f MecanumDriveKinematics::computeInverseKinematics(float linear_velocity_x, float linear_velocity_y,
		float yaw_rate) const
{
	// if (_max_speed < FLT_EPSILON) {
	// 	return Vector4f();
	// }

	linear_velocity_x = math::constrain(linear_velocity_x, -_vx_max, _vx_max);
	linear_velocity_y = math::constrain(linear_velocity_y, -_vy_max, _vy_max);
	yaw_rate = math::constrain(yaw_rate, -_max_angular_velocity, _max_angular_velocity);

	// Define input vector and matrix data
	float input_data[3] = {linear_velocity_x, linear_velocity_y, yaw_rate};
	Matrix<float, 3, 1> input(input_data);

	float m_data[12] = {1, -1, -(_lx + _ly), 1, 1, (_lx + _ly), 1, 1, -(_lx + _ly), 1, -1, (_lx + _ly)};
	Matrix<float, 4, 3> m(m_data);

	// Perform matrix-vector multiplication
	auto result = m * input; // result is Matrix<float, 4, 1>

	// Precompute scale factor
	const float scale = 1 / _r;

	// Scale the result by 1/_r
	for (size_t i = 0; i < 4; ++i) {
		result(i, 0) *= scale; // Efficiently use precomputed scale factor
	}

	// Initialize Vector4f with the scaled results
	Vector4f output(result(0, 0), result(1, 0), result(2, 0), result(3, 0));

	// output = {0.1f, 0.5f, -0.5f, -0.1f};

	// printf("output: %f, %f, %f, %f\n", (double)output(0), (double)output(1), (double)output(2), (double)output(3));

	return output;
}
