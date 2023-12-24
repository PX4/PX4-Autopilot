/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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

#include "DifferentialDriveKinematics.hpp"

#include <mathlib/mathlib.h>

using namespace matrix;

matrix::Vector2f DifferentialDriveKinematics::computeInverseKinematics(float linear_velocity_x, float yaw_rate)
{
	if (_max_speed < FLT_EPSILON) {
		return Vector2f();
	}

	linear_velocity_x = math::constrain(linear_velocity_x, -_max_speed, _max_speed);
	yaw_rate = math::constrain(yaw_rate, -_max_angular_velocity, _max_angular_velocity);

	const float rotational_velocity = (_wheel_base / 2.f) * yaw_rate;
	float combined_velocity = fabsf(linear_velocity_x) + fabsf(rotational_velocity);

	// Compute an initial gain
	float gain = 1.0f;

	if (combined_velocity > _max_speed) {
		float excess_velocity = fabsf(combined_velocity - _max_speed);
		float adjusted_linear_velocity = fabsf(linear_velocity_x) - excess_velocity;
		gain = adjusted_linear_velocity / fabsf(linear_velocity_x);
	}

	// Apply the gain
	linear_velocity_x *= gain;

	// Calculate the left and right wheel speeds
	return Vector2f(linear_velocity_x - rotational_velocity,
			linear_velocity_x + rotational_velocity) / _max_speed;
}

