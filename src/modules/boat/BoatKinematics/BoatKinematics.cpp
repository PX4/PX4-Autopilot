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

#include "BoatKinematics.hpp"

#include <mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

BoatKinematics::BoatKinematics(ModuleParams *parent) : ModuleParams(parent)
{}

void BoatKinematics::allocate()
{
	hrt_abstime now = hrt_absolute_time();

	if (_boat_control_output_sub.updated()) {
		_boat_control_output_sub.copy(&_boat_control_output);
	}

	const bool setpoint_timeout = (_boat_control_output.timestamp + 100_ms) < now;

	Vector2f boat_output =
		computeInverseKinematics(_boat_control_output.speed, _boat_control_output.yaw_rate);

	if (!_armed || setpoint_timeout) {
		boat_output = {}; // stop
	}

	boat_output = matrix::constrain(boat_output, -1.f, 1.f);

	actuator_motors_s actuator_motors{};
	actuator_motors.control[0] = boat_output(0);
	actuator_motors.control[1] = boat_output(0);
	actuator_motors.timestamp = now;
	_actuator_motors_pub.publish(actuator_motors);

	actuator_servos_s actuator_servos{};
	actuator_servos.control[0] = boat_output(1);
	actuator_servos.control[1] = boat_output(1);
	actuator_servos.timestamp = now;
	_actuator_servos_pub.publish(actuator_servos);
}

matrix::Vector2f BoatKinematics::computeInverseKinematics(float linear_velocity_x, float yaw_rate) const
{
	if (_max_speed < FLT_EPSILON) {
		return Vector2f();
	}

	// Room for more advanced dynamics, if required

	linear_velocity_x = math::constrain(linear_velocity_x, -_max_speed, _max_speed);
	yaw_rate = math::constrain(yaw_rate, -_max_angular_velocity, _max_angular_velocity);

	float throttle = linear_velocity_x / _max_speed;
	float rudder_angle = yaw_rate / _max_angular_velocity;

	return Vector2f(throttle, rudder_angle);
}
