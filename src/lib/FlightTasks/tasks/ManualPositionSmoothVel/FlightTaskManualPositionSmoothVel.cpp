/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "FlightTaskManualPositionSmoothVel.hpp"

#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualPositionSmoothVel::activate()
{
	bool ret = FlightTaskManualPosition::activate();

	// TODO: get current accel
	for (int i = 0; i < 3; ++i) {
		_smoothing[i].reset(0.f, _velocity(i), _position(i));
	}

	return ret;
}

void FlightTaskManualPositionSmoothVel::_updateSetpoints()
{
	/* Get yaw setpont, un-smoothed position setpoints.*/
	FlightTaskManualPosition::_updateSetpoints();

	/* Update constraints */
	_smoothing[0].setMaxAccel(MPC_ACC_HOR_MAX.get());
	_smoothing[1].setMaxAccel(MPC_ACC_HOR_MAX.get());
	_smoothing[0].setMaxVel(_constraints.speed_xy);
	_smoothing[1].setMaxVel(_constraints.speed_xy);

	Vector2f vel_xy = Vector2f(&_velocity(0));
	float jerk = _jerk_max.get();

	if (_jerk_min.get() > _jerk_max.get()) {
		_jerk_min.set(0.f);
	}

	if (_jerk_min.get() > FLT_EPSILON) {
		// interpolate between min and max jerk
		jerk = math::min(_jerk_min.get() + (_jerk_max.get() - _jerk_min.get()) * vel_xy.length() / _constraints.speed_xy,
				 _jerk_max.get());
	}

	for (int i = 0; i < 2; ++i) {
		_smoothing[i].setMaxJerk(jerk);

		float smoothed_velocity_setpoint, smoothed_position_setpoint;
		_smoothing[i].update(_deltatime, _position(i), _velocity_setpoint(i),
				     smoothed_velocity_setpoint, smoothed_position_setpoint);
		_position_setpoint(i) = smoothed_position_setpoint;
		_velocity_setpoint(i) = smoothed_velocity_setpoint;
	}

	// TODO: z
}
