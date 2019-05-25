/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file FlightTaskManualAcceleration.cpp
 */

#include "FlightTaskManualAcceleration.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualAcceleration::activate(vehicle_local_position_setpoint_s last_setpoint)
{
	bool ret = FlightTaskManual::activate(last_setpoint);
	return ret;
}

bool FlightTaskManualAcceleration::update()
{
	// maximum commanded acceleration and velocity
	Vector3f acceleration_scale;
	Vector3f velocity_scale;
	acceleration_scale(0) = acceleration_scale(1) = _param_mpc_acc_hor.get();
	velocity_scale(0) = velocity_scale(1) = _param_mpc_vel_manual.get();

	if (_velocity(2) < 0) {
		acceleration_scale(2) = _param_mpc_acc_up_max.get();
		velocity_scale(2) = _param_mpc_z_vel_max_up.get();

	} else {
		acceleration_scale(2) = _param_mpc_acc_down_max.get();
		velocity_scale(2) = _param_mpc_z_vel_max_dn.get();
	}

	acceleration_scale *= 2.f; // because of drag the average aceleration is half

	// Yaw
	_position_lock.updateYawFromStick(_yawspeed_setpoint, _yaw_setpoint,
					  _sticks_expo(3) * math::radians(_param_mpc_man_y_max.get()), _yaw, _deltatime);

	// Map stick input to acceleration
	Vector2f stick_xy(&_sticks_expo(0));
	_position_lock.limitStickUnitLengthXY(stick_xy);
	_position_lock.rotateIntoHeadingFrameXY(stick_xy, _yaw, _yaw_setpoint);


	_acceleration_setpoint = Vector3f(stick_xy(0), stick_xy(1), _sticks_expo(2)).emult(acceleration_scale);

	// Add drag to limit speed and brake again
	Vector3f cv = acceleration_scale.edivide(velocity_scale);
	_acceleration_setpoint -= cv.emult(_velocity);

	lockAltitude();
	lockPosition(stick_xy.length());

	_constraints.want_takeoff = _checkTakeoff();
	return true;
}

void FlightTaskManualAcceleration::lockAltitude()
{
	if (fabsf(_sticks_expo(2)) > FLT_EPSILON) {
		_position_setpoint(2) = NAN;
		_velocity_setpoint(2) = NAN;

	} else {
		if (!PX4_ISFINITE(_position_setpoint(2))) {
			_position_setpoint(2) = _position(2);
			_velocity_setpoint(2) = _velocity(2);
		}

		_position_setpoint(2) += _velocity_setpoint(2) * _deltatime;

		const float velocity_setpoint_z_last = _velocity_setpoint(2);
		_velocity_setpoint(2) += _acceleration_setpoint(2) * _deltatime;

		if (fabsf(_velocity_setpoint(2)) > fabsf(velocity_setpoint_z_last)) {
			_velocity_setpoint(2) = velocity_setpoint_z_last;
		}
	}
}

void FlightTaskManualAcceleration::lockPosition(const float stick_input_xy)
{
	if (stick_input_xy > FLT_EPSILON) {
		_position_setpoint(0) = _position_setpoint(1) = NAN;
		_velocity_setpoint(0) = _velocity_setpoint(1) = NAN;

	} else {
		Vector2f position_xy(_position_setpoint);
		Vector2f velocity_xy(_velocity_setpoint);

		if (!PX4_ISFINITE(position_xy(0))) {
			position_xy = Vector2f(_position);
			velocity_xy = Vector2f(_velocity);
		}

		position_xy += Vector2f(velocity_xy) * _deltatime;

		const Vector2f velocity_xy_last = velocity_xy;
		velocity_xy += Vector2f(_acceleration_setpoint) * _deltatime;

		if (velocity_xy.norm_squared() > velocity_xy_last.norm_squared()) {
			velocity_xy = velocity_xy_last;
		}

		_position_setpoint(0) = position_xy(0);
		_position_setpoint(1) = position_xy(1);
		_velocity_setpoint(0) = velocity_xy(0);
		_velocity_setpoint(1) = velocity_xy(1);
	}
}

void FlightTaskManualAcceleration::_ekfResetHandlerHeading(float delta_psi)
{
	// Only reset the yaw setpoint when the heading is locked
	if (PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint += delta_psi;
	}
}
