/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPosition.cpp
 */

#include "FlightTaskManualPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskManualPosition::FlightTaskManualPosition(control::SuperBlock *parent, const char *name) :
	FlightTaskManualAltitude(parent, name),
	_vel_xy_manual_max(parent, "MPC_VEL_MANUAL", false),
	_acc_xy_max(parent, "MPC_ACC_HOR_MAX", false), \
	_vel_xy_dz(parent,  "MPC_HOLD_MAX_XY", false)
{}

bool FlightTaskManualPosition::activate()
{
	_pos_sp_xy = matrix::Vector2f(&_position(0));
	_vel_sp_xy = matrix::Vector2f(0.0f, 0.0f);
	return FlightTaskManualAltitude::activate();
}

void FlightTaskManualPosition::_scaleSticks()
{
	/* Use same scaling as for FlightTaskManualAltitude to
	 * get yaw and z */
	FlightTaskManualAltitude::_scaleSticks();

	/* Constrain length of stick inputs to 1 for xy*/
	matrix::Vector2f stick_xy(_sticks_expo(0), _sticks_expo(1));

	float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);

	if (mag > FLT_EPSILON) {
		stick_xy = stick_xy.normalized() * mag;
	}

	/* Scale to velocity.*/
	_vel_sp_xy = stick_xy * _vel_xy_manual_max.get();

	/* Rotate setpoint into local frame. */
	matrix::Vector3f vel_sp{_vel_sp_xy(0), _vel_sp_xy(1), 0.0f};
	vel_sp = (matrix::Dcmf(matrix::Eulerf(0.0f, 0.0f, _yaw)) * vel_sp);
	_vel_sp_xy = matrix::Vector2f(vel_sp(0), vel_sp(1));
}

void FlightTaskManualPosition::_updateXYsetpoints()
{
	/* If position lock is not active, position setpoint is set to NAN.*/
	const float vel_xy_norm = Vector2f(&_velocity(0)).length();
	const bool stick_zero = matrix::Vector2f(_sticks_expo(0), _sticks_expo(1)).length() < FLT_EPSILON;
	const bool stopped = (_vel_xy_dz.get() < FLT_EPSILON || vel_xy_norm < _vel_xy_dz.get());

	if (stick_zero && stopped && !PX4_ISFINITE(_pos_sp_xy(0))) {
		_pos_sp_xy = matrix::Vector2f(&_position(0));

	} else if (!stick_zero) {
		_pos_sp_xy = _pos_sp_xy * NAN;
	}
}

void FlightTaskManualPosition::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // get yaw and z setpoints
	_updateXYsetpoints(); // get xy setpoints
}

bool FlightTaskManualPosition::update()
{
	_scaleSticks();
	_updateSetpoints();

	_setPositionSetpoint(Vector3f(_pos_sp_xy(0), _pos_sp_xy(1), _pos_sp_z));
	_setVelocitySetpoint(Vector3f(_vel_sp_xy(0), _vel_sp_xy(1), _vel_sp_z));
	_setYawSetpoint(_yaw_sp);
	_setYawspeedSetpoint(_yaw_rate_sp);

	return true;
}
