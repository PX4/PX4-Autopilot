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

/**
 * @file FlightTaskManualPosition.cpp
 */

#include "FlightTaskManualPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

FlightTaskManualPosition::FlightTaskManualPosition(control::SuperBlock *parent, const char *name) :
	FlightTaskManualAltitude(parent, name),
	_vel_xy_manual_max(parent, "MPC_VEL_MANUAL", false),
	_acc_xy_max(parent, "MPC_ACC_HOR_MAX", false),
	_vel_hold_thr_xy(parent,  "MPC_HOLD_MAX_XY", false)
{}

bool FlightTaskManualPosition::activate()
{
	bool ret = FlightTaskManualAltitude::activate();
	_vel_sp(0) = _vel_sp(1) = 0.0f;
	return ret;
}

void FlightTaskManualPosition::_scaleSticks()
{
	/* Use same scaling as for FlightTaskManualAltitude */
	FlightTaskManualAltitude::_scaleSticks();

	/* Constrain length of stick inputs to 1 for xy*/
	Vector2f stick_xy(_sticks_expo(0), _sticks_expo(1));

	float mag = math::constrain(stick_xy.length(), 0.0f, 1.0f);

	if (mag > FLT_EPSILON) {
		stick_xy = stick_xy.normalized() * mag;
	}

	/* Scale to velocity.*/
	Vector2f vel_sp_xy = stick_xy * _vel_xy_manual_max.get();

	/* Rotate setpoint into local frame. */
	Vector3f vel_local = (Dcmf(Eulerf(0.0f, 0.0f, _yaw_sp)) * Vector3f(vel_sp_xy(0), vel_sp_xy(1), 0.0f));
	_vel_sp(0) = vel_local(0);
	_vel_sp(1) = vel_local(1);
}

void FlightTaskManualPosition::_updateXYlock()
{
	/* If position lock is not active, position setpoint is set to NAN.*/
	const float vel_xy_norm = Vector2f(&_velocity(0)).length();
	const bool apply_brake = Vector2f(&_vel_sp(0)).length() < FLT_EPSILON;
	const bool stopped = (_vel_hold_thr_xy.get() < FLT_EPSILON || vel_xy_norm < _vel_hold_thr_xy.get());

	if (apply_brake && stopped && !PX4_ISFINITE(_pos_sp(0))) {
		_pos_sp(0) = _position(0);
		_pos_sp(1) = _position(1);

	} else if (!apply_brake) {
		/* don't lock*/
		_pos_sp(0) = NAN;
		_pos_sp(1) = NAN;
	}
}

void FlightTaskManualPosition::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction
	_thr_sp *= NAN; // don't require any thrust setpoints
	_updateXYlock(); // check for position lock
}
