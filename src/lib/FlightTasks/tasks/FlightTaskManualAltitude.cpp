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
 * @file FlightTaskManual.hpp
 *
 * Flight task for manual controlled altitude.
 *
 */

#include "FlightTaskManualAltitude.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskManualAltitude::FlightTaskManualAltitude(control::SuperBlock *parent, const char *name) :
	FlightTaskManual(parent, name),
	_vel_max_down(parent, "MPC_Z_VEL_MAX_DN", false),
	_vel_max_up(parent, "MPC_Z_VEL_MAX_UP", false),
	_yaw_rate_scaling(parent, "MPC_MAN_Y_MAX", false)
{}

bool FlightTaskManualAltitude::activate()
{
	_pos_sp_predicted = _pos_sp_z = _position(2);
	_yaw_sp_predicted = _yaw_sp = _yaw;
	_yaw_rate_sp = _vel_sp_z = NAN;

	return FlightTaskManual::activate();
}

void FlightTaskManualAltitude::scale_sticks()
{

	/* map stick to velocity */
	const float vel_max_z = (_sticks(2) > 0.0f) ? _vel_max_down.get() : _vel_max_up.get();
	_vel_sp_z = vel_max_z * _sticks_expo(2);
	_yaw_rate_sp = _sticks(3) * math::radians(_yaw_rate_scaling.get());

}

void FlightTaskManualAltitude::update_heading_setpoints()
{
	if (fabsf(_sticks(3)) < FLT_EPSILON) {
		/* want to hold yaw */
		_yaw_rate_sp = NAN;
		_yaw_sp = _yaw_sp_predicted;

	} else {
		/* want to change yaw based on yaw-rate */
		_yaw_sp = NAN;
		_yaw_sp_predicted = _wrap_pi(_yaw_sp_predicted + _yaw_rate_sp * _deltatime);
	}
}

void FlightTaskManualAltitude::update_z_setpoints()
{
	if (fabsf(_sticks(2)) < FLT_EPSILON) {
		/* want to hold altitude */
		_vel_sp_z = NAN;
		_pos_sp_z = _pos_sp_predicted;

	} else {
		/* want to change altitude through velocity */
		_pos_sp_z = NAN;
		_pos_sp_predicted = _position(2) + _vel_sp_z * _deltatime;
	}
}

void FlightTaskManualAltitude::update_setpoints()
{
	update_heading_setpoints();
	update_z_setpoints();
}

bool FlightTaskManualAltitude::update()
{
	scale_sticks();
	update_setpoints();


	_setPositionSetpoint(Vector3f(NAN, NAN, _pos_sp_z));
	_setVelocitySetpoint(Vector3f(NAN, NAN, _vel_sp_z));
	_setYawSetpoint(_yaw_sp);
	_setYawspeedSetpoint(_yaw_rate_sp);

	return true;
}
