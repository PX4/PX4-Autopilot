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
 * @file FlightManualAltitude.cpp
 */

#include "FlightTaskManualAltitude.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskManualAltitude::FlightTaskManualAltitude(control::SuperBlock *parent, const char *name) :
	FlightTaskManualStabilized(parent, name),
	_vel_max_down(parent, "MPC_Z_VEL_MAX_DN", false),
	_vel_max_up(parent, "MPC_Z_VEL_MAX_UP", false),
	_vel_z_dz(parent, "MPC_HOLD_MAX_Z", false)

{}

bool FlightTaskManualAltitude::activate()
{
	_pos_sp_z = NAN;
	_vel_sp_z = 0.0f;
	return FlightTaskManualStabilized::activate();
}

void FlightTaskManualAltitude::_scaleSticks()
{
	/* Reuse same scaling as for stabilized */
	FlightTaskManualStabilized::_scaleSticks();

	/* Scale horizontal velocity with expo curve stick input*/
	const float vel_max_z = (_sticks(2) > 0.0f) ? _vel_max_down.get() : _vel_max_up.get();
	_vel_sp_z = vel_max_z * _sticks_expo(2);
}

void FlightTaskManualAltitude::_updateZsetpoints()
{
	/* Depending on stick inputs, position is locked or
	 * velocity setpoint is used. If not locked, position
	 * setpoints is set to NAN.
	 */

	/* handle position and altitude hold */
	const bool stick_z_zero = fabsf(_sticks_expo(2)) <= FLT_EPSILON;
	const bool stopped_z = (_vel_z_dz.get() < FLT_EPSILON || fabsf(_velocity(2)) < _vel_z_dz.get());

	if (stick_z_zero && stopped_z && !PX4_ISFINITE(_pos_sp_z)) {
		_pos_sp_z = _position(2);

	} else if (!stick_z_zero) {
		_pos_sp_z = NAN;
	}
}

void FlightTaskManualAltitude::_updateSetpoints()
{
	FlightTaskManualStabilized::_updateSetpoints(); // get yaw and thrust setpoints
	_updateZsetpoints(); // get z setpoints
}

bool FlightTaskManualAltitude::update()
{
	_scaleSticks();
	_updateSetpoints();

	_setPositionSetpoint(Vector3f(NAN, NAN, _pos_sp_z));
	_setVelocitySetpoint(Vector3f(NAN, NAN, _vel_sp_z));
	_setYawSetpoint(_yaw_sp);
	_setYawspeedSetpoint(_yaw_rate_sp);
	_setThrustSetpoint(Vector3f(_thr_sp(0), _thr_sp(1), NAN));

	return true;
}
