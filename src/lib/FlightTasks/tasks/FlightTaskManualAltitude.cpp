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
#include <float.h>

using namespace matrix;

FlightTaskManualAltitude::FlightTaskManualAltitude(control::SuperBlock *parent, const char *name) :
	FlightTaskManualStabilized(parent, name),
	_vel_max_down(parent, "MPC_Z_VEL_MAX_DN", false),
	_vel_max_up(parent, "MPC_Z_VEL_MAX_UP", false),
	_vel_hold_thr_z(parent, "MPC_HOLD_MAX_Z", false)

{}

float FlightTaskManualAltitude::calcAltSetpoint(float climb_rate_setpoint)
{
	/* Depending on stick inputs and velocity, position is locked.
	 * If not locked, altitude setpoint is set to NAN.
	 */

	/* handle position and altitude hold */
	const bool apply_brake_z = fabsf(climb_rate_setpoint) <= FLT_EPSILON;
	const bool stopped_z = (_vel_hold_thr_z.get() < FLT_EPSILON || fabsf(_velocity(2)) < _vel_hold_thr_z.get());

	if (apply_brake_z && stopped_z && !PX4_ISFINITE(_last_altitude_setpoint)) {
		_last_altitude_setpoint = _position(2);

	} else if (!apply_brake_z) {
		_last_altitude_setpoint = NAN;
	}

	return _last_altitude_setpoint;
}

float FlightTaskManualAltitude::calcClimbRateSetpoint()
{
	/* Scale horizontal velocity with expo curve stick input*/
	const float vel_max_z = (_sticks(2) > 0.0f) ? _vel_max_down.get() : _vel_max_up.get();
	return vel_max_z * _sticks_expo(2);
}

void
FlightTaskManualAltitude::calcYawAndThrustXY(ControlSetpoint &setpoint)
{
	setpoint.yawspeed_setpoint = calcYawSpeedSetpoint();
	float yaw_setpoint = calcYawSetpoint();

	Vector2f sp{_sticks(0), _sticks(1)};
	_rotateIntoHeadingFrame(sp, yaw_setpoint);

	if (sp.length() > 1.0f) {
		sp.normalize();
	}

	setpoint.thrust_setpoint(0) = sp(0);
	setpoint.thrust_setpoint(1) = sp(1);
}

void
FlightTaskManualAltitude::initialiseOutputs(ControlSetpoint &setpoint)
{
	setpoint.thrust_setpoint(2) = NAN;
}

void FlightTaskManualAltitude::updateOutput(ControlSetpoint &setpoint)
{
	calcYawAndThrustXY(setpoint);

	float climb_rate_setpoint = calcClimbRateSetpoint();
	setpoint.velocity_setpoint(2) = climb_rate_setpoint;
	setpoint.position_setpoint(2) = calcAltSetpoint(climb_rate_setpoint);
}
