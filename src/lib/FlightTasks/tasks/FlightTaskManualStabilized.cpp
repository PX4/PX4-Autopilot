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
 * @file FlightManualStabilized.cpp
 *
 * Flight task for manual controlled attitude.
 * It generates thrust and yaw setpoints.
 * TODO: add thrust
 */

#include "FlightTaskManualStabilized.hpp"
#include <mathlib/mathlib.h>
#include <float.h>
#include <lib/geo/geo.h>

using namespace matrix;

FlightTaskManualStabilized::FlightTaskManualStabilized(control::SuperBlock *parent, const char *name) :
	FlightTaskManual(parent, name),
	_yaw_rate_scaling(parent, "MPC_MAN_Y_MAX", false)
{}

bool FlightTaskManualStabilized::activate()
{
	_yaw_sp = _yaw;
	_yaw_rate_sp = 0.0f;
	_sign_speed = 1.0f;
	return FlightTaskManual::activate();
}

void FlightTaskManualStabilized::_scaleSticks()
{
	/* Scale sticks to yaw and thrust using
	 * linear scale.
	 * TODO: add thrust */
	_yaw_rate_sp = _sticks(3) * math::radians(_yaw_rate_scaling.get());
}

void FlightTaskManualStabilized::_updateHeadingSetpoints()
{
	/* Yaw-lock depends on stick input. If locked,
	 * yawspeed_sp is set to NAN. Otherwise yaw_sp is set
	 * to NAN.*/
	const bool stick_yaw_zero = fabsf(_sticks(3)) <= _stick_dz.get();

	if (stick_yaw_zero && !PX4_ISFINITE(_yaw_sp)) {
		_yaw_sp = _wrap_pi(_yaw + _sign_speed * _yaw_rate_scaling.get() * _deltatime);

	} else if (!stick_yaw_zero) {
		_yaw_sp = NAN;
		_sign_speed = math::sign(_yaw_rate_sp);
	}
}

void FlightTaskManualStabilized::_updateThrustSetpoints()
{
	// TODO
}

void FlightTaskManualStabilized::_updateSetpoints()
{
	_updateHeadingSetpoints();
	//_updateThrustSetpoints();
}

bool FlightTaskManualStabilized::update()
{
	_scaleSticks(); // scales sticks linearly to yaw/yawspeed and thrust
	_updateSetpoints(); // applies yaw and position lock if required
	_setYawSetpoint(_yaw_sp);
	_setYawspeedSetpoint(_yaw_rate_sp);
	//_setThrustSetpoint(...) TODO

	return true;
}
