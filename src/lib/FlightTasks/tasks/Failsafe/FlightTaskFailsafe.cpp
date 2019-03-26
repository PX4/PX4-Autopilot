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
 * @file FlightTaskFailsafe.cpp
 */

#include "FlightTaskFailsafe.hpp"

bool FlightTaskFailsafe::activate()
{
	bool ret = FlightTask::activate();
	_position_setpoint = _position;
	_velocity_setpoint.zero();
	_thrust_setpoint = matrix::Vector3f(0.0f, 0.0f, -_param_mpc_thr_hover.get() * 0.6f);
	_yaw_setpoint = _yaw;
	_yawspeed_setpoint = 0.0f;
	return ret;
}

bool FlightTaskFailsafe::update()
{
	if (PX4_ISFINITE(_position(0)) && PX4_ISFINITE(_position(1))) {
		// stay at current position setpoint
		_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
		_thrust_setpoint(0) = _thrust_setpoint(1) = 0.0f;

	} else if (PX4_ISFINITE(_velocity(0)) && PX4_ISFINITE(_velocity(1))) {
		// don't move along xy
		_position_setpoint(0) = _position_setpoint(1) = NAN;
		_thrust_setpoint(0) = _thrust_setpoint(1) = NAN;
	}

	if (PX4_ISFINITE(_position(2))) {
		// stay at current altitude setpoint
		_velocity_setpoint(2) = 0.0f;
		_thrust_setpoint(2) = NAN;

	} else if (PX4_ISFINITE(_velocity(2))) {
		// land with landspeed
		_velocity_setpoint(2) = _param_mpc_land_speed.get();
		_position_setpoint(2) = NAN;
		_thrust_setpoint(2) = NAN;
	}

	return true;

}
