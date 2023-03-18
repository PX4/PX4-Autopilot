/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskDescend.cpp
 */

#include "FlightTaskDescend.hpp"

bool FlightTaskDescend::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_gear.landing_gear = landing_gear_s::GEAR_DOWN;
	return ret;
}

bool FlightTaskDescend::update()
{
	bool ret = FlightTask::update();

	if (PX4_ISFINITE(_velocity(2))) {
		// land with landspeed
		_velocity_setpoint(2) = _param_mpc_land_speed.get();
		_acceleration_setpoint(2) = NAN;

	} else {
		// descend with constant acceleration (crash landing)
		_velocity_setpoint(2) = NAN;
		_acceleration_setpoint(2) = .15f;
	}

	// Nudging
	if (_param_mpc_land_rc_help.get() && _sticks.checkAndUpdateStickInputs()) {
		_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, _sticks.getYawExpo(), _yaw,
					       _is_yaw_good_for_control, _deltatime);
		_acceleration_setpoint.xy() = _stick_tilt_xy.generateAccelerationSetpoints(_sticks.getPitchRoll(), _deltatime, _yaw,
					      _yaw_setpoint);

		// Stick full up -1 -> stop, stick full down 1 -> double the value
		_velocity_setpoint(2) *= (1 - _sticks.getThrottleZeroCenteredExpo());
		_acceleration_setpoint(2) -= _sticks.getThrottleZeroCentered() * 10.f;

	} else {
		_acceleration_setpoint = matrix::Vector3f(0.f, 0.f, NAN); // stay level to minimize horizontal drift
		_yawspeed_setpoint = NAN;

		// keep heading
		if (!PX4_ISFINITE(_yaw_setpoint)) {
			_yaw_setpoint = _yaw;
		}
	}

	return ret;
}
