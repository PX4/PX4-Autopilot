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
 * @file FlightTaskTranstion.cpp
 */

#include "FlightTaskTransition.hpp"
#include "Sticks.hpp"

FlightTaskTransition::FlightTaskTransition()
{
	_param_handle_pitch_cruise_degrees = param_find("FW_PSP_OFF");

	if (_param_handle_pitch_cruise_degrees != PARAM_INVALID) {
		param_get(_param_handle_pitch_cruise_degrees, &_param_pitch_cruise_degrees);
	}

}

bool FlightTaskTransition::updateInitialize()
{

	updateParameters();
	return FlightTask::updateInitialize();
}

void FlightTaskTransition::updateParameters()
{
// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		if (_param_handle_pitch_cruise_degrees != PARAM_INVALID) {
			param_get(_param_handle_pitch_cruise_degrees, &_param_pitch_cruise_degrees);
		}
	}
}

bool FlightTaskTransition::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);

	_vel_z_filter.setParameters(math::constrain(_deltatime, 0.01f, 0.1f), _vel_z_filter_time_const);

	if (PX4_ISFINITE(last_setpoint.vz)) {
		_vel_z_filter.reset(last_setpoint.vz);

	} else {
		_vel_z_filter.reset(_velocity(2));
	}

	_velocity_setpoint(2) = _vel_z_filter.getState();

	return ret;


}

bool FlightTaskTransition::update()
{
	// tailsitters will override attitude and thrust setpoint
	// tiltrotors and standard vtol will overrride roll and pitch setpoint but keep vertical thrust setpoint
	bool ret = FlightTask::update();

	_position_setpoint.setAll(NAN);

	// calculate a horizontal acceleration vector which corresponds to an attitude composed of pitch up by _param_pitch_cruise_degrees
	// and zero roll angle
	matrix::Vector2f tmp(-1.0f, 0.0f);
	Sticks::rotateIntoHeadingFrameXY(tmp, _yaw, NAN);
	_acceleration_setpoint.xy() = tmp * tanf(math::radians(_param_pitch_cruise_degrees)) * CONSTANTS_ONE_G;

	// slowly move vertical velocity setpoint to zero
	_vel_z_filter.setParameters(math::constrain(_deltatime, 0.01f, 0.1f), _vel_z_filter_time_const);
	_velocity_setpoint(2) = _vel_z_filter.update(0.0f);

	_yaw_setpoint = NAN;
	return ret;
}
