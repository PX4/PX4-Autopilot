/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file Sticks.cpp
 */

#include "Sticks.hpp"

using namespace time_literals;
using namespace matrix;

Sticks::Sticks(ModuleParams *parent) :
	ModuleParams(parent)
{}

bool Sticks::checkAndUpdateStickInputs()
{
	// Sticks are rescaled linearly and exponentially to [-1,1]
	manual_control_setpoint_s manual_control_setpoint;

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		// Linear scale
		_positions(0) = manual_control_setpoint.pitch;
		_positions(1) = manual_control_setpoint.roll;
		_positions(2) = -manual_control_setpoint.throttle;
		_positions(3) = manual_control_setpoint.yaw;

		// Exponential scale
		_positions_expo(0) = math::expo_deadzone(_positions(0), _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		_positions_expo(1) = math::expo_deadzone(_positions(1), _param_mpc_xy_man_expo.get(), _param_mpc_hold_dz.get());
		_positions_expo(2) = math::expo_deadzone(_positions(2), _param_mpc_z_man_expo.get(),  _param_mpc_hold_dz.get());
		_positions_expo(3) = math::expo_deadzone(_positions(3), _param_mpc_yaw_expo.get(),    _param_mpc_hold_dz.get());

		_aux_positions(0) = manual_control_setpoint.aux1;
		_aux_positions(1) = manual_control_setpoint.aux2;
		_aux_positions(2) = manual_control_setpoint.aux3;
		_aux_positions(3) = manual_control_setpoint.aux4;
		_aux_positions(4) = manual_control_setpoint.aux5;
		_aux_positions(5) = manual_control_setpoint.aux6;

		// valid stick inputs are required
		_input_available = manual_control_setpoint.valid && _positions.isAllFinite();

	} else {
		failsafe_flags_s failsafe_flags;

		if (_failsafe_flags_sub.update(&failsafe_flags)) {
			if (failsafe_flags.manual_control_signal_lost) {
				_input_available = false;
			}
		}
	}

	if (!_input_available) {
		// Timeout: set all sticks to zero
		_positions.zero();
		_positions_expo.zero();
	}

	return _input_available;
}

void Sticks::limitStickUnitLengthXY(Vector2f &v)
{
	const float vl = v.length();

	if (vl > 1.0f) {
		v /= vl;
	}
}

void Sticks::rotateIntoHeadingFrameXY(Vector2f &v, const float yaw, const float yaw_setpoint)
{
	const float yaw_rotate = PX4_ISFINITE(yaw_setpoint) ? yaw_setpoint : yaw;
	v = Dcm2f(yaw_rotate) * v;
}
