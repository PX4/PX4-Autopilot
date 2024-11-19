/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "StickTiltXY.hpp"

#include <geo/geo.h>
#include "Sticks.hpp"

using namespace matrix;

StickTiltXY::StickTiltXY(ModuleParams *parent) :
	ModuleParams(parent)
{
	updateParams();
}

void StickTiltXY::updateParams()
{
	ModuleParams::updateParams();
	// Consider maximum tilt but only between [0.02,3]g sideways acceleration -> ~[1,71]° tilt
	// Constrain tilt already because tanf(90+°) will give negative result
	const float maximum_tilt = math::radians(math::constrain(_param_mpc_man_tilt_max.get(), 0.f, 89.f));
	_maximum_acceleration = math::constrain(tanf(maximum_tilt), .02f, 3.f) * CONSTANTS_ONE_G;
}

Vector2f StickTiltXY::generateAccelerationSetpoints(Vector2f stick_xy, const float dt, const float yaw,
		const float yaw_setpoint)
{
	Sticks::limitStickUnitLengthXY(stick_xy);
	_man_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	stick_xy = _man_input_filter.update(stick_xy);
	Sticks::rotateIntoHeadingFrameXY(stick_xy, yaw, yaw_setpoint);
	return stick_xy * _maximum_acceleration;
}
