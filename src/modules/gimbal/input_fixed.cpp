/****************************************************************************
*
*   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "input_fixed.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

namespace gimbal
{

InputFixed::InputFixed(Parameters &parameters) :
	InputBase(parameters)
{}

InputFixed::UpdateResult InputFixed::update(unsigned int timeout_ms, ControlData &control_data, bool already_active)
{
	// Unlike the RC/MAVLink inputs there is no event source to poll, so block for
	// the requested timeout to pace the gimbal main loop. Without this the loop
	// never sleeps in fixed mode and busy-spins at 100% CPU, starving other tasks.
	if (timeout_ms > 0) {
		px4_usleep(timeout_ms * 1000);
	}

	// Hold a constant attitude in the world frame: level in roll/yaw and a fixed
	// pitch from MNT_FIXED_PITCH. All axes are requested in the absolute frame so
	// the output stabilizes them against vehicle motion (depending on MNT_DO_STAB).
	// The user cannot influence this: in fixed mode no RC/MAVLink input objects
	// are created.
	control_data.type = ControlData::Type::Angle;
	control_data.timestamp_last_update = hrt_absolute_time();

	control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
	control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
	control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;

	const matrix::Quatf q(matrix::Eulerf(0.f, math::radians(_parameters.mnt_fixed_pitch), 0.f));
	q.copyTo(control_data.type_data.angle.q);

	control_data.type_data.angle.angular_velocity[0] = NAN;
	control_data.type_data.angle.angular_velocity[1] = NAN;
	control_data.type_data.angle.angular_velocity[2] = NAN;

	// Nobody is in control: this attitude is not commandable.
	control_data.sysid_primary_control = 0;
	control_data.compid_primary_control = 0;

	return UpdateResult::UpdatedActive;
}

int InputFixed::initialize()
{
	return 0;
}

void InputFixed::print_status() const
{
	PX4_INFO("Input: Fixed (world-frame stabilized)");
	PX4_INFO_RAW("  pitch: % .1f deg\n", (double)_parameters.mnt_fixed_pitch);
}

} /* namespace gimbal */
