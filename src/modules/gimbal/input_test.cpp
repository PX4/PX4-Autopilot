/****************************************************************************
*
*   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

#include "input_test.h"

#include <px4_platform_common/posix.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>


namespace gimbal
{

InputTest::InputTest(Parameters &parameters) :
	InputBase(parameters)
{}

InputTest::UpdateResult InputTest::update(unsigned int timeout_ms, ControlData &control_data, bool already_active)
{
	if (!_has_been_set.load()) {
		return UpdateResult::NoUpdate;
	}

	control_data.type = ControlData::Type::Angle;

	control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
	control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
	control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

	matrix::Eulerf euler(
		math::radians((float)_roll_deg),
		math::radians((float)_pitch_deg),
		math::radians((float)_yaw_deg));
	matrix::Quatf q(euler);

	q.copyTo(control_data.type_data.angle.q);

	control_data.gimbal_shutter_retract = false;

	control_data.type_data.angle.angular_velocity[0] = NAN;
	control_data.type_data.angle.angular_velocity[1] = NAN;
	control_data.type_data.angle.angular_velocity[2] = NAN;

	// For testing we mark ourselves as in control.
	control_data.sysid_primary_control = _parameters.mav_sysid;
	control_data.compid_primary_control = _parameters.mav_compid;

	_has_been_set.store(false);
	return UpdateResult::UpdatedActive;
}

int InputTest::initialize()
{
	return 0;
}

void InputTest::print_status() const
{
	PX4_INFO("Input: Test");
	PX4_INFO_RAW("  roll : % 3d deg\n", _roll_deg);
	PX4_INFO_RAW("  pitch: % 3d deg\n", _pitch_deg);
	PX4_INFO_RAW("  yaw  : % 3d deg\n", _yaw_deg);
}

void InputTest::set_test_input(int roll_deg, int pitch_deg, int yaw_deg)
{
	_roll_deg = roll_deg;
	_pitch_deg = pitch_deg;
	_yaw_deg = yaw_deg;

	_has_been_set.store(true);
}

} /* namespace gimbal */
