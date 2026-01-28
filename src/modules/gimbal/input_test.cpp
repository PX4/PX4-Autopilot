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
#include <px4_platform_common/log.h>
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
	control_data.timestamp_last_update = hrt_absolute_time();

	if (PX4_ISFINITE(_roll_deg) && PX4_ISFINITE(_pitch_deg) && PX4_ISFINITE(_yaw_deg)) {
		control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
		control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
		control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
		matrix::Eulerf euler(
			math::radians((float)_roll_deg),
			math::radians((float)_pitch_deg),
			math::radians((float)_yaw_deg));
		matrix::Quatf q(euler);
		q.copyTo(control_data.type_data.angle.q);

	} else {
		control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngularRate;
		control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngularRate;
		control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngularRate;
		control_data.type_data.angle.q[0] = NAN;
		control_data.type_data.angle.q[1] = NAN;
		control_data.type_data.angle.q[2] = NAN;
		control_data.type_data.angle.q[3] = NAN;
		control_data.type_data.angle.angular_velocity[0] = math::radians(_rollrate_deg_s);
		control_data.type_data.angle.angular_velocity[1] = math::radians(_pitchrate_deg_s);
		control_data.type_data.angle.angular_velocity[2] = math::radians(_yawrate_deg_s);
	}


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
	PX4_INFO_RAW("  roll : % .1f deg\n", (double)_roll_deg);
	PX4_INFO_RAW("  pitch: % .1f deg\n", (double)_pitch_deg);
	PX4_INFO_RAW("  yaw  : % .1f deg\n", (double)_yaw_deg);
	PX4_INFO_RAW("  rollrate : % .1f deg/s\n", (double)_rollrate_deg_s);
	PX4_INFO_RAW("  pitchrate: % .1f deg/s\n", (double)_pitchrate_deg_s);
	PX4_INFO_RAW("  yawrate  : % .1f deg/s\n", (double)_yawrate_deg_s);
}

void InputTest::set_test_input_angles(float roll_deg, float pitch_deg, float yaw_deg)
{
	_roll_deg = roll_deg;
	_pitch_deg = pitch_deg;
	_yaw_deg = yaw_deg;
	_rollrate_deg_s = NAN;
	_pitchrate_deg_s = NAN;
	_yawrate_deg_s = NAN;
	_has_been_set.store(true);
}
void InputTest::set_test_input_angle_rates(float rollrate_deg_s, float pitchrate_deg_s, float yawrate_deg_s)
{
	_roll_deg = NAN;
	_pitch_deg = NAN;
	_yaw_deg = NAN;
	_rollrate_deg_s = rollrate_deg_s;
	_pitchrate_deg_s = pitchrate_deg_s;
	_yawrate_deg_s = yawrate_deg_s;
	_has_been_set.store(true);
}

} /* namespace gimbal */
