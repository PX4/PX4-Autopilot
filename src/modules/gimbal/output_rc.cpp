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


#include "output_rc.h"

#include <uORB/topics/gimbal_controls.h>
#include <px4_platform_common/defines.h>
#include <matrix/matrix/math.hpp>

using math::constrain;

namespace gimbal
{

OutputRC::OutputRC(const Parameters &parameters)
	: OutputBase(parameters)
{
}

void OutputRC::update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id)
{
	if (new_setpoints) {
		_set_angle_setpoints(control_data);
	}

	_handle_position_update(control_data);

	hrt_abstime t = hrt_absolute_time();
	_calculate_angle_output(t);

	// If the output is RC, then it means we are also the gimbal device. gimbal_device_id = (uint8_t)_parameters.mnt_mav_compid_v1;

	// _angle_outputs are in radians, gimbal_controls are in [-1, 1]
	gimbal_controls_s gimbal_controls{};
	auto roll_limit = math::radians(_parameters.mnt_range_roll / 2.0f);
	auto roll = (_angle_outputs[0] + math::radians(_parameters.mnt_off_roll)) * (1.0f / roll_limit);

	if (roll < -1.0f) {
		_angle_outputs[0] = -roll_limit;

	} else if (roll > 1.0f) {
		_angle_outputs[0] = roll_limit;
	}

	gimbal_controls.control[gimbal_controls_s::INDEX_ROLL] = constrain(roll, -1.f, 1.f);
	auto pitch_limit = math::radians(_parameters.mnt_range_pitch / 2.0f);
	auto pitch = (_angle_outputs[1] + math::radians(_parameters.mnt_off_pitch)) * (1.0f / pitch_limit);

	if (pitch < -1.0f) {
		_angle_outputs[1] = -pitch_limit;

	} else if (pitch > 1.0f) {
		_angle_outputs[1] = pitch_limit;
	}

	gimbal_controls.control[gimbal_controls_s::INDEX_PITCH] = constrain(pitch, -1.f, 1.f);
	auto yaw_limit = math::radians(_parameters.mnt_range_yaw / 2.0f);
	auto yaw = (_angle_outputs[2] + math::radians(_parameters.mnt_off_yaw)) * (1.0f / yaw_limit);

	if (yaw < -1.0f) {
		_angle_outputs[2] = -yaw_limit;

	} else if (yaw > 1.0f) {
		_angle_outputs[2] = yaw_limit;
	}

	gimbal_controls.control[gimbal_controls_s::INDEX_YAW] = constrain(yaw, -1.f, 1.f);
	gimbal_controls.timestamp = hrt_absolute_time();
	_gimbal_controls_pub.publish(gimbal_controls);

	_stream_device_attitude_status();

	_last_update = t;
}

void OutputRC::print_status() const
{
	PX4_INFO("Output: AUX");
}

void OutputRC::_stream_device_attitude_status()
{
	gimbal_device_attitude_status_s attitude_status{};
	attitude_status.timestamp = hrt_absolute_time();
	attitude_status.target_system = 0;
	attitude_status.target_component = 0;
	attitude_status.device_flags = gimbal_device_attitude_status_s::DEVICE_FLAGS_NEUTRAL |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_ROLL_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_PITCH_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK;

	matrix::Eulerf euler(_angle_outputs[0], _angle_outputs[1], _angle_outputs[2]);
	matrix::Quatf q(euler);

	// Adjust the angles if stabilization is being applied to represent the gimbal attitude in earth frame
	// to comply with Mavlink 2 GIMBAL_DEVICE_ATTITUDE_STATUS definition
	vehicle_attitude_s vehicle_attitude;
	matrix::Eulerf euler_vehicle{};

	if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
		euler_vehicle = matrix::Quatf(vehicle_attitude.q);
	}

	const matrix::Quatf q_vehicle(matrix::Eulerf(
					      (_stabilize[0]) ? euler_vehicle(0) : 0.0f,
					      (_stabilize[1]) ? euler_vehicle(1) : 0.0f,
					      (_stabilize[2]) ? euler_vehicle(2) : 0.0f));
	q = q_vehicle * q;

	q.copyTo(attitude_status.q);

	attitude_status.failure_flags = 0;
	_attitude_status_pub.publish(attitude_status);
}

} /* namespace gimbal */
