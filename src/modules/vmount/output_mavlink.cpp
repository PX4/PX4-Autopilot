/****************************************************************************
*
*   Copyright (c) 2016-2020 PX4 Development Team. All rights reserved.
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
 * @file output_mavlink.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output_mavlink.h"

#include <math.h>
#include <matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_command.h>
#include <px4_platform_common/defines.h>


namespace vmount
{

OutputMavlinkV1::OutputMavlinkV1(const OutputConfig &output_config)
	: OutputBase(output_config)
{
}

int OutputMavlinkV1::update(const ControlData *control_data)
{
	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.target_system = (uint8_t)_config.mavlink_sys_id;
	vehicle_command.target_component = (uint8_t)_config.mavlink_comp_id;

	if (control_data) {
		//got new command
		_set_angle_setpoints(control_data);

		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
		vehicle_command.timestamp = hrt_absolute_time();

		if (control_data->type == ControlData::Type::Neutral) {
			vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;

			vehicle_command.param5 = 0.0;
			vehicle_command.param6 = 0.0;
			vehicle_command.param7 = 0.0f;

		} else {
			vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;

			vehicle_command.param5 = static_cast<double>(control_data->type_data.angle.frames[0]);
			vehicle_command.param6 = static_cast<double>(control_data->type_data.angle.frames[1]);
			vehicle_command.param7 = static_cast<float>(control_data->type_data.angle.frames[2]);
		}

		vehicle_command.param2 = _stabilize[0] ? 1.0f : 0.0f;
		vehicle_command.param3 = _stabilize[1] ? 1.0f : 0.0f;
		vehicle_command.param4 = _stabilize[2] ? 1.0f : 0.0f;

		_vehicle_command_pub.publish(vehicle_command);
	}

	_handle_position_update();

	hrt_abstime t = hrt_absolute_time();
	_calculate_output_angles(t);

	vehicle_command.timestamp = t;
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;

	// vmount spec has roll, pitch on channels 0, 1, respectively; MAVLink spec has roll, pitch on channels 1, 0, respectively
	// vmount uses radians, MAVLink uses degrees
	vehicle_command.param1 = (_angle_outputs[1] + _config.pitch_offset) * M_RAD_TO_DEG_F;
	vehicle_command.param2 = (_angle_outputs[0] + _config.roll_offset) * M_RAD_TO_DEG_F;
	vehicle_command.param3 = (_angle_outputs[2] + _config.yaw_offset) * M_RAD_TO_DEG_F;
	vehicle_command.param7 = 2.0f; // MAV_MOUNT_MODE_MAVLINK_TARGETING;

	_vehicle_command_pub.publish(vehicle_command);

	_last_update = t;

	return 0;
}

void OutputMavlinkV1::print_status()
{
	PX4_INFO("Output: MAVLink gimbal protocol v1");
}

OutputMavlinkV2::OutputMavlinkV2(const OutputConfig &output_config)
	: OutputBase(output_config)
{
}

int OutputMavlinkV2::update(const ControlData *control_data)
{
	if (control_data) {
		//got new command
		_set_angle_setpoints(control_data);
		_publish_gimbal_device_set_attitude(control_data);
	}

	_handle_position_update();

	hrt_abstime t = hrt_absolute_time();
	_calculate_output_angles(t);

	_last_update = t;

	return 0;
}

void OutputMavlinkV2::print_status()
{
	PX4_INFO("Output: MAVLink gimbal protocol v2");
}

void OutputMavlinkV2::_publish_gimbal_device_set_attitude(const ControlData *control_data)
{
	gimbal_device_set_attitude_s set_attitude{};
	set_attitude.timestamp = hrt_absolute_time();
	set_attitude.target_system = (uint8_t)_config.mavlink_sys_id;
	set_attitude.target_component = (uint8_t)_config.mavlink_comp_id;

	matrix::Eulerf euler(control_data->type_data.angle.angles[0], control_data->type_data.angle.angles[1],
			     control_data->type_data.angle.angles[2]);
	matrix::Quatf q(euler);

	set_attitude.q[0] = q(0);
	set_attitude.q[1] = q(1);
	set_attitude.q[2] = q(2);
	set_attitude.q[3] = q(3);


	if (control_data->type_data.angle.frames[0] == ControlData::TypeData::TypeAngle::Frame::AngularRate) {
		set_attitude.angular_velocity_x = control_data->type_data.angle.angles[0]; //roll
	}

	if (control_data->type_data.angle.frames[1] == ControlData::TypeData::TypeAngle::Frame::AngularRate) {
		set_attitude.angular_velocity_y = control_data->type_data.angle.angles[1]; //pitch

	}

	if (control_data->type_data.angle.frames[2] == ControlData::TypeData::TypeAngle::Frame::AngularRate) {
		set_attitude.angular_velocity_z = control_data->type_data.angle.angles[2];
	}

	if (control_data->type == ControlData::Type::Neutral) {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_NEUTRAL;
	}

	if (control_data->type_data.angle.frames[0] == ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame) {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_ROLL_LOCK;
	}

	if (control_data->type_data.angle.frames[1] == ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame) {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_PITCH_LOCK;
	}

	if (control_data->type_data.angle.frames[2] == ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame) {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_LOCK;
	}

	_gimbal_device_set_attitude_pub.publish(set_attitude);

}

} /* namespace vmount */
