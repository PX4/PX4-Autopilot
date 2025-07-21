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


#include "output_mavlink.h"

#include <matrix/matrix/math.hpp>

#include <px4_platform_common/defines.h>


namespace gimbal
{

OutputMavlinkV1::OutputMavlinkV1(const Parameters &parameters)
	: OutputBase(parameters)
{}

void OutputMavlinkV1::update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id)
{
	hrt_abstime now = hrt_absolute_time();

	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.target_system = (uint8_t)_parameters.mnt_mav_sysid_v1;
	vehicle_command.target_component = (uint8_t)_parameters.mnt_mav_compid_v1;

	if (new_setpoints) {
		//got new command
		_set_angle_setpoints(control_data);

		const bool configuration_changed =
			(control_data.type != _previous_control_data_type);
		_previous_control_data_type = control_data.type;

		if (configuration_changed) {

			vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
			vehicle_command.timestamp = hrt_absolute_time();

			if (control_data.type == ControlData::Type::Neutral) {
				vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;

				vehicle_command.param5 = 0.0;
				vehicle_command.param6 = 0.0;
				vehicle_command.param7 = 0.0f;

			} else {
				vehicle_command.param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;

				vehicle_command.param5 = static_cast<double>(control_data.type_data.angle.frames[0]);
				vehicle_command.param6 = static_cast<double>(control_data.type_data.angle.frames[1]);
				vehicle_command.param7 = static_cast<float>(control_data.type_data.angle.frames[2]);
			}

			vehicle_command.param2 = _stabilize[0] ? 1.0f : 0.0f;
			vehicle_command.param3 = _stabilize[1] ? 1.0f : 0.0f;
			vehicle_command.param4 = _stabilize[2] ? 1.0f : 0.0f;

			_gimbal_v1_command_pub.publish(vehicle_command);
		}
	}

	_handle_position_update(control_data);

	_calculate_angle_output(now);

	vehicle_command.timestamp = now;
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;

	// gimbal spec has roll, pitch on channels 0, 1, respectively; MAVLink spec has roll, pitch on channels 1, 0, respectively
	// gimbal uses radians, MAVLink uses degrees
	vehicle_command.param1 = math::degrees(_angle_outputs[1] + math::radians(_parameters.mnt_off_pitch));
	vehicle_command.param2 = math::degrees(_angle_outputs[0] + math::radians(_parameters.mnt_off_roll));
	vehicle_command.param3 = math::degrees(_angle_outputs[2] + math::radians(_parameters.mnt_off_yaw));
	vehicle_command.param7 = 2.0f; // MAV_MOUNT_MODE_MAVLINK_TARGETING;

	_gimbal_v1_command_pub.publish(vehicle_command);

	_stream_device_attitude_status();

	// If the output is MAVLink v1, then we signal this by referring to compid 1.
	gimbal_device_id = 1;

	_last_update = now;
}

void OutputMavlinkV1::_stream_device_attitude_status()
{
	// This enables the use case where the gimbal v2 protocol is used
	// between the ground station and the drone, and the gimbal v1 protocol is
	// used between the drone and the gimbal.
	gimbal_device_attitude_status_s attitude_status{};
	attitude_status.timestamp = hrt_absolute_time();
	attitude_status.target_system = 0;
	attitude_status.target_component = 0;
	attitude_status.device_flags = gimbal_device_attitude_status_s::DEVICE_FLAGS_NEUTRAL |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_ROLL_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_PITCH_LOCK;

	matrix::Eulerf euler(_angle_outputs[0], _angle_outputs[1], _angle_outputs[2]);
	matrix::Quatf q(euler);
	q.copyTo(attitude_status.q);

	attitude_status.failure_flags = 0;
	_attitude_status_pub.publish(attitude_status);
}

void OutputMavlinkV1::print_status() const
{
	PX4_INFO("Output: MAVLink gimbal protocol v1");
}

OutputMavlinkV2::OutputMavlinkV2(const Parameters &parameters)
	: OutputBase(parameters)
{
}

void OutputMavlinkV2::update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id)
{
	hrt_abstime now = hrt_absolute_time();

	_check_for_gimbal_device_information();

	if (!_gimbal_device_found && now - _last_gimbal_device_checked > 1000000) {
		_request_gimbal_device_information();
		_last_gimbal_device_checked = now;

	} else {
		if (new_setpoints) {
			//got new command
			_set_angle_setpoints(control_data);

			_handle_position_update(control_data);
			_last_update = now;
		}

		gimbal_device_id = _gimbal_device_found ? _gimbal_device_id : 0;

		_publish_gimbal_device_set_attitude();
	}
}

void OutputMavlinkV2::_request_gimbal_device_information()
{
	vehicle_command_s vehicle_cmd{};
	vehicle_cmd.timestamp = hrt_absolute_time();
	vehicle_cmd.command = vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE;
	vehicle_cmd.param1 = vehicle_command_s::VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION;
	vehicle_cmd.target_system = 0;
	vehicle_cmd.target_component = 0;
	vehicle_cmd.source_system = _parameters.mav_sysid;
	vehicle_cmd.source_component = _parameters.mav_compid;
	vehicle_cmd.confirmation = 0;
	vehicle_cmd.from_external = false;

	uORB::Publication<vehicle_command_s> vehicle_command_pub{ORB_ID(vehicle_command)};
	vehicle_command_pub.publish(vehicle_cmd);
}

void OutputMavlinkV2::_check_for_gimbal_device_information()
{
	gimbal_device_information_s gimbal_device_information;

	if (_gimbal_device_information_sub.update(&gimbal_device_information)) {
		_gimbal_device_found = true;
		_gimbal_device_id = gimbal_device_information.gimbal_device_id;
	}
}

void OutputMavlinkV2::print_status() const
{
	PX4_INFO("Output: MAVLink gimbal protocol v2");

	PX4_INFO_RAW("  quaternion: [%.1f %.1f %.1f %.1f]\n",
		     (double)_q_setpoint[0],
		     (double)_q_setpoint[1],
		     (double)_q_setpoint[2],
		     (double)_q_setpoint[3]);
	PX4_INFO_RAW("  angular velocity: [%.1f %.1f %.1f]\n",
		     (double)_angle_velocity[0],
		     (double)_angle_velocity[1],
		     (double)_angle_velocity[2]);

	if (_gimbal_device_found) {
		PX4_INFO_RAW("  gimbal device compid found: %d\n", _gimbal_device_id);

	} else {
		PX4_INFO_RAW("  gimbal device compid not found\n");
	}
}

void OutputMavlinkV2::_publish_gimbal_device_set_attitude()
{
	gimbal_device_set_attitude_s set_attitude{};
	set_attitude.timestamp = hrt_absolute_time();
	set_attitude.target_system = (uint8_t)_parameters.mav_sysid;
	set_attitude.target_component = _gimbal_device_id;

	set_attitude.angular_velocity_x = _angle_velocity[0];
	set_attitude.angular_velocity_y = _angle_velocity[1];
	set_attitude.angular_velocity_z = _angle_velocity[2];
	set_attitude.q[0] = _q_setpoint[0];
	set_attitude.q[1] = _q_setpoint[1];
	set_attitude.q[2] = _q_setpoint[2];
	set_attitude.q[3] = _q_setpoint[3];

	if (_absolute_angle[0]) {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_ROLL_LOCK;
	}

	if (_absolute_angle[1]) {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_PITCH_LOCK;
	}

	if (_absolute_angle[2]) {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_LOCK;
	}

	_gimbal_device_set_attitude_pub.publish(set_attitude);
}

} /* namespace gimbal */
