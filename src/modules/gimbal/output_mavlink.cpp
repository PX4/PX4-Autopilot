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

#include <px4_platform_common/events.h>

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
	vehicle_command.param1 = math::degrees(_angle_outputs[1]);
	vehicle_command.param2 = math::degrees(_angle_outputs[0]);
	vehicle_command.param3 = math::degrees(_angle_outputs[2]);
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
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME;

	} else {
		set_attitude.flags |= gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
	}

	_gimbal_device_set_attitude_pub.publish(set_attitude);
}

OutputMavlinkToGimbalManager::OutputMavlinkToGimbalManager(const Parameters &parameters)
	: OutputBase(parameters)
{
}

const char *OutputMavlinkToGimbalManager::_control_right_str(ControlRights control_right)
{
	switch (control_right) {
	case ControlRights::PRIMARY:
		return "PRIMARY";
		break;

	case ControlRights::SECONDARY:
		return "SECONDARY";
		break;

	case ControlRights::NONE:
	default:
		return "NONE";
	}
}

void OutputMavlinkToGimbalManager::update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id)
{
	const hrt_abstime now = hrt_absolute_time();

	gimbal_device_id = 0;

	// Discovery until a valid manager is detected
	_check_for_gimbal_manager_information();

	if (!_have_valid_manager()) {
		if ((now - _last_info_request) >= INFO_REQUEST_PERIOD_US) {
			_last_info_request = now;
			_request_gimbal_manager_information();
		}

		return;
	}

	gimbal_device_id = (_gimbal_device_id != 0) ? _gimbal_device_id : 0;

	// Update control ownership state and process any pending take-control ACK
	_check_for_gimbal_manager_status();
	_check_for_take_control_ack();

	// Only publish a new setpoint when fresh control data is available
	if (!new_setpoints) {
		return;
	}

	// We may publish immediately if PX4 already has control, otherwise try to acquire it
	_can_publish_set_attitude = (_control_rights != ControlRights::NONE) || (_acquire_control_for_autopilot(now));

	if (!_can_publish_set_attitude) { return; }

	// Update the outgoing setpoint and forward it to the external gimbal manager
	_set_angle_setpoints(control_data);
	_handle_position_update(control_data);
	_last_update = now;

	_publish_gimbal_manager_set_attitude();
}

void OutputMavlinkToGimbalManager::_publish_gimbal_manager_set_attitude()
{
	external_gimbal_manager_set_attitude_s msg{};
	msg.timestamp 		= hrt_absolute_time();

	msg.origin_sysid 	= (uint8_t)_parameters.mav_sysid;
	msg.origin_compid 	= (uint8_t)_parameters.mav_compid;

	msg.target_system 	= _manager_sysid;
	msg.target_component 	= _manager_compid;
	msg.gimbal_device_id 	= _gimbal_device_id;

	msg.angular_velocity_x 	= _angle_velocity[0];
	msg.angular_velocity_y 	= _angle_velocity[1];
	msg.angular_velocity_z 	= _angle_velocity[2];

	msg.q[0] = _q_setpoint[0];
	msg.q[1] = _q_setpoint[1];
	msg.q[2] = _q_setpoint[2];
	msg.q[3] = _q_setpoint[3];

	if (_absolute_angle[2]) {
		msg.flags |= external_gimbal_manager_set_attitude_s::EXTERNAL_GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME;

	} else {
		msg.flags |= external_gimbal_manager_set_attitude_s::EXTERNAL_GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME;
	}

	_external_gimbal_manager_set_attitude_pub.publish(msg);
}

void OutputMavlinkToGimbalManager::_request_gimbal_manager_information()
{
	vehicle_command_s cmd{};
	cmd.timestamp	= hrt_absolute_time();
	cmd.command 	= vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE;

	cmd.target_system 	= 0;
	cmd.target_component	= 0;

	cmd.source_system	= (uint8_t)_parameters.mav_sysid;
	cmd.source_component	= (uint8_t)_parameters.mav_compid;

	cmd.confirmation	= 0;
	cmd.from_external	= false;
	cmd.param1 = static_cast<float>(vehicle_command_s::VEHICLE_CMD_GIMBAL_MANAGER_INFORMATION);

	_vehicle_command_pub.publish(cmd);
}

void OutputMavlinkToGimbalManager::_check_for_gimbal_manager_information()
{
	external_gimbal_manager_information_s info{};

	if (_external_gimbal_manager_information_sub.update(&info)) {
		_manager_sysid		= info.source_system;
		_manager_compid		= info.source_component;
		_gimbal_device_id	= info.gimbal_device_id;
	}
}

void OutputMavlinkToGimbalManager::_check_for_gimbal_manager_status()
{
	external_gimbal_manager_status_s gm_st{};

	if (_external_gimbal_manager_status_sub.update(&gm_st)) {
		if (gm_st.gimbal_device_id == 0) {
			return;
		}

		if (_gimbal_device_id != 0 && gm_st.gimbal_device_id != _gimbal_device_id) {
			return;
		}

		if (gm_st.source_system == _parameters.mav_sysid
		    && gm_st.source_component == _parameters.mav_compid) { // Reject GIMBAL_MANAGER_STATUS published by PX4
			return;
		}

		const bool have_primary_control =
			((gm_st.primary_control_sysid == _parameters.mav_sysid) &&
			 (gm_st.primary_control_compid == _parameters.mav_compid));

		const bool have_secondary_control =
			((gm_st.secondary_control_sysid == _parameters.mav_sysid) &&
			 (gm_st.secondary_control_compid == _parameters.mav_compid));

		if (have_primary_control) {
			_control_rights = ControlRights::PRIMARY;
		}

		else if (have_secondary_control) {
			_control_rights = ControlRights::SECONDARY;

		} else {
			_control_rights = ControlRights::NONE;
		}
	}
}

void OutputMavlinkToGimbalManager::_reset_take_control_state()
{
	_take_control_retry_count = 0;
	_wait_ack_start_time = 0;
	_take_control_ack_received = false;
	_take_control_ack_accepted = false;
}

bool OutputMavlinkToGimbalManager::_acquire_control_for_autopilot(const hrt_abstime &now)
{
	const bool gimbal_busy_active = (_take_control_backoff_start != 0) && ((now - _take_control_backoff_start) < GIMBAL_BUSY_TIMEOUT_US);
	const bool wait_ack_active = (_wait_ack_start_time != 0) && ((now - _wait_ack_start_time) < WAIT_ACK_TIMEOUT_US);

	if (_control_rights != ControlRights::NONE) {
		_reset_take_control_state();
		return true;
	}

	if (gimbal_busy_active) {
		return false;
	}

	if (!wait_ack_active) {
		if (_take_control_retry_count < 3) {
			_send_take_control_request();
			return true;
		}

		events::send(events::ID("gimbal_take_control_limit_reached"), events::Log::Warning,
			     "Gimbal control could not be acquired, backing off for 15s");
		PX4_WARN("Gimbal control could not be acquired after multiple attempts, backing off for 15s");

		_take_control_backoff_start = now;
		_reset_take_control_state();
		return false;
	}

	if (!_take_control_ack_received) {
		return true;
	}

	if (!_take_control_ack_accepted) {
		events::send(events::ID("gimbal_take_control_rejected"), events::Log::Error,
			     "Gimbal manager rejected PX4 control request, backing off for 15s");
		PX4_ERR("Gimbal manager rejected PX4 take control request, backing off for 15s");
		_take_control_backoff_start = now;
		_reset_take_control_state();
		return false;
	}

	if (_take_control_retry_count < 3) {
		_send_take_control_request();
		return true;
	}

	events::send(events::ID("gimbal_take_control_no_rights_limit_reached"), events::Log::Warning,
		     "Gimbal control could not be acquired, backing off for 15s");
	PX4_WARN("Gimbal control could not be acquired after multiple attempts, backing off for 15s");
	_take_control_backoff_start = now;
	_reset_take_control_state();
	return false;
}

bool OutputMavlinkToGimbalManager::_check_for_take_control_ack()
{
	vehicle_command_ack_s ack{};

	if (!_vehicle_command_ack_sub.update(&ack)) {
		return false;
	}

	if (ack.command != vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE) {
		return false;
	}

	_take_control_ack_received = true;
	_take_control_ack_accepted = _handle_take_control_command_ack(ack);
	return true;
}

bool OutputMavlinkToGimbalManager::_handle_take_control_command_ack(const vehicle_command_ack_s &ack)
{
	if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
	    || ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
		return true;
	}

	return false;
}

void OutputMavlinkToGimbalManager::_send_take_control_request()
{
	vehicle_command_s cmd{};
	cmd.timestamp 	= hrt_absolute_time();
	cmd.command 	= vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;

	cmd.target_system 	= _manager_sysid;
	cmd.target_component 	= _manager_compid;

	cmd.source_system	= static_cast<uint8_t>(_parameters.mav_sysid);
	cmd.source_component 	= static_cast<uint8_t>(_parameters.mav_compid);

	cmd.confirmation 	= 0;
	cmd.from_external	= false;

	cmd.param1 = -2.0f;
	cmd.param2 = -2.0f;

	cmd.param3 = -1.f;
	cmd.param4 = -1.f;
	cmd.param7 = static_cast<float>(_gimbal_device_id);

	_vehicle_command_pub.publish(cmd);

	PX4_INFO("SEND TAKE CTRL to %u/%u from %u/%u",
		 cmd.target_system, cmd.target_component,
		 cmd.source_system, cmd.source_component);

	_take_control_retry_count++;
	_wait_ack_start_time = cmd.timestamp;
	_take_control_ack_received = false;
	_take_control_ack_accepted = false;
}

void OutputMavlinkToGimbalManager::print_status() const
{
	PX4_INFO("Output: MAVLink -> GimbalManager");

	PX4_INFO_RAW("\n\n");

	PX4_INFO("	device id: %u", (unsigned)_gimbal_device_id);
	PX4_INFO("	our sys/comp: %u/%u", (unsigned)_parameters.mav_sysid, (unsigned)_parameters.mav_compid);
	PX4_INFO("	gimbal manager sys/comp: %u/%u", (unsigned)_manager_sysid, (unsigned)_manager_compid);
	PX4_INFO("	have valid manager: %s", _have_valid_manager() ? "true" : "false");
	PX4_INFO("	control_rights: %s", _control_right_str(_control_rights));
}
}
