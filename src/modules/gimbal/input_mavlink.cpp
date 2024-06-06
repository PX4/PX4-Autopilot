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


#include "input_mavlink.h"
#include <uORB/Publication.hpp>
#include <uORB/topics/gimbal_manager_information.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <errno.h>
#include <math.h>
#include <matrix/matrix/math.hpp>

namespace gimbal
{

InputMavlinkROI::InputMavlinkROI(Parameters &parameters) :
	InputBase(parameters) {}

InputMavlinkROI::~InputMavlinkROI()
{
	if (_vehicle_roi_sub >= 0) {
		orb_unsubscribe(_vehicle_roi_sub);
	}

	if (_position_setpoint_triplet_sub >= 0) {
		orb_unsubscribe(_position_setpoint_triplet_sub);
	}
}

int InputMavlinkROI::initialize()
{
	_vehicle_roi_sub = orb_subscribe(ORB_ID(vehicle_roi));

	if (_vehicle_roi_sub < 0) {
		return -errno;
	}

	_position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

	if (_position_setpoint_triplet_sub < 0) {
		return -errno;
	}

	return 0;
}

InputMavlinkROI::UpdateResult
InputMavlinkROI::update(unsigned int timeout_ms, ControlData &control_data, bool already_active)
{
	const int num_poll = 2;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = _vehicle_roi_sub;
	polls[0].events = POLLIN;
	polls[1].fd = _position_setpoint_triplet_sub;
	polls[1].events = POLLIN;

	int ret = px4_poll(polls, num_poll, timeout_ms);

	if (ret <= 0) {
		return UpdateResult::NoUpdate;
	}

	if (polls[0].revents & POLLIN) {
		vehicle_roi_s vehicle_roi;
		orb_copy(ORB_ID(vehicle_roi), _vehicle_roi_sub, &vehicle_roi);

		if (vehicle_roi.mode == vehicle_roi_s::ROI_NONE) {

			control_data.type = ControlData::Type::Neutral;
			_cur_roi_mode = vehicle_roi.mode;
			control_data.sysid_primary_control = _parameters.mav_sysid;
			control_data.compid_primary_control = _parameters.mav_compid;

			return UpdateResult::UpdatedActiveOnce;

		} else if (vehicle_roi.mode == vehicle_roi_s::ROI_WPNEXT) {
			control_data.type = ControlData::Type::LonLat;
			_read_control_data_from_position_setpoint_sub(control_data);
			control_data.type_data.lonlat.pitch_fixed_angle = -10.f;

			control_data.type_data.lonlat.roll_offset = vehicle_roi.roll_offset;
			control_data.type_data.lonlat.pitch_offset = vehicle_roi.pitch_offset;
			control_data.type_data.lonlat.yaw_offset = vehicle_roi.yaw_offset;

			_cur_roi_mode = vehicle_roi.mode;

			return UpdateResult::UpdatedActive;

		} else if (vehicle_roi.mode == vehicle_roi_s::ROI_LOCATION) {
			control_data_set_lon_lat(control_data, vehicle_roi.lon, vehicle_roi.lat, vehicle_roi.alt);

			_cur_roi_mode = vehicle_roi.mode;

			return UpdateResult::UpdatedActive;

		} else if (vehicle_roi.mode == vehicle_roi_s::ROI_TARGET) {
			PX4_WARN("ROI_TARGET not supported yet");
			return UpdateResult::NoUpdate;
		}

		return UpdateResult::NoUpdate;
	}

	// check whether the position setpoint has been updated
	if (polls[1].revents & POLLIN) {
		if (_cur_roi_mode == vehicle_roi_s::ROI_WPNEXT) {
			_read_control_data_from_position_setpoint_sub(control_data);

			return UpdateResult::UpdatedActive;

		} else { // must do an orb_copy() in *every* case
			position_setpoint_triplet_s position_setpoint_triplet;
			orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub,
				 &position_setpoint_triplet);
		}
	}

	return UpdateResult::NoUpdate;
}

void InputMavlinkROI::_read_control_data_from_position_setpoint_sub(ControlData &control_data)
{
	position_setpoint_triplet_s position_setpoint_triplet;
	orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet);
	control_data.type_data.lonlat.lon = position_setpoint_triplet.current.lon;
	control_data.type_data.lonlat.lat = position_setpoint_triplet.current.lat;
	control_data.type_data.lonlat.altitude = position_setpoint_triplet.current.alt;
}

void InputMavlinkROI::print_status() const
{
	PX4_INFO("Input: Mavlink (ROI)");
}

InputMavlinkCmdMount::InputMavlinkCmdMount(Parameters &parameters) :
	InputBase(parameters)
{}

InputMavlinkCmdMount::~InputMavlinkCmdMount()
{
	if (_vehicle_command_sub >= 0) {
		orb_unsubscribe(_vehicle_command_sub);
	}
}

int InputMavlinkCmdMount::initialize()
{
	if ((_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command))) < 0) {
		return -errno;
	}

	// rate-limit inputs to 100Hz. If we don't do this and the output is configured to mavlink mode,
	// it will publish vehicle_command's as well, causing the input poll() in here to return
	// immediately, which in turn will cause an output update and thus a busy loop.
	orb_set_interval(_vehicle_command_sub, 10);

	return 0;
}


InputMavlinkCmdMount::UpdateResult
InputMavlinkCmdMount::update(unsigned int timeout_ms, ControlData &control_data, bool already_active)
{
	const int num_poll = 1;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = _vehicle_command_sub;
	polls[0].events = POLLIN;

	int poll_timeout = (int) timeout_ms;

	const hrt_abstime poll_start = hrt_absolute_time();

	// If we get a command that we need to handle we exit the loop, otherwise we poll until we
	// reach the timeout.
	// We can't return early instead because we need to copy all topics that triggered poll.

	bool exit_loop = false;
	UpdateResult update_result = UpdateResult::NoUpdate;

	while (!exit_loop && poll_timeout >= 0) {

		const int ret = px4_poll(polls, num_poll, poll_timeout);

		if (ret <= 0) {
			// Error, or timeout, give up.
			exit_loop = true;
		}

		if (polls[0].revents & POLLIN) {

			vehicle_command_s vehicle_command;
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &vehicle_command);

			update_result = _process_command(control_data, vehicle_command);

		}

		if (update_result != UpdateResult::NoUpdate) {
			exit_loop = true;
		}

		// Keep going reading commands until timeout is up.
		poll_timeout = timeout_ms - (hrt_absolute_time() - poll_start) / 1000;
	}

	return update_result;
}

InputMavlinkCmdMount::UpdateResult
InputMavlinkCmdMount::_process_command(ControlData &control_data, const vehicle_command_s &vehicle_command)
{
	// Process only if the command is for us or for anyone (component id 0).
	const bool sysid_correct = (vehicle_command.target_system == _parameters.mav_sysid);
	const bool compid_correct = ((vehicle_command.target_component == _parameters.mav_compid) ||
				     (vehicle_command.target_component == 0));

	if (!sysid_correct || !compid_correct) {
		return UpdateResult::NoUpdate;
	}

	if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL) {

		UpdateResult update_result = UpdateResult::NoUpdate;

		switch ((int) vehicle_command.param7) {
		case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:

		// fallthrough
		case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
			control_data.type = ControlData::Type::Neutral;
			control_data.sysid_primary_control = vehicle_command.source_system;
			control_data.compid_primary_control = vehicle_command.source_component;
			update_result = UpdateResult::UpdatedActiveOnce;
			break;

		case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING: {
				control_data.type = ControlData::Type::Angle;
				control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
				control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
				control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

				// gimbal spec has roll on channel 0, MAVLink spec has pitch on channel 0
				const float roll = math::radians(vehicle_command.param2);
				// gimbal spec has pitch on channel 1, MAVLink spec has roll on channel 1
				const float pitch = math::radians(vehicle_command.param1);
				// both specs have yaw on channel 2
				float yaw = math::radians(vehicle_command.param3);

				matrix::Eulerf euler(roll, pitch, yaw);

				matrix::Quatf q(euler);
				q.copyTo(control_data.type_data.angle.q);

				control_data.type_data.angle.angular_velocity[0] = NAN;
				control_data.type_data.angle.angular_velocity[1] = NAN;
				control_data.type_data.angle.angular_velocity[2] = NAN;
				control_data.sysid_primary_control = vehicle_command.source_system;
				control_data.compid_primary_control = vehicle_command.source_component;
				update_result = UpdateResult::UpdatedActive;
			}
			break;

		case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
			// Take over control ourselves, that's supposedly what RC means.
			control_data.sysid_primary_control = _parameters.mav_sysid;
			control_data.compid_primary_control = _parameters.mav_compid;
			update_result = UpdateResult::NoUpdate;
			break;

		case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
			control_data_set_lon_lat(control_data, (double)vehicle_command.param6, (double)vehicle_command.param5,
						 vehicle_command.param4);
			control_data.sysid_primary_control = vehicle_command.source_system;
			control_data.compid_primary_control = vehicle_command.source_component;
			update_result = UpdateResult::UpdatedActive;
			break;
		}

		_ack_vehicle_command(vehicle_command);
		return update_result;

	} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE) {

		// Stabilization params are ignored. Use MNT_DO_STAB param instead.

		const int params[] = {
			(int)((float) vehicle_command.param5 + 0.5f),
			(int)((float) vehicle_command.param6 + 0.5f),
			(int)(vehicle_command.param7 + 0.5f)
		};

		for (int i = 0; i < 3; ++i) {
			switch (params[i]) {

			case 0:
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
				break;

			case 1:
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngularRate;
				break;

			case 2:
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
				break;

			default:
				// Not supported, fallback to body angle.
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
				break;
			}
		}

		control_data.sysid_primary_control = vehicle_command.source_system;
		control_data.compid_primary_control = vehicle_command.source_component;

		_ack_vehicle_command(vehicle_command);
		return UpdateResult::UpdatedActive;
	}

	return UpdateResult::NoUpdate;
}

void InputMavlinkCmdMount::_ack_vehicle_command(const vehicle_command_s &cmd)
{
	vehicle_command_ack_s vehicle_command_ack{};

	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = cmd.command;
	vehicle_command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	vehicle_command_ack.target_system = cmd.source_system;
	vehicle_command_ack.target_component = cmd.source_component;

	uORB::Publication<vehicle_command_ack_s> cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	cmd_ack_pub.publish(vehicle_command_ack);
}

void InputMavlinkCmdMount::print_status() const
{
	PX4_INFO("Input: Mavlink (CMD_MOUNT)");
}

InputMavlinkGimbalV2::InputMavlinkGimbalV2(Parameters &parameters) :
	InputBase(parameters)
{
}

InputMavlinkGimbalV2::~InputMavlinkGimbalV2()
{
	if (_vehicle_roi_sub >= 0) {
		orb_unsubscribe(_vehicle_roi_sub);
	}

	if (_position_setpoint_triplet_sub >= 0) {
		orb_unsubscribe(_position_setpoint_triplet_sub);
	}

	if (_gimbal_manager_set_attitude_sub >= 0) {
		orb_unsubscribe(_gimbal_manager_set_attitude_sub);
	}

	if (_vehicle_command_sub >= 0) {
		orb_unsubscribe(_vehicle_command_sub);
	}

	if (_gimbal_manager_set_manual_control_sub >= 0) {
		orb_unsubscribe(_gimbal_manager_set_manual_control_sub);
	}
}


void InputMavlinkGimbalV2::print_status() const
{
	PX4_INFO("Input: Mavlink (Gimbal V2)");
}

int InputMavlinkGimbalV2::initialize()
{
	_vehicle_roi_sub = orb_subscribe(ORB_ID(vehicle_roi));

	if (_vehicle_roi_sub < 0) {
		return -errno;
	}

	_position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

	if (_position_setpoint_triplet_sub < 0) {
		return -errno;
	}

	_gimbal_manager_set_attitude_sub = orb_subscribe(ORB_ID(gimbal_manager_set_attitude));

	if (_gimbal_manager_set_attitude_sub < 0) {
		return -errno;
	}

	if ((_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command))) < 0) {
		return -errno;
	}

	if ((_gimbal_manager_set_manual_control_sub = orb_subscribe(ORB_ID(gimbal_manager_set_manual_control))) < 0) {
		return -errno;
	}

	// rate-limit inputs to 100Hz. If we don't do this and the output is configured to mavlink mode,
	// it will publish vehicle_command's as well, causing the input poll() in here to return
	// immediately, which in turn will cause an output update and thus a busy loop.
	orb_set_interval(_vehicle_command_sub, 10);

	return 0;
}

void InputMavlinkGimbalV2::_stream_gimbal_manager_status(const ControlData &control_data)
{
	gimbal_device_attitude_status_s gimbal_device_attitude_status{};

	if (_gimbal_device_attitude_status_sub.updated()) {
		_gimbal_device_attitude_status_sub.copy(&gimbal_device_attitude_status);

		gimbal_manager_status_s gimbal_manager_status{};
		gimbal_manager_status.timestamp = hrt_absolute_time();
		gimbal_manager_status.flags = gimbal_device_attitude_status.device_flags;
		gimbal_manager_status.gimbal_device_id = control_data.device_compid;
		gimbal_manager_status.primary_control_sysid = control_data.sysid_primary_control;
		gimbal_manager_status.primary_control_compid = control_data.compid_primary_control;
		gimbal_manager_status.secondary_control_sysid = 0; // TODO: support secondary control
		gimbal_manager_status.secondary_control_compid = 0; // TODO: support secondary control
		_gimbal_manager_status_pub.publish(gimbal_manager_status);
	}
}

void InputMavlinkGimbalV2::_stream_gimbal_manager_information(const ControlData &control_data)
{
	// TODO: Take gimbal_device_information into account.

	gimbal_manager_information_s gimbal_manager_info;
	gimbal_manager_info.timestamp = hrt_absolute_time();

	gimbal_manager_info.cap_flags =
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK |
		gimbal_manager_information_s::GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL;

	gimbal_manager_info.pitch_max = M_PI_F / 2;
	gimbal_manager_info.pitch_min = -M_PI_F / 2;
	gimbal_manager_info.yaw_max = M_PI_F;
	gimbal_manager_info.yaw_min = -M_PI_F;

	gimbal_manager_info.gimbal_device_id = control_data.device_compid;

	_gimbal_manager_info_pub.publish(gimbal_manager_info);
}

InputMavlinkGimbalV2::UpdateResult
InputMavlinkGimbalV2::update(unsigned int timeout_ms, ControlData &control_data, bool already_active)
{

	const int num_poll = 5;
	px4_pollfd_struct_t polls[num_poll];
	polls[0].fd = _gimbal_manager_set_attitude_sub;
	polls[0].events = POLLIN;
	polls[1].fd = _vehicle_roi_sub;
	polls[1].events = POLLIN;
	polls[2].fd = _position_setpoint_triplet_sub;
	polls[2].events = POLLIN;
	polls[3].fd = _vehicle_command_sub;
	polls[3].events = POLLIN;
	polls[4].fd = _gimbal_manager_set_manual_control_sub;
	polls[4].events = POLLIN;

	int poll_timeout = (int) timeout_ms;

	hrt_abstime poll_start = hrt_absolute_time();

	// If we get a command that we need to handle we exit the loop, otherwise we poll until we
	// reach the timeout.
	// We can't return early instead because we need to copy all topics that triggered poll.

	bool exit_loop = false;
	UpdateResult update_result = already_active ? UpdateResult::UpdatedActive : UpdateResult::NoUpdate;

	while (!exit_loop && poll_timeout >= 0) {

		const int ret = px4_poll(polls, num_poll, poll_timeout);

		if (ret <= 0) {
			// Error, or timeout, give up.
			exit_loop = true;
		}

		if (polls[0].revents & POLLIN) {
			gimbal_manager_set_attitude_s set_attitude;
			orb_copy(ORB_ID(gimbal_manager_set_attitude), _gimbal_manager_set_attitude_sub, &set_attitude);

			update_result = _process_set_attitude(control_data, set_attitude);
		}

		if (polls[1].revents & POLLIN) {
			vehicle_roi_s vehicle_roi;
			orb_copy(ORB_ID(vehicle_roi), _vehicle_roi_sub, &vehicle_roi);

			UpdateResult new_result = _process_vehicle_roi(control_data, vehicle_roi);

			if (new_result != UpdateResult::NoUpdate) {
				update_result = new_result;
			}
		}

		// check whether the position setpoint got updated
		if (polls[2].revents & POLLIN) {
			position_setpoint_triplet_s position_setpoint_triplet;
			orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub,
				 &position_setpoint_triplet);

			UpdateResult new_result = _process_position_setpoint_triplet(control_data, position_setpoint_triplet);

			if (new_result != UpdateResult::NoUpdate) {
				update_result = new_result;
			}
		}

		if (polls[3].revents & POLLIN) {
			vehicle_command_s vehicle_command;
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &vehicle_command);

			UpdateResult new_result = _process_command(control_data, vehicle_command);

			if (new_result != UpdateResult::NoUpdate) {
				update_result = new_result;
			}
		}

		if (polls[4].revents & POLLIN) {
			gimbal_manager_set_manual_control_s set_manual_control;
			orb_copy(ORB_ID(gimbal_manager_set_manual_control), _gimbal_manager_set_manual_control_sub,
				 &set_manual_control);

			update_result = _process_set_manual_control(control_data, set_manual_control);
		}

		poll_timeout = timeout_ms - (hrt_absolute_time() - poll_start) / 1000;
	}

	_stream_gimbal_manager_status(control_data);

	if (_last_device_compid != control_data.device_compid) {
		_last_device_compid = control_data.device_compid;
		_stream_gimbal_manager_information(control_data);
	}

	return update_result;
}

InputMavlinkGimbalV2::UpdateResult InputMavlinkGimbalV2::_process_set_attitude(ControlData &control_data,
		const gimbal_manager_set_attitude_s &set_attitude)
{

	if (set_attitude.origin_sysid == control_data.sysid_primary_control &&
	    set_attitude.origin_compid == control_data.compid_primary_control) {
		const matrix::Quatf q(set_attitude.q);
		const matrix::Vector3f angular_velocity(set_attitude.angular_velocity_x,
							set_attitude.angular_velocity_y,
							set_attitude.angular_velocity_z);

		_set_control_data_from_set_attitude(control_data, set_attitude.flags, q, angular_velocity);
		return UpdateResult::UpdatedActive;

	} else {
		PX4_DEBUG("Ignoring gimbal_manager_set_attitude from %d/%d",
			  set_attitude.origin_sysid, set_attitude.origin_compid);
		return UpdateResult::UpdatedNotActive;
	}
}

InputMavlinkGimbalV2::UpdateResult InputMavlinkGimbalV2::_process_vehicle_roi(ControlData &control_data,
		const vehicle_roi_s &vehicle_roi)
{
	if (vehicle_roi.mode == vehicle_roi_s::ROI_NONE) {

		control_data.type = ControlData::Type::Neutral;
		_cur_roi_mode = vehicle_roi.mode;
		return UpdateResult::UpdatedActiveOnce;

	} else if (vehicle_roi.mode == vehicle_roi_s::ROI_WPNEXT) {
		control_data.type = ControlData::Type::LonLat;
		_read_control_data_from_position_setpoint_sub(control_data);
		control_data.type_data.lonlat.pitch_fixed_angle = -10.f;

		control_data.type_data.lonlat.roll_offset = vehicle_roi.roll_offset;
		control_data.type_data.lonlat.pitch_offset = vehicle_roi.pitch_offset;
		control_data.type_data.lonlat.yaw_offset = vehicle_roi.yaw_offset;

		_cur_roi_mode = vehicle_roi.mode;

		return UpdateResult::UpdatedActive;

	} else if (vehicle_roi.mode == vehicle_roi_s::ROI_LOCATION) {
		control_data_set_lon_lat(control_data, vehicle_roi.lon, vehicle_roi.lat, vehicle_roi.alt);

		_cur_roi_mode = vehicle_roi.mode;

		return UpdateResult::UpdatedActive;

	} else if (vehicle_roi.mode == vehicle_roi_s::ROI_TARGET) {
		//TODO is this even supported?

		return UpdateResult::NoUpdate;

	}

	return UpdateResult::NoUpdate;
}

InputMavlinkGimbalV2::UpdateResult InputMavlinkGimbalV2::_process_position_setpoint_triplet(ControlData &control_data,
		const position_setpoint_triplet_s &position_setpoint_triplet)
{
	if (_cur_roi_mode == vehicle_roi_s::ROI_WPNEXT) {
		control_data.type_data.lonlat.lon = position_setpoint_triplet.current.lon;
		control_data.type_data.lonlat.lat = position_setpoint_triplet.current.lat;
		control_data.type_data.lonlat.altitude = position_setpoint_triplet.current.alt;
		return UpdateResult::UpdatedActive;

	} else {
		return UpdateResult::NoUpdate;
	}
}

InputMavlinkGimbalV2::UpdateResult
InputMavlinkGimbalV2::_process_command(ControlData &control_data, const vehicle_command_s &vehicle_command)
{
	// Process only if the command is for us or for anyone (component id 0).
	const bool sysid_correct =
		(vehicle_command.target_system == _parameters.mav_sysid) || (vehicle_command.target_system == 0);
	const bool compid_correct = ((vehicle_command.target_component == _parameters.mav_compid) ||
				     (vehicle_command.target_component == 0));

	if (!sysid_correct || !compid_correct) {
		return UpdateResult::NoUpdate;
	}

	if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL) {
		// FIXME: Remove me later. For now, we support this for legacy missions
		//        using gimbal v1 protocol.

		UpdateResult update_result = UpdateResult::NoUpdate;

		switch ((int) vehicle_command.param7) {
		case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:

		// fallthrough

		case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
			control_data.type = ControlData::Type::Neutral;
			update_result = InputBase::UpdateResult::UpdatedActiveOnce;
			break;

		case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING: {
				control_data.type = ControlData::Type::Angle;
				control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
				control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
				control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

				// gimbal spec has roll on channel 0, MAVLink spec has pitch on channel 0
				const float roll = math::radians(vehicle_command.param2);
				// gimbal spec has pitch on channel 1, MAVLink spec has roll on channel 1
				const float pitch = math::radians(vehicle_command.param1);
				// both specs have yaw on channel 2
				float yaw = math::radians(vehicle_command.param3);

				// We expect angle of [-pi..+pi]. If the input range is [0..2pi] we can fix that.
				if (yaw > M_PI_F) {
					yaw -= 2 * M_PI_F;
				}

				matrix::Eulerf euler(roll, pitch, yaw);

				matrix::Quatf q(euler);
				q.copyTo(control_data.type_data.angle.q);

				control_data.type_data.angle.angular_velocity[0] = NAN;
				control_data.type_data.angle.angular_velocity[1] = NAN;
				control_data.type_data.angle.angular_velocity[2] = NAN;
				update_result = InputBase::UpdateResult::UpdatedActive;
			}
			break;

		case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
			update_result = UpdateResult::NoUpdate;
			break;

		case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
			control_data_set_lon_lat(control_data, (double)vehicle_command.param6, (double)vehicle_command.param5,
						 vehicle_command.param4);
			update_result = UpdateResult::UpdatedActive;
			break;
		}

		_ack_vehicle_command(vehicle_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		return update_result;

	} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE) {

		// Stabilization params are ignored. Use MNT_DO_STAB param instead.

		const int params[] = {
			(int)((float) vehicle_command.param5 + 0.5f),
			(int)((float) vehicle_command.param6 + 0.5f),
			(int)(vehicle_command.param7 + 0.5f)
		};

		for (int i = 0; i < 3; ++i) {
			switch (params[i]) {

			case 0:
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
				break;

			case 1:
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngularRate;
				break;

			case 2:
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
				break;

			default:
				// Not supported, fallback to body angle.
				control_data.type_data.angle.frames[i] =
					ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
				break;
			}
		}

		_ack_vehicle_command(vehicle_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		return UpdateResult::UpdatedActive;

	} else if (vehicle_command.command ==
		   vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE) {

		const int param_sysid = roundf(vehicle_command.param1);
		const int param_compid = roundf(vehicle_command.param2);

		uint8_t new_sysid_primary_control = [&]() {
			switch (param_sysid) {

			case 0 ... 255:
				// Valid new sysid.
				return (uint8_t) param_sysid;

			case -1:
				// leave unchanged
				return control_data.sysid_primary_control;

			case -2:
				// set itself
				return (uint8_t) _parameters.mav_sysid;

			case -3:

				// release control if in control
				if (control_data.sysid_primary_control == vehicle_command.source_system) {
					return (uint8_t) 0;

				} else {
					return control_data.sysid_primary_control;
				}

			default:
				PX4_WARN("Unknown param1 value for DO_GIMBAL_MANAGER_CONFIGURE");
				return control_data.sysid_primary_control;
			}
		}();

		uint8_t new_compid_primary_control = [&]() {
			switch (param_compid) {
			case 0 ... 255:
				// Valid new compid.
				return (uint8_t) param_compid;

			case -1:
				// leave unchanged
				return control_data.compid_primary_control;

			case -2:
				// set itself
				return (uint8_t) _parameters.mav_compid;

			case -3:

				// release control if in control
				if (control_data.compid_primary_control == vehicle_command.source_component) {
					return (uint8_t) 0;

				} else {
					return control_data.compid_primary_control;
				}

			default:
				PX4_WARN("Unknown param2 value for DO_GIMBAL_MANAGER_CONFIGURE");
				return control_data.compid_primary_control;
			}
		}();

		_ack_vehicle_command(vehicle_command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

		if (new_sysid_primary_control != control_data.sysid_primary_control ||
		    new_compid_primary_control != control_data.compid_primary_control) {
			PX4_INFO("Configured primary gimbal control sysid/compid from %d/%d to %d/%d",
				 control_data.sysid_primary_control, control_data.compid_primary_control,
				 new_sysid_primary_control, new_compid_primary_control);
			control_data.sysid_primary_control = new_sysid_primary_control;
			control_data.compid_primary_control = new_compid_primary_control;
		}

		// Just doing the configuration doesn't mean there is actually an update to use yet.
		// After that we still need to have an actual setpoint.
		return UpdateResult::NoUpdate;

		// TODO: support secondary control
		// TODO: support gimbal device id for multiple gimbals

	} else if (vehicle_command.command ==
		   vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW) {

		if (vehicle_command.source_system == control_data.sysid_primary_control &&
		    vehicle_command.source_component == control_data.compid_primary_control) {

			const matrix::Eulerf euler(0.0f, math::radians(vehicle_command.param1),
						   math::radians(vehicle_command.param2));
			const matrix::Quatf q(euler);
			const matrix::Vector3f angular_velocity(0.0f, vehicle_command.param3,
								vehicle_command.param4);
			const uint32_t flags = vehicle_command.param5;

			// TODO: support gimbal device id for multiple gimbals

			_set_control_data_from_set_attitude(control_data, flags, q, angular_velocity);
			_ack_vehicle_command(vehicle_command,
					     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);

			return UpdateResult::UpdatedActiveOnce;

		} else {
			PX4_INFO("GIMBAL_MANAGER_PITCHYAW from %d/%d denied, in control: %d/%d",
				 vehicle_command.source_system,
				 vehicle_command.source_component,
				 control_data.sysid_primary_control, control_data.compid_primary_control);
			_ack_vehicle_command(vehicle_command,
					     vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);

			return UpdateResult::UpdatedNotActive;
		}
	}

	return UpdateResult::NoUpdate;
}

InputMavlinkGimbalV2::UpdateResult InputMavlinkGimbalV2::_process_set_manual_control(ControlData &control_data,
		const gimbal_manager_set_manual_control_s &set_manual_control)
{
	if (set_manual_control.origin_sysid == control_data.sysid_primary_control &&
	    set_manual_control.origin_compid == control_data.compid_primary_control) {

		const matrix::Quatf q =
			(PX4_ISFINITE(set_manual_control.pitch) && PX4_ISFINITE(set_manual_control.yaw))
			?
			matrix::Quatf(
				matrix::Eulerf(0.0f, set_manual_control.pitch, set_manual_control.yaw))
			:
			matrix::Quatf(NAN, NAN, NAN, NAN);

		const matrix::Vector3f angular_velocity =
			(PX4_ISFINITE(set_manual_control.pitch_rate) &&
			 PX4_ISFINITE(set_manual_control.yaw_rate)) ?
			matrix::Vector3f(0.0f,
					 math::radians(_parameters.mnt_rate_pitch) * set_manual_control.pitch_rate,
					 math::radians(_parameters.mnt_rate_yaw) * set_manual_control.yaw_rate) :
			matrix::Vector3f(NAN, NAN, NAN);

		_set_control_data_from_set_attitude(control_data, set_manual_control.flags, q,
						    angular_velocity);

		return UpdateResult::UpdatedActive;

	} else {
		PX4_DEBUG("Ignoring gimbal_manager_set_manual_control from %d/%d",
			  set_manual_control.origin_sysid, set_manual_control.origin_compid);
		return UpdateResult::UpdatedNotActive;
	}
}

void InputMavlinkGimbalV2::_set_control_data_from_set_attitude(ControlData &control_data, const uint32_t flags,
		const matrix::Quatf &q,
		const matrix::Vector3f &angular_velocity)
{
	if ((flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_RETRACT) != 0) {
		// not implemented in ControlData
	} else if ((flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_NEUTRAL) != 0) {
		control_data.type = ControlData::Type::Neutral;

	} else {
		control_data.type = ControlData::Type::Angle;

		q.copyTo(control_data.type_data.angle.q);

		control_data.type_data.angle.angular_velocity[0] = angular_velocity(0);
		control_data.type_data.angle.angular_velocity[1] = angular_velocity(1);
		control_data.type_data.angle.angular_velocity[2] = angular_velocity(2);

		control_data.type_data.angle.frames[0] = (flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_ROLL_LOCK)
				? ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame
				: ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

		control_data.type_data.angle.frames[1] = (flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_PITCH_LOCK)
				? ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame
				: ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

		control_data.type_data.angle.frames[2] = (flags & gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_YAW_LOCK)
				? ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame
				: ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;
	}
}

//TODO move this one to input.cpp such that it can be shared across functions
void InputMavlinkGimbalV2::_ack_vehicle_command(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s vehicle_command_ack{};

	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = cmd.command;
	vehicle_command_ack.result = result;
	vehicle_command_ack.target_system = cmd.source_system;
	vehicle_command_ack.target_component = cmd.source_component;

	uORB::Publication<vehicle_command_ack_s> cmd_ack_pub{ORB_ID(vehicle_command_ack)};
	cmd_ack_pub.publish(vehicle_command_ack);
}

void InputMavlinkGimbalV2::_read_control_data_from_position_setpoint_sub(ControlData &control_data)
{
	position_setpoint_triplet_s position_setpoint_triplet;
	orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet);
	control_data.type_data.lonlat.lon = position_setpoint_triplet.current.lon;
	control_data.type_data.lonlat.lat = position_setpoint_triplet.current.lat;
	control_data.type_data.lonlat.altitude = position_setpoint_triplet.current.alt;
}

} /* namespace gimbal */
