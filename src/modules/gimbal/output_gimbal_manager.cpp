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

#include "output_gimbal_manager.h"

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

namespace gimbal
{

OutputToGimbalManager::OutputToGimbalManager(const Parameters &parameters)
	: OutputBase(parameters)
{
}

void OutputToGimbalManager::update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id)
{
	// Note: we intentionally leave gimbal_device_id untouched. We don't own a
	// gimbal device, so we must not claim one via control_data.device_compid.
	(void)gimbal_device_id;

	const hrt_abstime now = hrt_absolute_time();

	_update_manager_info();
	_update_manager_status();

	if (!_manager_found) {
		// Nothing to talk to yet. The external manager streams its
		// information/status, so we just wait until we've seen it.
		return;
	}

	// We want control whenever there is an active setpoint to forward.
	const bool want_control = (control_data.type != ControlData::Type::Neutral);

	if (want_control) {
		switch (_control_state) {
		case ControlState::Released:
			_send_configure(true);
			_last_acquire_request = now;
			_control_state = ControlState::Acquiring;
			break;

		case ControlState::Acquiring:
			if (_have_primary_control()) {
				_control_state = ControlState::InControl;

			} else if (now - _last_acquire_request > kAcquireRetryInterval) {
				_send_configure(true);
				_last_acquire_request = now;
			}

			break;

		case ControlState::InControl:
			if (!_have_primary_control()) {
				// Someone took control from us. Try to reacquire.
				_send_configure(true);
				_last_acquire_request = now;
				_control_state = ControlState::Acquiring;
			}

			break;
		}

		if (_control_state == ControlState::InControl) {
			if (new_setpoints) {
				_set_angle_setpoints(control_data);
			}

			// Keep pointing updated as the vehicle moves relative to a location target.
			_handle_position_update(control_data);
			_publish_set_pitchyaw();
			_last_update = now;
		}

	} else {
		// No active setpoint: release control so other components (e.g. a
		// ground station) can command the manager.
		if (_control_state != ControlState::Released) {
			_send_configure(false);
			_control_state = ControlState::Released;
		}
	}
}

void OutputToGimbalManager::_update_manager_info()
{
	external_gimbal_manager_information_s information;

	if (_information_sub.update(&information)) {
		if (!_manager_found) {
			_manager_found = true;
			_manager_sysid = information.manager_sysid;
			_manager_compid = information.manager_compid;
			_gimbal_device_id = information.gimbal_device_id;
		}
	}
}

void OutputToGimbalManager::_update_manager_status()
{
	external_gimbal_manager_status_s status;

	if (_status_sub.update(&status)) {
		_status = status;
		_status_valid = true;

		// The status is streamed periodically, so it also works as a discovery
		// source if we haven't received the information message.
		if (!_manager_found) {
			_manager_found = true;
			_manager_sysid = status.manager_sysid;
			_manager_compid = status.manager_compid;
			_gimbal_device_id = status.gimbal_device_id;
		}
	}
}

bool OutputToGimbalManager::_have_primary_control() const
{
	return _status_valid
	       && _status.primary_control_sysid == (uint8_t)_parameters.mav_sysid
	       && _status.primary_control_compid == (uint8_t)_parameters.mav_compid;
}

void OutputToGimbalManager::_send_configure(bool acquire)
{
	// Special values per MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE: -2 releases
	// control, -1 leaves it unchanged. To acquire, we set ourselves as primary.
	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	cmd.param1 = acquire ? (float)_parameters.mav_sysid : -2.f;	// primary control sysid
	cmd.param2 = acquire ? (float)_parameters.mav_compid : -2.f;	// primary control compid
	cmd.param3 = -1.f;	// secondary control sysid: leave unchanged
	cmd.param4 = -1.f;	// secondary control compid: leave unchanged
	cmd.param7 = _gimbal_device_id;
	cmd.target_system = _manager_sysid;
	cmd.target_component = _manager_compid;
	cmd.source_system = _parameters.mav_sysid;
	cmd.source_component = _parameters.mav_compid;
	cmd.from_external = false;

	_vehicle_command_pub.publish(cmd);
}

void OutputToGimbalManager::_publish_set_pitchyaw()
{
	const matrix::Quatf q_setpoint(_q_setpoint);
	const matrix::Eulerf euler(q_setpoint);

	gimbal_manager_set_pitchyaw_s set_pitchyaw{};
	set_pitchyaw.timestamp = hrt_absolute_time();
	set_pitchyaw.target_system = _manager_sysid;
	set_pitchyaw.target_component = _manager_compid;
	set_pitchyaw.gimbal_device_id = _gimbal_device_id;

	set_pitchyaw.pitch = euler.theta();
	set_pitchyaw.yaw = euler.psi();
	set_pitchyaw.pitch_rate = _angle_velocity[1];
	set_pitchyaw.yaw_rate = _angle_velocity[2];

	uint32_t flags = 0;

	if (_absolute_angle[1]) {
		flags |= gimbal_manager_set_pitchyaw_s::GIMBAL_MANAGER_FLAGS_PITCH_LOCK;
	}

	if (_absolute_angle[2]) {
		flags |= gimbal_manager_set_pitchyaw_s::GIMBAL_MANAGER_FLAGS_YAW_LOCK;
		flags |= gimbal_manager_set_pitchyaw_s::GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME;
	}

	set_pitchyaw.flags = flags;

	_set_pitchyaw_pub.publish(set_pitchyaw);
}

void OutputToGimbalManager::print_status() const
{
	PX4_INFO("Output: external gimbal manager");

	if (_manager_found) {
		PX4_INFO_RAW("  manager: %d/%d, gimbal device id: %d\n",
			     _manager_sysid, _manager_compid, _gimbal_device_id);

	} else {
		PX4_INFO_RAW("  manager: not found yet\n");
	}

	const char *state_str = "released";

	switch (_control_state) {
	case ControlState::Acquiring: state_str = "acquiring"; break;

	case ControlState::InControl: state_str = "in control"; break;

	default: break;
	}

	PX4_INFO_RAW("  control: %s\n", state_str);
}

} /* namespace gimbal */
