/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "Gimbal.hpp"
#include <px4_platform_common/events.h>

using namespace time_literals;
using namespace matrix;

Gimbal::Gimbal(ModuleParams *parent) :
	ModuleParams(parent)
{}

Gimbal::~Gimbal()
{
	releaseGimbalControlIfNeeded();
}

bool Gimbal::checkForTelemetry(const hrt_abstime now)
{
	if (_gimbal_device_attitude_status_sub.updated()) {
		gimbal_device_attitude_status_s gimbal_device_attitude_status{};

		if (_gimbal_device_attitude_status_sub.copy(&gimbal_device_attitude_status)) {
			_telemtry_timestamp = gimbal_device_attitude_status.timestamp;
			_telemetry_flags = gimbal_device_attitude_status.device_flags;
			_telemetry_yaw = Eulerf(Quatf(gimbal_device_attitude_status.q)).psi();
		}
	}

	return now < _telemtry_timestamp + 2_s;
}

void Gimbal::acquireGimbalControlIfNeeded()
{
	if (!_have_gimbal_control) {
		_have_gimbal_control = true;

		vehicle_command_s vehicle_command{};
		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
		vehicle_command.param1 = _param_mav_sys_id.get();
		vehicle_command.param2 = _param_mav_comp_id.get();
		vehicle_command.param3 = -1.0f; // Leave unchanged.
		vehicle_command.param4 = -1.0f; // Leave unchanged.
		vehicle_command.timestamp = hrt_absolute_time();
		vehicle_command.source_system = _param_mav_sys_id.get();
		vehicle_command.source_component = _param_mav_comp_id.get();
		vehicle_command.target_system = _param_mav_sys_id.get();
		vehicle_command.target_component = _param_mav_sys_id.get();
		vehicle_command.from_external = false;
		_vehicle_command_pub.publish(vehicle_command);
	}
}

void Gimbal::releaseGimbalControlIfNeeded()
{
	if (_have_gimbal_control) {
		_have_gimbal_control = false;

		// Restore default flags, setting rate setpoints to NAN lead to unexpected behavior
		publishGimbalManagerSetAttitude(FLAGS_ROLL_PITCH_LOCKED,
						Quatf(NAN, NAN, NAN, NAN),
						Vector3f(NAN, 0.f, 0.f));

		// Release gimbal
		vehicle_command_s vehicle_command{};
		vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
		vehicle_command.param1 = -3.0f; // Remove control if it had it.
		vehicle_command.param2 = -3.0f; // Remove control if it had it.
		vehicle_command.param3 = -1.0f; // Leave unchanged.
		vehicle_command.param4 = -1.0f; // Leave unchanged.
		vehicle_command.timestamp = hrt_absolute_time();
		vehicle_command.from_external = false;
		vehicle_command.source_system = _param_mav_sys_id.get();
		vehicle_command.source_component = _param_mav_comp_id.get();
		vehicle_command.target_system = _param_mav_sys_id.get();
		vehicle_command.target_component = _param_mav_comp_id.get();
		_vehicle_command_pub.publish(vehicle_command);
	}
}

void Gimbal::publishGimbalManagerSetAttitude(const uint16_t gimbal_flags,
		const matrix::Quatf &q_gimbal_setpoint,
		const matrix::Vector3f &gimbal_rates)
{
	gimbal_manager_set_attitude_s gimbal_setpoint{};
	gimbal_setpoint.origin_sysid = _param_mav_sys_id.get();
	gimbal_setpoint.origin_compid =  _param_mav_comp_id.get();
	gimbal_setpoint.flags = gimbal_flags;
	q_gimbal_setpoint.copyTo(gimbal_setpoint.q);
	gimbal_setpoint.angular_velocity_x =  gimbal_rates(0);
	gimbal_setpoint.angular_velocity_y = gimbal_rates(1);
	gimbal_setpoint.angular_velocity_z = gimbal_rates(2);
	gimbal_setpoint.timestamp = hrt_absolute_time();
	_gimbal_manager_set_attitude_pub.publish(gimbal_setpoint);
}
