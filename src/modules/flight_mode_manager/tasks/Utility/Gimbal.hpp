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

/**
 * @file Gimbal.hpp
 * @brief Gimbal interface with flight tasks
 * @author Pernilla Wikström <pernilla@auterion.com>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/gimbal_device_attitude_status.h>
#include <uORB/topics/vehicle_command.h>

#include "Sticks.hpp"


class Gimbal : public ModuleParams
{
public:
	Gimbal(ModuleParams *parent);
	~Gimbal();

	bool checkForTelemetry(const hrt_abstime now);
	void publishGimbalManagerSetAttitude(const uint16_t gimbal_flags,
					     const matrix::Quatf &q_gimbal_setpoint,
					     const matrix::Vector3f &gimbal_rates);
	void acquireGimbalControlIfNeeded();
	void releaseGimbalControlIfNeeded();
	float getTelemetryYaw() { return _telemetry_yaw; }
	bool allAxesLockedConfirmed() { return _telemetry_flags == FLAGS_ALL_AXES_LOCKED; }

	static constexpr uint16_t FLAGS_ALL_AXES_LOCKED = gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_ROLL_LOCK
			| gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_PITCH_LOCK
			| gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_YAW_LOCK;
	static constexpr uint16_t FLAGS_ROLL_PITCH_LOCKED =
		gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_ROLL_LOCK
		| gimbal_manager_set_attitude_s::GIMBAL_MANAGER_FLAGS_PITCH_LOCK;

private:
	bool _have_gimbal_control{false};

	uORB::Subscription _gimbal_device_attitude_status_sub{ORB_ID(gimbal_device_attitude_status)};
	hrt_abstime _telemtry_timestamp{};
	uint16_t _telemetry_flags{};
	float _telemetry_yaw{};

	uORB::Publication<gimbal_manager_set_attitude_s> _gimbal_manager_set_attitude_pub{ORB_ID(gimbal_manager_set_attitude)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>) _param_mav_comp_id
	)
};
