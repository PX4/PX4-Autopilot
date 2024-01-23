/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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
 * @file DifferentialDriveControl.hpp
 *
 * Controller for heading rate and forward speed.
 */

#pragma once

#include <lib/pid/pid.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/differential_drive_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

class DifferentialDriveControl : public ModuleParams
{
public:
	DifferentialDriveControl(ModuleParams *parent);
	~DifferentialDriveControl() = default;

	void control(float dt);
	float getVehicleBodyYawRate() const { return _vehicle_body_yaw_rate; }
	float getVehicleYaw() const { return _vehicle_yaw; }

protected:
	void updateParams() override;

private:
	uORB::Subscription _differential_drive_setpoint_sub{ORB_ID(differential_drive_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	uORB::Publication<differential_drive_setpoint_s> _differential_drive_control_output_pub{ORB_ID(differential_drive_control_output)};

	differential_drive_setpoint_s _differential_drive_setpoint{};

	matrix::Quatf _vehicle_attitude_quaternion{};
	float _vehicle_yaw{0.f};

	// States
	float _vehicle_body_yaw_rate{0.f};
	float _vehicle_forward_speed{0.f};

	PID_t _pid_angular_velocity; ///< The PID controller for yaw rate.
	PID_t _pid_speed; ///< The PID controller for velocity.

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RDD_P_SPEED>) _param_rdd_p_gain_speed,
		(ParamFloat<px4::params::RDD_I_SPEED>) _param_rdd_i_gain_speed,
		(ParamFloat<px4::params::RDD_P_ANG_VEL>) _param_rdd_p_gain_angular_velocity,
		(ParamFloat<px4::params::RDD_I_ANG_VEL>) _param_rdd_i_gain_angular_velocity
	)
};
