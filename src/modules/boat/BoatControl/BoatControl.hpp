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
 * @file BoatControl.hpp
 *
 * Controller for heading rate and forward speed.
 */

#pragma once

#include <lib/pid/pid.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/boat_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

class BoatControl : public ModuleParams
{
public:
	BoatControl(ModuleParams *parent);
	~BoatControl() = default;

	void control(float dt);
	float getVehicleBodyYawRate() const { return _vehicle_body_yaw_rate; }
	float getVehicleYaw() const { return _vehicle_yaw; }
	vehicle_local_position_s getLocalPosition() const { return _vehicle_local_position; }

protected:
	void updateParams() override;

private:
	uORB::Subscription _boat_setpoint_sub{ORB_ID(boat_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	uORB::Publication<boat_setpoint_s> _boat_control_output_pub{ORB_ID(boat_control_output)};

	boat_setpoint_s _boat_setpoint{};

	matrix::Quatf _vehicle_attitude_quaternion{};
	float _vehicle_yaw{0.f};


	// States
	float _vehicle_body_yaw_rate{0.f};
	float _vehicle_forward_speed{0.f};
	Vector2f _vehicle_speed{0.f, 0.f};
	vehicle_local_position_s _vehicle_local_position{};

	PID_t _pid_angular_velocity;
	PID_t _pid_speed;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BT_SPD_P>) _param_bt_spd_p,
		(ParamFloat<px4::params::BT_SPD_I>) _param_bt_spd_i,
		(ParamFloat<px4::params::BT_SPD_IMAX>) _param_bt_spd_imax,
		(ParamFloat<px4::params::BT_SPD_OUTLIM>) _param_bt_spd_outlim,
		(ParamFloat<px4::params::BT_ANG_P>) _param_bt_ang_p,
		(ParamFloat<px4::params::BT_ANG_I>) _param_bt_ang_i,
		(ParamFloat<px4::params::BT_ANG_IMAX>) _param_bt_ang_imax,
		(ParamFloat<px4::params::BT_ANG_OUTLIM>) _param_bt_ang_outlim
	)
};
