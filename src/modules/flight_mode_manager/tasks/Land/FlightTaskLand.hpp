/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskLand.hpp
 *
 * Flight task for landing
 *
 * @author Claudio Chies <claudio@chies.com>
 */
#pragma once

#include "FlightTask.hpp"
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/geo/geo.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include "Sticks.hpp"
#include "StickAccelerationXY.hpp"
#include "StickYaw.hpp"

using matrix::Vector3f;
using matrix::Vector2f;

class FlightTaskLand : public FlightTask
{
public:
	FlightTaskLand() = default;
	virtual ~FlightTaskLand() = default;

	bool update() override;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;

protected:
	void updateParams() override;

	PositionSmoothing _position_smoothing;
	Sticks _sticks{this};
	StickAccelerationXY _stick_acceleration_xy{this};
	StickYaw _stick_yaw{this};

	uORB::SubscriptionData<home_position_s>		_sub_home_position{ORB_ID(home_position)};
	uORB::SubscriptionData<vehicle_status_s>	_sub_vehicle_status{ORB_ID(vehicle_status)};
	uORB::SubscriptionData<position_setpoint_triplet_s> _sub_triplet_setpoint{ORB_ID(position_setpoint_triplet)};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) 	_param_mpc_acc_down_max,
					(ParamFloat<px4::params::MPC_ACC_HOR>) 		_param_mpc_acc_hor,
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) 	_param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_JERK_AUTO>) 	_param_mpc_jerk_auto,
					(ParamFloat<px4::params::MPC_JERK_MAX>) 	_param_mpc_jerk_max,
					(ParamFloat<px4::params::MPC_LAND_ALT1>) 	_param_mpc_land_alt1,
					(ParamFloat<px4::params::MPC_LAND_ALT2>) 	_param_mpc_land_alt2,
					(ParamFloat<px4::params::MPC_LAND_ALT3>) 	_param_mpc_land_alt3,
					(ParamFloat<px4::params::MPC_LAND_CRWL>) 	_param_mpc_land_crawl_speed,
					(ParamFloat<px4::params::MPC_LAND_RADIUS>) 	_param_mpc_land_radius,
					(ParamInt<px4::params::MPC_LAND_RC_HELP>) 	_param_mpc_land_rc_help,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) 	_param_mpc_land_speed,
					(ParamFloat<px4::params::MPC_XY_ERR_MAX>) 	_param_mpc_xy_err_max,
					(ParamFloat<px4::params::MPC_XY_TRAJ_P>) 	_param_mpc_xy_traj_p,
					(ParamFloat<px4::params::MPC_XY_VEL_MAX>) 	_param_mpc_xy_vel_max,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) 	_param_mpc_z_v_auto_dn,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) 	_param_mpc_z_v_auto_up,
					(ParamFloat<px4::params::NAV_MC_ALT_RAD>) 	_param_nav_mc_alt_rad
				       );
private:
	void _CalculateBrakingLocation();
	void _HandleHighVelocities();
	void _PerformLanding();
	void _SmoothBrakingPath();
	void _UpdateTrajConstraints();

	bool _landing{false};
	bool _is_initialized{false};
	bool _exceeded_max_vel{false};

	Vector3f _initial_land_position;
	Vector3f _land_position;
	float _land_heading{0.0f};


};
