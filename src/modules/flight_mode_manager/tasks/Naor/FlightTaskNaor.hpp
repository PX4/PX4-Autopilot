/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskNaor.hpp
 *
 * Flight task for NAOR manual control mode.
 */

#pragma once

#include <lib/stick_yaw/StickYaw.hpp>
#include <lib/sticks/Sticks.hpp>
#include "FlightTask.hpp"
#include "StickTiltXY.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/naor_debug.h>
#include <uORB/topics/super_hold_debug.h>
#include <uORB/topics/pid_consts.h>
#include <matrix/matrix/math.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <uORB/Publication.hpp>
#include <lib/pid/PID.hpp>



enum class SuperHoldMode_state {
	init = 0,
	operation =  1,
	Error_close =  2,

};

#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class FlightTaskNaor : public FlightTask
{
public:
	FlightTaskNaor() = default;
	virtual ~FlightTaskNaor() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;


protected:

	virtual void _updateSetpoints_acc(matrix::Vector2f acceleration_setpoint); /**< updates all setpoints */
	virtual void _throttleToVelocity(); /**< scales sticks to velocity in z */
	void first_pid_init(pid_consts_s pid_data);
	void Fast_error_close(float cartzian_error);
	void update_pid_constants(pid_consts_s pid_data);
	bool has_pid_constants_updated(pid_consts_s pid_data);
	matrix::Vector3f pid_operation(matrix::Vector3f target_position);
	void _execute_trajectory();
	void _publish_debug_topics();
	float total_Error_calc();
	bool _checkTakeoff() override; /**< override to remove altitude constraints */

	Sticks _sticks{this};
	StickTiltXY _stick_tilt_xy{this};
	StickYaw _stick_yaw{this};

	bool _sticks_data_required = true; ///< let inherited task-class define if it depends on stick data



private:

	void _updateTrajectoryBoundaries();
	bool _updateYawCorrection();
	SuperHoldMode_state _super_hold_mode_state = SuperHoldMode_state::init;
	vehicle_odometry_s _visual_odometry;
	uORB::Subscription _visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};
	bool _check_timer(uint64_t _timestamp);
	double _timer_threshold = 300000;  // 100ms = 100,000 microseconds


	naor_debug_s _naor_debug;
	uORB::Publication<naor_debug_s> _naor_debug_pub{ORB_ID(naor_debug)};
	super_hold_debug_s _super_hold_debug;
	uORB::Publication<super_hold_debug_s> _super_hold_debug_pub{ORB_ID(super_hold_debug)};
	uORB::Subscription _pid_consts_sub{ORB_ID(pid_consts)};
	PositionSmoothing _position_smoothing;

	PID _pid_x;
	PID _pid_y;
	PID _pid_altitude;
	pid_consts_s _pid_consts;
	uint64_t _last_pid_update_timestamp{0};

	matrix::Vector3f _position_to_hold;

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_XY_TRAJ_P>) _param_mpc_xy_traj_p,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max
	)
	bool _first_time = true;
	int init_counter = 200;


};

