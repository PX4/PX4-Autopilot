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
 #include "FlightTask.hpp"
 #include "Sticks.hpp"
 #include "StickTiltXY.hpp"
 #include <uORB/Subscription.hpp>
 #include <uORB/topics/vehicle_odometry.h>
 #include <matrix/matrix/math.hpp>
 #include <lib/motion_planning/PositionSmoothing.hpp>
 #include <uORB/Publication.hpp>
 #include <lib/pid/PID.hpp>


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
	 virtual void _updateSetpoints_velo(matrix::Vector2f velocity_setpoint); /**< updates all setpoints */
	 virtual void _throttleToVelocity(); /**< scales sticks to velocity in z */
	 virtual void _goToOrigin(); /**< go to origin position (0,0) */

	 bool _checkTakeoff() override; /**< override to remove altitude constraints */

	 Sticks _sticks{this};
	 StickTiltXY _stick_tilt_xy{this};
	 StickYaw _stick_yaw{this};

	 bool _sticks_data_required = true; ///< let inherited task-class define if it depends on stick data



 private:
	 void _updateTrajectoryBoundaries();
	 bool _updateYawCorrection();
	 vehicle_odometry_s _visual_odometry;
	 uORB::Subscription _visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};
	 bool _check_timer(uint64_t _timestamp);
	 double _timer_threshold = 300000;  // 100ms = 100,000 microseconds
	 // or more conservatively:


	 naor_debug_s _naor_debug;
	 uORB::Publication<naor_debug_s> _naor_debug_pub{ORB_ID(naor_debug)};
	 PositionSmoothing _position_smoothing;

	 DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
		 (ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		 (ParamFloat<px4::params::MPC_XY_TRAJ_P>) _param_mpc_xy_traj_p,
		 (ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max
	 )
	 float _origin_x = 0.0f;
	 float _origin_y = 0.0f;


 };
