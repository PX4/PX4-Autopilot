/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoLineSmoothVel.hpp
 *
 * Flight task for autonomous, gps driven mode. The vehicle flies
 * along a straight line in between waypoints.
 */

#pragma once

#include "FlightTaskAutoMapper2.hpp"
#include "VelocitySmoothing.hpp"

class FlightTaskAutoLineSmoothVel : public FlightTaskAutoMapper2
{
public:
	FlightTaskAutoLineSmoothVel() = default;
	virtual ~FlightTaskAutoLineSmoothVel() = default;

	bool activate() override;
	void reActivate() override;

protected:

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAutoMapper2,
					(ParamFloat<px4::params::MIS_YAW_ERR>) MIS_YAW_ERR, // yaw-error threshold
					(ParamFloat<px4::params::MPC_ACC_HOR>) MPC_ACC_HOR, // acceleration in flight
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) MPC_ACC_UP_MAX,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) MPC_ACC_DOWN_MAX,
					(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) MPC_ACC_HOR_MAX,
					(ParamFloat<px4::params::MPC_JERK_MIN>) MPC_JERK_MIN,
					(ParamFloat<px4::params::MPC_XY_TRAJ_P>) MPC_XY_TRAJ_P,
					(ParamFloat<px4::params::MPC_Z_TRAJ_P>) MPC_Z_TRAJ_P
				       );

	void _generateSetpoints() override; /**< Generate setpoints along line. */
	void _setDefaultConstraints() override;

	inline float constrain_one_side(float val, float constrain);
	void _generateHeading();
	bool _generateHeadingAlongTraj(); /**< Generates heading along trajectory. */
	void _updateTrajConstraints();
	void _prepareSetpoints(); /**< Generate velocity target points for the trajectory generator. */
	void _generateTrajectory();
	VelocitySmoothing _trajectory[3]; ///< Trajectories in x, y and z directions
	float _yaw_sp_prev;
};
