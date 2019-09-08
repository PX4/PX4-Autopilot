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
 * @file FlightTaskManualPositionSmoothVel.hpp
 *
 * Flight task for smooth manual controlled position.
 */

#pragma once

#include "FlightTaskManualPosition.hpp"
#include "VelocitySmoothing.hpp"

using matrix::Vector2f;

class FlightTaskManualPositionSmoothVel : public FlightTaskManualPosition
{
public:
	FlightTaskManualPositionSmoothVel() = default;

	virtual ~FlightTaskManualPositionSmoothVel() = default;

	bool activate(vehicle_local_position_setpoint_s last_setpoint) override;
	void reActivate() override;

protected:

	virtual void _updateSetpoints() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualPosition,
					(ParamFloat<px4::params::MPC_JERK_MAX>) _param_mpc_jerk_max,
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max
				       )

private:
	void checkSetpoints(vehicle_local_position_setpoint_s &setpoints);

	void _resetPositionLock();
	void _resetPositionLockXY();
	void _resetPositionLockZ();

	void _initEkfResetCounters();
	void _initEkfResetCountersXY();
	void _initEkfResetCountersZ();

	void _checkEkfResetCounters(); /**< Reset the trajectories when the ekf resets velocity or position */
	void _checkEkfResetCountersXY();
	void _checkEkfResetCountersZ();

	void _updateTrajectories();
	void _updateTrajectoriesXY();
	void _updateTrajectoriesZ();

	void _updateTrajConstraints();
	void _updateTrajConstraintsXY();
	void _updateTrajConstraintsZ();

	void _updateTrajDurations();
	void _updateTrajDurationsXY();
	void _updateTrajDurationsZ();

	void _checkPositionLock();
	void _checkPositionLockXY();
	void _checkPositionLockZ();

	void _setOutputState();
	void _setOutputStateXY();
	void _setOutputStateZ();

	VelocitySmoothing _smoothing_xy[2]; ///< Smoothing in x and y directions
	VelocitySmoothing _smoothing_z; ///< Smoothing in z direction

	Vector2f _velocity_target_xy;
	float _velocity_target_z{0.f};

	bool _position_lock_xy_active{false};
	bool _position_lock_z_active{false};

	Vector2f _position_setpoint_xy_locked;
	float _position_setpoint_z_locked{NAN};

	/* counters for estimator local position resets */
	struct {
		uint8_t xy;
		uint8_t vxy;
		uint8_t z;
		uint8_t vz;
	} _reset_counters{0, 0, 0, 0};

	struct {
		Vector2f j;
		Vector2f a;
		Vector2f v;
		Vector2f x;
	} _traj_xy;

	struct {
		float j;
		float a;
		float v;
		float x;
	} _traj_z{0.f, 0.f, 0.f, NAN};
};
