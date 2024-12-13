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
 * @file StickYaw.hpp
 * @brief Generate setpoints to Stop the vehicle smoothly from any given initial velocity
 * @author Claudio Chies <claudio@chies.com>
 */
#include <px4_platform_common/module_params.h>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <uORB/topics/vehicle_constraints.h>
#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/log.h>

using matrix::Vector3f;
using matrix::Vector2f;
#pragma once
/**
 * This utility class generates setpoints which slow down the vehicle smoothly from any given initial velocity
 * to use it, first call initialize(), and then update() at every iteration.
 * it is important to use the getConstraints() to adjust the PositionController constraints if we exceed the maximum velocity in the
 * current flighttask.
 */
class Stop : public ModuleParams
{
public:
	Stop(ModuleParams *parent);
	~Stop() = default;

	void initialize(const Vector3f &acceleration, const Vector3f &velocity, const Vector3f &position,
			const float &deltatime);
	void update(const Vector3f &acceleration, const Vector3f &velocity, const Vector3f &position, const float &deltatime);

	/** @brief the the current constraints based on the current state of the vehicle, and update the constraints in the FlightTask by passing the reference
	 */
	void getConstraints(vehicle_constraints_s &constraints);

	bool checkMaxVelocityLimit(const Vector3f &velocity, const float &factor = 1.0f);

	// Getters
	Vector3f getPositionSetpoint() const 		{ return _position_setpoint; }
	Vector3f getVelocitySetpoint() const 		{ return _velocity_setpoint; }
	Vector3f getAccelerationSetpoint() const 	{ return _acceleration_setpoint; }
	Vector3f getJerkSetpoint() const 		{ return _jerk_setpoint; }
	Vector3f getUnsmoothedVelocity() const 		{ return _unsmoothed_velocity; }
	Vector3f getStopPosition() const 		{ return _stop_position; }

	void setYaw(float yaw) 				{ _yaw_setpoint = yaw; }
	float getYaw() const 				{ return _yaw_setpoint; }

	bool isActive() const {return _isActive;};
	bool wasActive()
	{
		bool curr_state = _wasActive;
		_wasActive = false;
		return curr_state;
	};
private:

	PositionSmoothing _position_smoothing;

	Vector3f _stop_position;
	bool _exceeded_max_vel{false}; // true if we exceed the maximum velcoity of the auto flight task.
	bool _isActive{false};		// true if the condition for braking is still valid
	bool _wasActive{false};		// true if the condition for braking was valid in the previous iteration

	Vector3f _jerk_setpoint;
	Vector3f _acceleration_setpoint;
	Vector3f _velocity_setpoint;
	Vector3f _position_setpoint;
	Vector3f _unsmoothed_velocity;
	float 	 _yaw_setpoint{NAN};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) 	_param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_ACC_HOR>) 		_param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) 	_param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_JERK_AUTO>) 	_param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_JERK_MAX>) 	_param_mpc_jerk_max,
		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) 	_param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_XY_TRAJ_P>) 	_param_mpc_xy_traj_p,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) 	_param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) 	_param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) 	_param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>) 	_param_nav_mc_alt_rad
	);
};
