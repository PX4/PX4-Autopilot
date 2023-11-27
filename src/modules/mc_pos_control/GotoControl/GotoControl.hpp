/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file GotoControl.hpp
 *
 * A class which smooths position and heading references from "go-to" setpoints
 * for planar multicopters.
 *
 * Be sure to set constraints with setGotoConstraints() before calling the update() method for the first time
 */

#pragma once

#include <lib/motion_planning/HeadingSmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <mathlib/math/Limits.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>

class GotoControl : public ModuleParams
{
public:
	GotoControl(ModuleParams *parent) : ModuleParams(parent) {};
	~GotoControl() = default;

	/** @param error [m] position smoother's maximum allowed horizontal position error at which trajectory integration halts */
	void setMaxAllowedHorizontalPositionError(const float error) { _position_smoothing.setMaxAllowedHorizontalError(error); }

	bool checkForSetpoint(const hrt_abstime &now, const bool enabled);

	/**
	 * @brief resets the position smoother at the current position with zero velocity and acceleration.
	 *
	 * @param position [m] (NED) local vehicle position
	 */
	void resetPositionSmoother(const matrix::Vector3f &position);

	/**
	 * @brief resets the heading smoother at the current heading with zero heading rate and acceleration.
	 *
	 * @param heading [rad] (from North) vehicle heading
	 */
	void resetHeadingSmoother(const float heading);

	/**
	 * @brief updates the smoothers with the current setpoints and outputs the "trajectory setpoint" for lower level
	 * loops to track.
	 *
	 * @param[in] dt [s] time since last control update
	 * @param[in] position [m] (NED) local vehicle position
	 * @param[in] heading [rad] (from North) vehicle heading
	 * @param[in] goto_setpoint struct containing current go-to setpoints
	 * @param[out] trajectory_setpoint struct containing trajectory (tracking) setpoints
	 */
	void update(const float dt, const matrix::Vector3f &position, const float heading);

private:
	void updateParams() override;

	/**
	 * @brief optionally sets dynamic translational speed limits with corresponding scale on acceleration
	 *
	 * @param goto_setpoint struct containing current go-to setpoints
	 */
	void setPositionSmootherLimits(const goto_setpoint_s &goto_setpoint);

	/**
	 * @brief optionally sets a dynamic heading rate limit with corresponding scale on heading acceleration
	 *
	 * @param goto_setpoint struct containing current go-to setpoints
	 */
	void setHeadingSmootherLimits(const goto_setpoint_s &goto_setpoint);

	uORB::SubscriptionData<goto_setpoint_s> _goto_setpoint_sub{ORB_ID(goto_setpoint)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_constraints_s> _vehicle_constraints_pub{ORB_ID(vehicle_constraints)};

	PositionSmoothing _position_smoothing;
	HeadingSmoothing _heading_smoothing;

	bool _is_initialized{false}; ///< true if smoothers were reset to current state

	// flags that the next update() requires a valid current vehicle position to reset the smoothers
	bool _need_smoother_reset{true};

	// flags if the last update() was controlling heading
	bool _controlling_heading{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) _param_mpc_z_v_auto_dn,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up
	);
};
