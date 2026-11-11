/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskTakeoff.hpp
 *
 * Multicopter takeoff that also works from a moving platform.
 *
 * The vehicle leaves the ground level, without correcting horizontally, so it can't tip over or
 * scrape its landing gear over the platform. Above that, it eases into horizontal velocity control
 * with a growing acceleration limit and finally stops horizontally while it climbs to the takeoff
 * altitude.
 */

#pragma once

#include "FlightTask.hpp"

#include <lib/motion_planning/PositionSmoothing.hpp>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/takeoff_status.h>

class FlightTaskTakeoff : public FlightTask
{
public:
	FlightTaskTakeoff() = default;
	virtual ~FlightTaskTakeoff() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

protected:
	void _ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy) override;
	void _ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy) override;
	void _ekfResetHandlerPositionZ(float delta_z) override;
	void _ekfResetHandlerVelocityZ(float delta_vz) override;
	void _ekfResetHandlerHeading(float delta_psi) override;

private:
	enum class Phase {
		level, ///< leave the ground level without horizontal correction
		ease_in, ///< brake to zero horizontal velocity with a growing acceleration limit
		climb ///< stop horizontally and climb to the takeoff altitude
	};

	void startLevel();
	void startEaseIn();
	void startClimb(const matrix::Vector3f &acceleration, const matrix::Vector3f &velocity,
			const matrix::Vector3f &position);

	void updateLevel();
	void updateEaseIn(float height);
	void updateClimb();

	/** @return local z of the takeoff altitude, NAN if Navigator doesn't provide one */
	float takeoffZ() const;

	// TODO: make these configurable once the behavior is settled
	static constexpr float kLevelHeight = 0.5f; ///< [m] height above ground to climb level
	static constexpr float kEaseInHeight = 1.f; ///< [m] height over which to ease into velocity control

	uORB::SubscriptionData<position_setpoint_triplet_s> _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::SubscriptionData<takeoff_status_s> _takeoff_status_sub{ORB_ID(takeoff_status)};

	PositionSmoothing _position_smoothing;

	Phase _phase{Phase::level};
	float _ground_z{NAN}; ///< [m] local position of the ground the vehicle took off from
	float _yaw_hold{NAN}; ///< [rad] heading to keep during the takeoff
	matrix::Vector2f _ease_in_velocity_xy; ///< [m/s] horizontal velocity setpoint while easing in

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor,
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
					(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
					(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
					(ParamFloat<px4::params::MPC_Z_ERR_MAX>) _param_mpc_z_err_max,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) _param_mpc_z_v_auto_dn,
					(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed
				       )
};
