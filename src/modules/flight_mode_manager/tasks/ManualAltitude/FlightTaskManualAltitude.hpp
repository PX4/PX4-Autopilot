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
 * @file FlightTaskManualAltitude.hpp
 *
 * Flight task for manual controlled altitude.
 */

#pragma once

#include "FlightTask.hpp"
#include "Sticks.hpp"
#include "StickTiltXY.hpp"
#include "StickYaw.hpp"
#include <uORB/Subscription.hpp>

class FlightTaskManualAltitude : public FlightTask
{
public:
	FlightTaskManualAltitude() = default;
	virtual ~FlightTaskManualAltitude() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;
	void setMaxDistanceToGround(float max_distance) { _max_distance_to_ground = max_distance; }

protected:
	void _ekfResetHandlerHeading(float delta_psi) override; /**< adjust heading setpoint in case of EKF reset event */
	void _ekfResetHandlerHagl(float delta_hagl) override;

	virtual void _updateSetpoints(); /**< updates all setpoints */
	virtual void _scaleSticks(); /**< scales sticks to velocity in z */
	bool _checkTakeoff() override;
	void _updateConstraintsFromEstimator();

	/**
	 *  Check and sets for position lock.
	 *  If sticks are at center position, the vehicle
	 *  will exit velocity control and enter position control.
	 */
	void _updateAltitudeLock();

	Sticks _sticks{this};
	StickTiltXY _stick_tilt_xy{this};
	StickYaw _stick_yaw{this};

	bool _sticks_data_required = true; ///< let inherited task-class define if it depends on stick data
	bool _terrain_hold{false}; /**< true when vehicle is controlling height above a static ground position */

	float _velocity_constraint_up{INFINITY};
	float _velocity_constraint_down{INFINITY};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_HOLD_MAX_Z>) _param_mpc_hold_max_z,
					(ParamInt<px4::params::MPC_ALT_MODE>) _param_mpc_alt_mode,
					(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) _param_mpc_hold_max_xy,
					(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p, /**< position controller altitude propotional gain */
					(ParamFloat<px4::params::MPC_LAND_ALT1>) _param_mpc_land_alt1, /**< altitude at which to start downwards slowdown */
					(ParamFloat<px4::params::MPC_LAND_ALT2>) _param_mpc_land_alt2, /**< altitude below which to land with land speed */
					(ParamFloat<px4::params::MPC_LAND_SPEED>)
					_param_mpc_land_speed, /**< desired downwards speed when approaching the ground */
					(ParamFloat<px4::params::MPC_TKO_SPEED>)
					_param_mpc_tko_speed /**< desired upwards speed when still close to the ground */
				       )
private:
	/**
	 * Terrain following.
	 * During terrain following, the position setpoint is adjusted
	 * such that the distance to ground is kept constant.
	 * @param apply_brake is true if user wants to break
	 * @param stopped is true if vehicle has stopped moving in D-direction
	 */
	void _terrainFollowing(bool apply_brake, bool stopped);

	/**
	 * Minimum Altitude during range sensor operation.
	 * If a range sensor is used for altitude estimates, for
	 * best operation a minimum altitude is required. The minimum
	 * altitude is only enforced during altitude lock.
	 */
	void _respectMinAltitude();

	void _respectMaxAltitude();

	/**
	 * Sets downwards velocity constraint based on the distance to ground.
	 * To ensure a slowdown to land speed before hitting the ground.
	 */
	void _respectGroundSlowdown();

	void setGearAccordingToSwitch();

	bool _updateYawCorrection();

	uint8_t _reset_counter = 0; /**< counter for estimator resets in z-direction */

	float _min_distance_to_ground{(float)(-INFINITY)}; /**< min distance to ground constraint */
	float _max_distance_to_ground{(float)INFINITY};  /**< max distance to ground constraint */

	/**
	 * Distance to ground during terrain following.
	 * If user does not demand velocity change in D-direction and the vehcile
	 * is in terrain-following mode, then height to ground will be locked to
	 * _dist_to_ground_lock.
	 */
	float _dist_to_ground_lock = NAN;
};
