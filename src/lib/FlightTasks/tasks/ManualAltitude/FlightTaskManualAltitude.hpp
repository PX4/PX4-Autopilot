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
 * @file FlightManualAltitude.hpp
 *
 * Flight task for manual controlled altitude.
 */

#pragma once

#include "FlightTaskManualStabilized.hpp"

class FlightTaskManualAltitude : public FlightTaskManualStabilized
{
public:
	FlightTaskManualAltitude() = default;
	virtual ~FlightTaskManualAltitude() = default;
	bool activate() override;
	bool updateInitialize() override;

protected:
	void _updateSetpoints() override; /**< updates all setpoints */
	void _scaleSticks() override; /**< scales sticks to velocity in z */

	/**
	 *  Check and sets for position lock.
	 *  If sticks are at center position, the vehicle
	 *  will exit velocity control and enter position control.
	 */
	void _updateAltitudeLock();

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskManualStabilized,
					(ParamFloat<px4::params::MPC_HOLD_MAX_Z>) MPC_HOLD_MAX_Z,
					(ParamInt<px4::params::MPC_ALT_MODE>) MPC_ALT_MODE,
					(ParamFloat<px4::params::MPC_HOLD_MAX_XY>) MPC_HOLD_MAX_XY,
					(ParamFloat<px4::params::MPC_Z_P>) MPC_Z_P
				       )
private:
	uint8_t _reset_counter = 0; /**< counter for estimator resets in z-direction */
	float _max_speed_up = 10.0f;
	float _min_speed_down = 1.0f;
	bool _terrain_follow{false}; /**< true when the vehicle is following the terrain height */
	bool _terrain_hold{false}; /**< true when vehicle is controlling height above a static ground position */

	/**
	 * Distance to ground during terrain following.
	 * If user does not demand velocity change in D-direction and the vehcile
	 * is in terrain-following mode, then height to ground will be locked to
	 * _dist_to_ground_lock.
	 */
	float _dist_to_ground_lock = NAN;

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
};
