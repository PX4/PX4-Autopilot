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
 * @file FlightTaskAutoLine.hpp
 *
 * Flight task for autonomous, gps driven mode. The vehicle flies
 * along a straight line in between waypoints.
 */

#pragma once

#include "FlightTaskAuto.hpp"

class FlightTaskAutoLine : public FlightTaskAuto
{
public:
	FlightTaskAutoLine() = default;
	virtual ~FlightTaskAutoLine() = default;
	bool activate() override;
	bool update() override;

protected:

	matrix::Vector3f _destination{}; /**< Current target. Is not necessarily the same as triplet target. */
	matrix::Vector3f _origin{}; /**< Previous waypoint. Is not necessarily the same as triplet previous. */
	float _speed_at_target = 0.0f; /**< Desired velocity at target. */
	float _alt_above_ground{0.0f}; /**< If home provided, then it is altitude above home, otherwise it is altitude above local position reference. */

	enum class State {
		offtrack, /**< Vehicle is more than cruise speed away from track */
		target_behind, /**< Vehicle is in front of target. */
		previous_infront, /**< Vehilce is behind previous waypoint.*/
		none /**< Vehicle is in normal tracking mode from triplet previous to triplet target */
	};
	State _current_state{State::none};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAuto,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) MPC_LAND_SPEED,
					(ParamFloat<px4::params::MPC_CRUISE_90>) MPC_CRUISE_90, // speed at corner when angle is 90 degrees
					(ParamFloat<px4::params::NAV_ACC_RAD>) NAV_ACC_RAD, // acceptance radius at which waypoints are updated
					(ParamFloat<px4::params::MIS_YAW_ERR>) MIS_YAW_ERR, // yaw-error threshold
					(ParamFloat<px4::params::MPC_LAND_ALT1>) MPC_LAND_ALT1, // altitude at which speed limit downwards reaches maximum speed
					(ParamFloat<px4::params::MPC_LAND_ALT2>) MPC_LAND_ALT2, // altitude at which speed limit downwards reached minimum speed
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) MPC_ACC_UP_MAX,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) MPC_ACC_DOWN_MAX,
					(ParamFloat<px4::params::MPC_ACC_HOR>) MPC_ACC_HOR, // acceleration in flight
					(ParamFloat<px4::params::MPC_TILTMAX_LND>) MPC_TILTMAX_LND,
					(ParamFloat<px4::params::MPC_TKO_SPEED>) MPC_TKO_SPEED
				       )

	void _generateIdleSetpoints();
	void _generateLandSetpoints();
	void _generateVelocitySetpoints();
	void _generateTakeoffSetpoints();
	void _updateInternalWaypoints(); /**< Depending on state of vehicle, the internal waypoints might differ from target (for instance if offtrack). */
	void _generateSetpoints(); /**< Generate velocity and position setpoint for following line. */
	void _generateAltitudeSetpoints(); /**< Generate velocity and position setpoints for following line along z. */
	void _updateAltitudeAboveGround(); /**< Computes altitude above ground based on sensors available. */
	void _generateXYsetpoints(); /**< Generate velocity and position setpoints for following line along xy. */
	void updateParams() override; /**< See ModuleParam class */

private:
	float _getVelocityFromAngle(const float angle); /**< Computes the speed at target depending on angle. */
	bool _highEnoughForLandingGear(); /**< Checks if gears can be lowered. */
	void _reset(); /**< Resets member variables to current vehicle state */
	WaypointType _type_previous{WaypointType::idle}; /**< Previous type of current target triplet. */

};
