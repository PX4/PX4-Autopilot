/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
	FlightTaskAutoLine(control::SuperBlock *parent, const char *name);

	virtual ~FlightTaskAutoLine() = default;

	bool activate() override;

	bool update() override;

protected:

	matrix::Vector2f _vel_sp_xy{};
	matrix::Vector2f _pos_sp_xy{};
	float _vel_sp_z = 0.0f;
	float _pos_sp_z = 0.0f;

	matrix::Vector3f _destination{}; /**< Current target. Is not necessarily the same as triplet target. */
	matrix::Vector3f _origin{}; /**< Previous waypoint. Is not necessarily the same as triplet previous. */
	float _speed_at_target = 0.0f; /**< Desired velocity at target. */

	enum class State {
		offtrack, /**< Vehicle is more than cruise speed away from track */
		target_behind, /**< Vehicle is in front of target. */
		previous_infront, /**< Vehilce is behind previous waypoint.*/
		none /**< Vehicle is in normal tracking mode from triplet previous to triplet target */
	};
	State _current_state{State::none};

	control::BlockParamFloat _land_speed; /**< Downward speed during landing. */
	control::BlockParamFloat _vel_max_up; /**< Maximum upward velocity. */
	control::BlockParamFloat _vel_max_down; /**< Maximum downward velocity. */
	control::BlockParamFloat _acc_max_xy; /**< Maximum acceleration in hover. */
	control::BlockParamFloat _acc_xy; /**< Maximum acceleration from hover to fast forward flight. */
	control::BlockParamFloat _acc_max_up; /**< Max acceleration up. */
	control::BlockParamFloat _acc_max_down; /**< Max acceleration down. */
	control::BlockParamFloat _cruise_speed_90; /**< Speed when angle is 90 degrees between prev-current/current-next. */

	/* None-position control params */
	control::BlockParamFloat _nav_rad; /**< Radius that is used by navigator that defines when to update triplets */
	control::BlockParamFloat _mis_yaw_error; /**< Yaw threshold during mission to consider yaw as accepted */

	void _generateIdleSetpoints();
	void _generateLandSetpoints();
	void _generateVelocitySetpoints();
	void _generateTakeoffSetpoints();
	void _updateInternalWaypoints(); /* Depending on state of vehicle, the internal waypoints might differ from target (for instance if offtrack). */
	void _generateSetpoints(); /**< Generate velocity and position setpoint for following line. */
	void _generateAltitudeSetpoints(); /**< Generate velocity and position setpoints for following line along z. */
	void _generateXYsetpoints(); /**< Generate velocity and position setpoints for following line along xy. */

private:
	float _getVelocityFromAngle(const float angle); /** Computes the speed at target depending on angle. */
	void _reset(); /** Resets setpoint. */
	float _getVelcoityFromAngle(const float angle); /** Computes the speed at target depending on angle. */
};
