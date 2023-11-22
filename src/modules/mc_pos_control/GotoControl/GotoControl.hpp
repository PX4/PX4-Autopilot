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
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>

class GotoControl
{
public:
	GotoControl() = default;
	~GotoControl() = default;

	/** @brief struct containing maximum vehicle translational and rotational constraints */
	struct GotoConstraints {
		float max_horizontal_speed; // [m/s]
		float max_down_speed; // [m/s]
		float max_up_speed; // [m/s]
		float max_horizontal_accel; // [m/s^2]
		float max_down_accel; // [m/s^2]
		float max_up_accel; // [m/s^2]
		float max_jerk; // [m/s^3]
		float max_heading_rate; // [rad/s]
		float max_heading_accel; // [rad/s^2]
	};

	/**
	 * @brief sets the maximum vehicle translational and rotational constraints. note these can be more conservatively
	 * overriden (e.g. slowed down) via the speed scalers in the go-to setpoint.
	 *
	 * @param vehicle_constraints Struct containing desired vehicle constraints
	 */
	void setGotoConstraints(const GotoConstraints &vehicle_constraints)
	{
		_goto_constraints.max_horizontal_speed = math::max(0.f, vehicle_constraints.max_horizontal_speed);
		_goto_constraints.max_down_speed = math::max(0.f, vehicle_constraints.max_down_speed);
		_goto_constraints.max_up_speed = math::max(0.f, vehicle_constraints.max_up_speed);
		_goto_constraints.max_horizontal_accel = math::max(0.f,
				vehicle_constraints.max_horizontal_accel);
		_goto_constraints.max_down_accel = math::max(0.f, vehicle_constraints.max_down_accel);
		_goto_constraints.max_up_accel = math::max(0.f, vehicle_constraints.max_up_accel);
		_goto_constraints.max_jerk = math::max(0.f, vehicle_constraints.max_jerk);
		_goto_constraints.max_heading_rate = math::max(0.f,
						     vehicle_constraints.max_heading_rate);
		_goto_constraints.max_heading_accel = math::max(0.f,
						      vehicle_constraints.max_heading_accel);
	}

	/** @param error [m] position smoother's maximum allowed horizontal position error at which trajectory integration halts */
	void setMaxAllowedHorizontalPositionError(const float error) { _position_smoothing.setMaxAllowedHorizontalError(error); }

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
	void update(const float dt, const matrix::Vector3f &position, const float heading,
		    const goto_setpoint_s &goto_setpoint, trajectory_setpoint_s &trajectory_setpoint);

private:
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

	GotoConstraints _goto_constraints{};

	PositionSmoothing _position_smoothing;
	HeadingSmoothing _heading_smoothing;

	// flags that the next update() requires a valid current vehicle position to reset the smoothers
	bool _need_smoother_reset{true};

	// flags if the last update() was controlling heading
	bool _controlling_heading{false};
};
