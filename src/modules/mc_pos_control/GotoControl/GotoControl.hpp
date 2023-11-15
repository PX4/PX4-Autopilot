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
 * Be sure to set constraints with setVehicleConstraints() before calling the update() method for the first time, otherwise
 * the default motion will be VERY slow.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/HeadingSmoother.hpp>
#include <uORB/topics/goto_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>

class GotoControl
{
public:
	GotoControl() = default;
	~GotoControl() = default;

	/**
	 * @brief struct containing maximum vehicle translational and rotational constraints
	 *
	 */
	struct VehicleConstraints {
		float max_horizontal_speed = kMinSpeed; // [m/s]
		float max_down_speed = kMinSpeed; // [m/s]
		float max_up_speed = kMinSpeed; // [m/s]
		float max_horizontal_accel = VelocitySmoothing::kMinAccel; // [m/s^2]
		float max_down_accel = VelocitySmoothing::kMinAccel; // [m/s^2]
		float max_up_accel = VelocitySmoothing::kMinAccel; // [m/s^2]
		float max_jerk = VelocitySmoothing::kMinJerk; // [m/s^3]
		float max_heading_rate = HeadingSmoother::kMinHeadingRate; // [rad/s]
		float max_heading_accel = HeadingSmoother::kMinHeadingAccel; // [rad/s^2]
	};

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
	 * @brief sets the maximum vehicle translational and rotational constraints. note these can be more conservatively
	 * overriden (e.g. slowed down) via the speed scalers in the go-to setpoint.
	 *
	 * @param vehicle_constraints Struct containing desired vehicle constraints
	 */
	void setVehicleConstraints(const VehicleConstraints &vehicle_constraints)
	{
		_vehicle_constraints.max_horizontal_speed = math::max(kMinSpeed, vehicle_constraints.max_horizontal_speed);
		_vehicle_constraints.max_down_speed = math::max(kMinSpeed, vehicle_constraints.max_down_speed);
		_vehicle_constraints.max_up_speed = math::max(kMinSpeed, vehicle_constraints.max_up_speed);
		_vehicle_constraints.max_horizontal_accel = math::max(VelocitySmoothing::kMinAccel,
				vehicle_constraints.max_horizontal_accel);
		_vehicle_constraints.max_down_accel = math::max(VelocitySmoothing::kMinAccel, vehicle_constraints.max_down_accel);
		_vehicle_constraints.max_up_accel = math::max(VelocitySmoothing::kMinAccel, vehicle_constraints.max_up_accel);
		_vehicle_constraints.max_jerk = math::max(VelocitySmoothing::kMinJerk, vehicle_constraints.max_jerk);
		_vehicle_constraints.max_heading_rate = math::max(HeadingSmoother::kMinHeadingRate,
							vehicle_constraints.max_heading_rate);
		_vehicle_constraints.max_heading_accel = math::max(HeadingSmoother::kMinHeadingAccel,
				vehicle_constraints.max_heading_accel);
	}

	/**
	 * @brief Set the position smoother's maximum allowed horizontal position error at which trajectory integration halts
	 *
	 * @param error [m] horizontal position error
	 */
	void setMaxAllowedHorizontalPositionError(const float error)
	{
		_position_smoother.setMaxAllowedHorizontalError(error);
	}

private:

	// [m/s] minimum value of the maximum translational speeds
	static constexpr float kMinSpeed{0.1f};

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

	VehicleConstraints _vehicle_constraints{};
	PositionSmoothing _position_smoother;
	HeadingSmoother _heading_smoother;

	// flags that the next update() requires a valid current vehicle position to reset the smoothers
	bool _need_smoother_reset{true};

	// flags if the last update() was controlling heading
	bool _controlling_heading{false};
};
