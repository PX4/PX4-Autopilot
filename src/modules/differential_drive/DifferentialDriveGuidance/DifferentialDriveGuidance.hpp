/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module_params.h>

#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <math.h>

#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/differential_drive_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>

#include <lib/pid/pid.h>


/**
 * @brief Enum class for the different states of guidance.
 */
enum class GuidanceState {
	TURNING, ///< The vehicle is currently turning.
	DRIVING, ///< The vehicle is currently driving straight.
	GOAL_REACHED ///< The vehicle has reached its goal.
};

/**
 * @brief Class for differential drive guidance.
 */
class DifferentialDriveGuidance : public ModuleParams
{
public:
	/**
	 * @brief Constructor for DifferentialDriveGuidance.
	 * @param parent The parent ModuleParams object.
	 */
	DifferentialDriveGuidance(ModuleParams *parent);
	~DifferentialDriveGuidance() = default;

	/**
	 * @brief Compute guidance for the vehicle.
	 * @param global_pos The global position of the vehicle in degrees.
	 * @param current_waypoint The current waypoint the vehicle is heading towards in degrees.
	 * @param next_waypoint The next waypoint the vehicle will head towards after reaching the current waypoint in degrees.
	 * @param vehicle_yaw The yaw orientation of the vehicle in radians.
	 * @param body_velocity The velocity of the vehicle in m/s.
	 * @param angular_velocity The angular velocity of the vehicle in rad/s.
	 * @param dt The time step in seconds.
	 */
	void computeGuidance(float yaw, float angular_velocity, float dt);

	/**
	 * @brief Set the maximum speed for the vehicle.
	 * @param max_speed The maximum speed in m/s.
	 * @return The set maximum speed in m/s.
	 */
	float setMaxSpeed(float max_speed) { return _max_speed = max_speed; }


	/**
	 * @brief Set the maximum angular velocity for the vehicle.
	 * @param max_angular_velocity The maximum angular velocity in rad/s.
	 * @return The set maximum angular velocity in rad/s.
	 */
	float setMaxAngularVelocity(float max_angular_velocity) { return _max_angular_velocity = max_angular_velocity; }

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};

	uORB::Publication<differential_drive_setpoint_s> _differential_drive_setpoint_pub{ORB_ID(differential_drive_setpoint)};

	position_setpoint_triplet_s _position_setpoint_triplet{};
	vehicle_global_position_s _vehicle_global_position{};

	GuidanceState _currentState; ///< The current state of guidance.

	float _desired_angular_velocity; ///< The desired angular velocity.

	float _max_speed; ///< The maximum speed.
	float _max_angular_velocity; ///< The maximum angular velocity.

	matrix::Vector2d _current_waypoint; ///< The current waypoint.

	VelocitySmoothing _forwards_velocity_smoothing; ///< The velocity smoothing for forward motion.
	PositionSmoothing _position_smoothing; ///< The position smoothing.

	PID_t _heading_p_controller; ///< The PID controller for yaw rate.

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RDD_P_HEADING>) _param_rdd_p_gain_heading,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad,
		(ParamFloat<px4::params::RDD_MAX_JERK>) _param_rdd_max_jerk,
		(ParamFloat<px4::params::RDD_MAX_ACCEL>) _param_rdd_max_accel
	)
};
