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

#pragma once

// PX4 includes
#include <px4_platform_common/module_params.h>
#include <lib/pure_pursuit/PurePursuit.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rover_mecanum_guidance_status.h>
#include <uORB/topics/rover_mecanum_setpoint.h>

// Standard libraries
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <math.h>

using namespace matrix;

/**
 * @brief Class for mecanum rover guidance.
 */
class RoverMecanumGuidance : public ModuleParams
{
public:
	/**
	 * @brief Constructor for RoverMecanumGuidance.
	 * @param parent The parent ModuleParams object.
	 */
	RoverMecanumGuidance(ModuleParams *parent);
	~RoverMecanumGuidance() = default;

	/**
	 * @brief Compute guidance for the vehicle.
	 * @param yaw The yaw orientation of the vehicle in radians.
	 * @param nav_state Navigation state of the rover.
	 */
	void computeGuidance(float yaw, int nav_state);

	void setDesiredYaw(float desired_yaw) { _desired_yaw = desired_yaw; };

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Update uORB subscriptions
	 */
	void updateSubscriptions();

	/**
	 * @brief Update global/ned waypoint coordinates
	 */
	void updateWaypoints();

	// uORB subscriptions
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};

	// uORB publications
	uORB::Publication<rover_mecanum_guidance_status_s> _rover_mecanum_guidance_status_pub{ORB_ID(rover_mecanum_guidance_status)};
	uORB::Publication<rover_mecanum_setpoint_s> _rover_mecanum_setpoint_pub{ORB_ID(rover_mecanum_setpoint)};

	// Variables
	MapProjection _global_ned_proj_ref{}; // Transform global to ned coordinates.
	PurePursuit _pure_pursuit{this}; // Pure pursuit library
	bool _mission_finished{false};

	// Waypoints
	Vector2d _curr_pos{};
	Vector2f _curr_pos_ned{};
	Vector2d _prev_wp{};
	Vector2f _prev_wp_ned{};
	Vector2d _curr_wp{};
	Vector2f _curr_wp_ned{};
	Vector2d _next_wp{};
	Vector2f _next_wp_ned{};
	Vector2d _home_position{};
	float _max_velocity_magnitude{0.f};
	float _waypoint_transition_angle{0.f}; // Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	float _desired_yaw{0.f};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RM_MAX_SPEED>) _param_rm_max_speed,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad,
		(ParamFloat<px4::params::RM_MAX_JERK>) _param_rm_max_jerk,
		(ParamFloat<px4::params::RM_MAX_ACCEL>) _param_rm_max_accel,
		(ParamFloat<px4::params::RM_MAX_YAW_RATE>) _param_rm_max_yaw_rate,
		(ParamFloat<px4::params::RM_MISS_VEL_GAIN>) _param_rm_miss_vel_gain
	)
};
