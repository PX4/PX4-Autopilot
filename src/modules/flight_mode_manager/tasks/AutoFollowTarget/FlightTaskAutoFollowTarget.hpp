/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoFollowTarget.hpp
 *
 * Flight task for autonomous, gps driven follow-me mode.
 *
 * @author Alessandro Simovic <potaito-dev@protonmail.com>
 */

#pragma once

#include "FlightTaskAuto.hpp"
#include "follow_target_estimator/TargetEstimator.hpp"
#include "Sticks.hpp"

#include <parameters/param.h>
#include <mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/follow_target_status.h>
#include <uORB/topics/follow_target_estimator.h>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/vehicle_command.h>

#include <lib/mathlib/math/filter/second_order_reference_model.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/helper_functions.hpp>

// Minimum safety altitude above home (or bottom distance sensor)
// underneath which the flight task will stop moving horizontally
static constexpr float MINIMUM_SAFETY_ALTITUDE = 1.0f;

// [m] max vertical deviation from position setpoint, above
// which no horizontal control is done
static constexpr float ALT_ACCEPTANCE_THRESHOLD = 3.0f;

// Vertical ascent speed when the drone detects that it
// is too close to the ground (below MINIMUM_SAFETY_ALTITUDE)
static constexpr float EMERGENCY_ASCENT_SPEED = 0.2f;

// [m] Minimum distance between drone and target for the drone to do any yaw control.
static constexpr float MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL = 1.0f;

// Second order filter parameter for target position filter
static constexpr float TARGET_POSE_FILTER_NATURAL_FREQUENCY = 1.0f; // [rad/s]
static constexpr float TARGET_POSE_FILTER_DAMPING_RATIO = 0.7071;

// [us] If the target estimator output isn't updated longer than this, reset pose filter.
static constexpr uint64_t TARGET_ESTIMATOR_TIMEOUT_US = 1500000UL;

// [m/s] Velocity deadzone for which, under this velocity, the target orientation
// tracking will freeze, since orientation can be noisy in low velocities
static constexpr float TARGET_VELOCITY_DEADZONE_FOR_ORIENTATION_TRACKING = 1.0;

// [m/s] Velocity limit to limit orbital angular rate depending on follow distance
static constexpr float MAXIMUM_TANGENTIAL_ORBITING_SPEED = 5.0;

// Yaw setpoint filter to avoid jitter-ness, which can happen because the yaw is
// calculated off of position offset between target & drone, which updates very frequently.
static constexpr float YAW_SETPOINT_FILTER_TIME_CONSTANT = 0.1;

// Yaw setpoint enabler varaible to keep the yaw filtering scheme
static constexpr bool YAW_SETPOINT_FILTER_ENABLE = true;

// [m/s] Speed to which follow distance will be adjusted by, when commanded in full deflection via RC command
static constexpr float FOLLOW_DISTANCE_USER_ADJUST_SPEED = 1.5;

// [m/s] Speed to which follow height will be adjusted by, when commanded in full deflection via RC command
static constexpr float FOLLOW_HEIGHT_USER_ADJUST_SPEED = 1.0;

// [rad/s] Angular to which follow distance will be adjusted by, when commanded in full deflection via RC command
static constexpr float FOLLOW_ANGLE_USER_ADJUST_SPEED = 1.5;


class FlightTaskAutoFollowTarget : public FlightTask
{
public:
	FlightTaskAutoFollowTarget();
	virtual ~FlightTaskAutoFollowTarget();

	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool update() override;

protected:
	// Follow Perspectives set by the parameter NAV_FT_FS
	enum {
		FOLLOW_PERSPECTIVE_NONE,
		FOLLOW_PERSPECTIVE_BEHIND,
		FOLLOW_PERSPECTIVE_FRONT,
		FOLLOW_PERSPECTIVE_FRONT_RIGHT,
		FOLLOW_PERSPECTIVE_FRONT_LEFT,
		FOLLOW_PERSPECTIVE_MID_RIGHT,
		FOLLOW_PERSPECTIVE_MID_LEFT,
		FOLLOW_PERSPECTIVE_BEHIND_RIGHT,
		FOLLOW_PERSPECTIVE_BEHIND_LEFT,
		FOLLOW_PERSPECTIVE_MIDDLE_FOLLOW,
		FOLLOW_PERSPECTIVE_INVALID  // Leave this as last!
	};

	// Angles [deg] for the different follow-me perspectives
	enum {
		FOLLOW_PERSPECTIVE_BEHIND_ANGLE_DEG = 180,
		FOLLOW_PERSPECTIVE_FRONT_ANGLE_DEG = 0,
		FOLLOW_PERSPECTIVE_FRONT_RIGHT_ANGLE_DEG = 45,
		FOLLOW_PERSPECTIVE_FRONT_LEFT_ANGLE_DEG = 315,
		FOLLOW_PERSPECTIVE_MID_RIGHT_ANGLE_DEG = 90,
		FOLLOW_PERSPECTIVE_MID_LEFT_ANGLE_DEG = 270,
		FOLLOW_PERSPECTIVE_BEHIND_RIGHT_ANGLE_DEG = 135,
		FOLLOW_PERSPECTIVE_BEHIND_LEFT_ANGLE_DEG = 225
	};

	// Follow Altitude modes set by the parameter NAV_FT_ALT_M
	enum {
		FOLLOW_ALTITUDE_MODE_CONSTANT,
		FOLLOW_ALTITUDE_MODE_TRACK_TERRAIN,
		FOLLOW_ALTITUDE_MODE_TRACK_TARGET
	};

	/**
	 * Get the RC command from the user to adjust Follow Angle, Distance and Height internally
	 */
	void update_stick_command();

	/**
	 * Update the Second Order Target Pose Filter to track kinematically feasible target position and velocity
	 */
	void update_target_pose_filter(follow_target_estimator_s follow_target_estimator);

	/**
	 * Calculate the tracked target orientation
	 *
	 * @return Angle [rad] Tracked target orientation (heading)
	 */
	float update_target_orientation(Vector2f target_velocity);

	/**
	 * Release Gimbal Control
	 *
	 * Releases Gimbal Control Authority of Follow-Target Flight Task, to allow other modules / Ground station
	 * to control the gimbal when the task exits.
	 */
	void release_gimbal_control();

	/**
	 * Updates the orbit angle setpoint, taking into account the maximal orbit tangential speed
	 *
	 * @return Angle [rad] Next feasible orbit angle setpoint
	 */
	float update_orbit_angle(float target_orientation, float fllow_angle);

	/**
	 * Calculates desired drone position, taking into account the follow target altitude mode
	 *
	 * @return Position [Vector3f] Final position setpoint for the drone
	 */
	Vector3f calculate_desired_drone_position(Vector3f target_position);

	/**
	 * Calculate the gimbal height offset to the target to calculate the pitch angle command
	 *
	 * @return Height [m] Difference between the target and the drone
	 */
	float calculate_gimbal_height(float target_height);

	/**
	 * Publishes gimbal control command to track the target, given xy distance and z (height) difference.
	 */
	void point_gimbal_at(float xy_distance, float z_distance);

	/**
	 * Get the current follow-me perspective setting from PX4 parameters
	 *
	 * @param param_nav_ft_fs value of the parameter NAV_FT_FS
	 * @return Angle [deg] from which the drone should view the target while following it, with zero degrees indicating the target's 12 o'clock
	 */
	float get_follow_me_angle_setting(int param_nav_ft_fs) const;

	// Sticks object to read in stick commands from the user
	Sticks _sticks;

	// Estimator for target position and velocity
	TargetEstimator _target_estimator;
	follow_target_estimator_s _follow_target_estimator;

	// Last target estimator timestamp to handle timeout filter reset
	uint64_t _last_valid_target_estimator_timestamp{0};

	// Second Order Filter to calculate kinematically feasible target position
	SecondOrderReferenceModel<matrix::Vector3f> _target_pose_filter;

	// Internally tracked Follow Target characteristics, to allow RC control input adjustments
	float _follow_target_distance{0.0f};
	float _follow_target_height{0.0f};
	float _follow_angle_rad{0.0f}; // 0 degrees following from front, and then clockwise rotation

	// Estimated (Filtered) target orientation setpoint
	float _target_orientation_rad{0.0f};

	// Orbit angle setpoint, measured in global frame, against the target
	float _orbit_angle_setpoint{0.0f};

	// Actual drone to target heading [rad]
	float _drone_to_target_heading{0.0f};

	// Actual orbit angle in relation to the target
	float _measured_orbit_angle{0.0f};

	// Actual drone to target 2d position vector
	Vector2f _drone_to_target_vector{0.0f, 0.0f};

	// Tracked orbit tangential speed, to compensate for the orbital motion for velocity setpoints
	Vector2f _orbit_tangential_velocity{0.0f, 0.0f};

	// NOTE: If more of these internal state variables come into existence, it
	// would make sense to create an internal state machine with a single enum
	bool _emergency_ascent = false;

	// Yaw setpoint filter to remove jitter-ness
	AlphaFilter<float> _yaw_setpoint_filter;

	// Variable to remember the home position's z coordinate, which will be baseline for the position z setpoint
	float _home_position_z{0.0f};

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		FlightTask,
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::MAV_COMP_ID>) _param_mav_comp_id,
		(ParamFloat<px4::params::NAV_MIN_FT_HT>) _param_nav_min_ft_ht,
		(ParamFloat<px4::params::NAV_FT_MIN_HT>) _param_nav_ft_min_ht,
		(ParamFloat<px4::params::NAV_FT_DST>) _param_nav_ft_dst,
		(ParamInt<px4::params::NAV_FT_FS>) _param_nav_ft_fs,
		(ParamInt<px4::params::NAV_FT_ALT_M>) _param_nav_ft_alt_m,
		(ParamFloat<px4::params::NAV_FT_YAW_T>) _param_ft_yaw_t
	)

	uORB::Subscription _follow_target_estimator_sub{ORB_ID(follow_target_estimator)};

	uORB::Publication<follow_target_status_s> _follow_target_status_pub{ORB_ID(follow_target_status)};
	uORB::Publication<gimbal_manager_set_attitude_s> _gimbal_manager_set_attitude_pub{ORB_ID(gimbal_manager_set_attitude)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	// Debugging
	float _gimbal_pitch{0};
};
