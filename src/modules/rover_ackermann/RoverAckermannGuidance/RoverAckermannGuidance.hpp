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
#include <uORB/topics/rover_ackermann_guidance_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/home_position.h>

// Standard library includes
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <math.h>
#include <lib/pid/pid.h>

using namespace matrix;

/**
 * @brief Class for ackermann drive guidance.
 */
class RoverAckermannGuidance : public ModuleParams
{
public:
	/**
	 * @brief Constructor for RoverAckermannGuidance.
	 * @param parent The parent ModuleParams object.
	 */
	RoverAckermannGuidance(ModuleParams *parent);
	~RoverAckermannGuidance() = default;

	/**
	 * @brief Struct for steering and throttle setpoints.
	 * @param steering Steering setpoint.
	 * @param throttle Throttle setpoint.
	 */
	struct motor_setpoint {
		float steering{0.f};
		float throttle{0.f};
	};

	/**
	 * @brief Calculate motor setpoints based on the mission plan.
	 * @param nav_state Vehicle navigation state.
	 * @return Motor setpoints for throttle and steering.
	 */
	motor_setpoint computeGuidance(int nav_state);

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
	 * @brief Update global/NED waypoint coordinates and acceptance radius.
	 */
	void updateWaypointsAndAcceptanceRadius();

	/**
	 * @brief Publish the acceptance radius for current waypoint based on the angle between a line segment
	 * from the previous to the current waypoint/current to the next waypoint and maximum steer angle of the vehicle.
	 * @param curr_wp_ned Current waypoint in NED frame [m].
	 * @param prev_wp_ned Previous waypoint in NED frame [m].
	 * @param next_wp_ned Next waypoint in NED frame [m].
	 * @param default_acceptance_radius Default acceptance radius for waypoints [m].
	 * @param acceptance_radius_gain Tuning parameter that scales the geometric optimal acceptance radius for the corner cutting [-].
	 * @param acceptance_radius_max Maximum value for the acceptance radius [m].
	 * @param wheel_base Rover wheelbase [m].
	 * @param max_steer_angle Rover maximum steer angle [rad].
	 * @return Updated acceptance radius [m].
	 */
	float updateAcceptanceRadius(const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
				     const Vector2f &next_wp_ned, float default_acceptance_radius, float acceptance_radius_gain,
				     float acceptance_radius_max, float wheel_base, float max_steer_angle);


	/**
	 * @brief Calculate the desired speed setpoint. During cornering the speed is calculated as the inverse
	 * of the acceptance radius multiplied with a tuning factor. On straight lines it is based on a velocity trajectory
	 * such that the rover will arrive at the next corner with the desired cornering speed under consideration of the
	 * maximum acceleration and jerk.
	 * @param miss_vel_def Default desired velocity for the rover during mission [m/s].
	 * @param miss_vel_min Minimum desired velocity for the rover during mission [m/s].
	 * @param miss_vel_gain Tuning parameter for the slow down effect during cornering [-].
	 * @param distance_to_prev_wp Distance to the previous waypoint [m].
	 * @param distance_to_curr_wp Distance to the current waypoint [m].
	 * @param acc_rad Acceptance radius of the current waypoint [m].
	 * @param prev_acc_rad Acceptance radius of the previous waypoint [m].
	 * @param max_accel Maximum allowed acceleration for the rover [m/s^2].
	 * @param max_jerk Maximum allowed jerk for the rover [m/s^3].
	 * @param nav_state Current nav_state of the rover.
	 * @return Speed setpoint for the rover [m/s].
	 */
	float calcDesiredSpeed(float miss_vel_def, float miss_vel_min, float miss_vel_gain, float distance_to_prev_wp,
			       float distance_to_curr_wp, float acc_rad, float prev_acc_rad, float max_accel, float max_jerk, int nav_state);

	/**
	 * @brief Calculate desired steering angle. The desired steering is calulated as the steering that is required to
	 * reach the point calculated using the pure pursuit algorithm (see PurePursuit.hpp).
	 * @param pure_pursuit Pure pursuit class instance.
	 * @param curr_wp_ned Current waypoint in NED frame [m].
	 * @param prev_wp_ned Previous waypoint in NED frame [m].
	 * @param curr_pos_ned Current position of the vehicle in NED frame [m].
	 * @param wheel_base Rover wheelbase [m].
	 * @param desired_speed Desired speed for the rover [m/s].
	 * @param vehicle_yaw Current yaw of the rover [rad].
	 * @param max_steering Maximum steering angle of the rover [rad].
	 * @return Steering setpoint for the rover [rad].
	 */
	float calcDesiredSteering(PurePursuit &pure_pursuit, const Vector2f &curr_wp_ned, const Vector2f &prev_wp_ned,
				  const Vector2f &curr_pos_ned, float wheel_base, float desired_speed, float vehicle_yaw, float max_steering);

	/**
	 * @brief Calculate the throttle setpoint. Calculated with a PID controller using the difference between
	 * the desired/actual speed and a feedforward term based on the full throttle speed.
	 * @param pid_throttle Reference to PID instance.
	 * @param desired_speed Reference speed for the rover [m/s].
	 * @param actual_speed Actual speed of the rover [m/s].
	 * @param max_speed Rover speed at full throttle [m/s].
	 * @param dt Time interval since last update [s].
	 * @return Normalized throttle setpoint [0, 1].
	 */
	float calcThrottleSetpoint(PID_t &pid_throttle, float desired_speed, float actual_speed, float max_speed, float dt);

	// uORB subscriptions
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};

	// uORB publications
	uORB::Publication<rover_ackermann_guidance_status_s> _rover_ackermann_guidance_status_pub{ORB_ID(rover_ackermann_guidance_status)};
	uORB::Publication<position_controller_status_s>	_position_controller_status_pub{ORB_ID(position_controller_status)};
	rover_ackermann_guidance_status_s _rover_ackermann_guidance_status{};

	// Class instances
	MapProjection _global_ned_proj_ref{}; // Transform global to NED coordinates
	PurePursuit _pure_pursuit{this}; // Pure pursuit library

	// Rover variables
	float _desired_steering{0.f};
	float _vehicle_yaw{0.f};
	float _desired_speed{0.f};
	float _actual_speed{0.f};
	Vector2d _curr_pos{};
	Vector2f _curr_pos_ned{};
	PID_t _pid_throttle;
	hrt_abstime _timestamp{0};

	// Waypoint variables
	Vector2d _home_position{};
	Vector2f _curr_wp_ned{};
	Vector2f _prev_wp_ned{};
	Vector2f _next_wp_ned{};
	Vector2d _curr_wp{};
	Vector2d _prev_wp{};
	Vector2d _next_wp{};
	float _distance_to_prev_wp{0.f};
	float _distance_to_curr_wp{0.f};
	float _distance_to_next_wp{0.f};
	float _acceptance_radius{0.5f};
	float _prev_acceptance_radius{0.5f};
	float _wp_max_desired_vel{0.f};
	bool _mission_finished{false};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RA_WHEEL_BASE>) _param_ra_wheel_base,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>) _param_ra_max_steer_angle,
		(ParamFloat<px4::params::RA_ACC_RAD_MAX>) _param_ra_acc_rad_max,
		(ParamFloat<px4::params::RA_ACC_RAD_GAIN>) _param_ra_acc_rad_gain,
		(ParamFloat<px4::params::RA_MISS_VEL_DEF>) _param_ra_miss_vel_def,
		(ParamFloat<px4::params::RA_MISS_VEL_MIN>) _param_ra_miss_vel_min,
		(ParamFloat<px4::params::RA_MISS_VEL_GAIN>) _param_ra_miss_vel_gain,
		(ParamFloat<px4::params::RA_SPEED_P>) _param_ra_p_speed,
		(ParamFloat<px4::params::RA_SPEED_I>) _param_ra_i_speed,
		(ParamFloat<px4::params::RA_MAX_SPEED>) _param_ra_max_speed,
		(ParamFloat<px4::params::RA_MAX_JERK>) _param_ra_max_jerk,
		(ParamFloat<px4::params::RA_MAX_ACCEL>) _param_ra_max_accel,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad
	)
};
