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

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_ackermann_guidance_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/mission_result.h>

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

	struct motor_setpoint {
		float steering{0.f};
		float throttle{0.f};
	};

	/**
	 * @brief Compute guidance for ackermann rover and return motor_setpoint for throttle and steering.
	 */
	motor_setpoint purePursuit();

	/**
	 * @brief Update global/local waypoint coordinates and acceptance radius
	 */
	void updateWaypoints();

	/**
	 * @brief Returns and publishes the  acceptance radius for current waypoint based on the angle between a line segment
	 * from the previous to the current waypoint/current to the next waypoint and maximum steer angle of
	 * the vehicle.
	 * @param curr_wp_local Current waypoint in local frame.
	 * @param prev_wp_local Previous waypoint in local frame.
	 * @param next_wp_local Next waypoint in local frame.
	 * @param default_acceptance_radius Default acceptance radius for waypoints.
	 * @param acceptance_radius_gain Scaling of the geometric optimal acceptance radius for the rover to cut corners.
	 * @param acceptance_radius_max Maximum value for the acceptance radius.
	 * @param wheel_base Rover wheelbase.
	 * @param max_steer_angle Rover maximum steer angle.
	 */
	float updateAcceptanceRadius(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local,
				     const Vector2f &next_wp_local, const float &default_acceptance_radius, const float &acceptance_radius_gain,
				     const float &acceptance_radius_max, const float &wheel_base, const float &max_steer_angle);

	/**
	 * @brief Calculate and return desired steering input
	 * @param curr_wp_local Current waypoint in local frame.
	 * @param prev_wp_local Previous waypoint in local frame.
	 * @param curr_pos_local Current position of the vehicle in local frame.
	 * @param lookahead_gain Tuning parameter for the lookahead distance pure pursuit controller.
	 * @param lookahead_min Minimum lookahead distance.
	 * @param lookahead_max Maximum lookahead distance.
	 * @param wheel_base Rover wheelbase.
	 * @param desired_speed Desired speed for the rover.
	 * @param vehicle_yaw Current yaw of the rover.
	 */
	float calcDesiredSteering(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local, const Vector2f &curr_pos_local,
				  const float &lookahead_gain, const float &lookahead_min, const float &lookahead_max, const float &wheel_base,
				  const float &desired_speed, const float &vehicle_yaw);

	/**
	 * @brief Return desired heading to the intersection point between a circle with a radius of
	 * lookahead_distance around the vehicle and a line segment from the previous to the current waypoint.
	 * @param curr_wp_local Current waypoint in local frame.
	 * @param prev_wp_local Previous waypoint in local frame.
	 * @param curr_pos_local Current position of the vehicle in local frame.
	 * @param lookahead_distance Radius of circle around vehicle.
	 */
	float calcDesiredHeading(const Vector2f &curr_wp_local, const Vector2f &prev_wp_local, const Vector2f &curr_pos_local,
				 const float &lookahead_distance);

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	// uORB subscriptions
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};

	// uORB publications
	uORB::Publication<rover_ackermann_guidance_status_s> _rover_ackermann_guidance_status_pub{ORB_ID(rover_ackermann_guidance_status)};
	uORB::Publication<position_controller_status_s>	_position_controller_status_pub{ORB_ID(position_controller_status)};
	rover_ackermann_guidance_status_s _rover_ackermann_guidance_status{};


	MapProjection _global_local_proj_ref{}; // Transform global to local coordinates.

	// Rover variables
	Vector2d _curr_pos{};
	Vector2f _curr_pos_local{};
	PID_t _pid_throttle;
	hrt_abstime _time_stamp_last{0};

	// Waypoint variables
	Vector2d _curr_wp{};
	Vector2d _next_wp{};
	Vector2d _prev_wp{};
	Vector2f _curr_wp_local{};
	Vector2f _prev_wp_local{};
	Vector2f _next_wp_local{};
	float _acceptance_radius{0.5f};
	float _prev_acc_rad{0.f};
	bool _mission_finished{false};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RA_WHEEL_BASE>) _param_ra_wheel_base,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>) _param_ra_max_steer_angle,
		(ParamFloat<px4::params::RA_LOOKAHD_GAIN>) _param_ra_lookahd_gain,
		(ParamFloat<px4::params::RA_LOOKAHD_MAX>) _param_ra_lookahd_max,
		(ParamFloat<px4::params::RA_LOOKAHD_MIN>) _param_ra_lookahd_min,
		(ParamFloat<px4::params::RA_ACC_RAD_MAX>) _param_ra_acc_rad_max,
		(ParamFloat<px4::params::RA_ACC_RAD_DEF>) _param_ra_acc_rad_def,
		(ParamFloat<px4::params::RA_ACC_RAD_GAIN>) _param_ra_acc_rad_gain,
		(ParamFloat<px4::params::RA_MISS_VEL_DEF>) _param_ra_miss_vel_def,
		(ParamFloat<px4::params::RA_MISS_VEL_MIN>) _param_ra_miss_vel_min,
		(ParamFloat<px4::params::RA_MISS_VEL_GAIN>) _param_ra_miss_vel_gain,
		(ParamFloat<px4::params::RA_SPEED_P>) _param_ra_p_speed,
		(ParamFloat<px4::params::RA_SPEED_I>) _param_ra_i_speed,
		(ParamFloat<px4::params::RA_MAX_SPEED>) _param_ra_max_speed,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad
	)
};
