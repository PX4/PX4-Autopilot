/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

// Libraries
#include <lib/rover_control/RoverControl.hpp>
#include <math.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/rover_position_setpoint.h>

/**
 * @brief Class for ackermann auto mode.
 */
class AckermannAutoMode : public ModuleParams
{
public:
	/**
	 * @brief Constructor for auto mode.
	 * @param parent The parent ModuleParams object.
	 */
	AckermannAutoMode(ModuleParams *parent);
	~AckermannAutoMode() = default;

	/**
	 * @brief Generate and publish roverPositionSetpoint from positionSetpointTriplet.
	 */
	void autoControl();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Update global/NED waypoint coordinates and acceptance radius.
	 */
	void updateWaypointsAndAcceptanceRadius();

	/**
	 * @brief Publish the acceptance radius for current waypoint based on the angle between a line segment
	 * from the previous to the current waypoint/current to the next waypoint and maximum steer angle of the vehicle.
	 * @param waypoint_transition_angle Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	 * @param default_acceptance_radius Default acceptance radius for waypoints [m].
	 * @param acceptance_radius_gain Tuning parameter that scales the geometric optimal acceptance radius for the corner cutting [-].
	 * @param acceptance_radius_max Maximum value for the acceptance radius [m].
	 * @param wheel_base Rover wheelbase [m].
	 * @param max_steer_angle Rover maximum steer angle [rad].
	 * @return Updated acceptance radius [m].
	 */
	float updateAcceptanceRadius(float waypoint_transition_angle, float default_acceptance_radius,
				     float acceptance_radius_gain, float acceptance_radius_max, float wheel_base, float max_steer_angle);

	/**
	 * @brief Calculate the speed at which the rover should arrive at the current waypoint based on the upcoming corner.
	 * @param cruising_speed Cruising speed [m/s].
	 * @param min_speed Minimum speed setpoint [m/s].
	 * @param acc_rad Acceptance radius of the current waypoint [m].
	 * @param curr_wp_type Type of the current waypoint.
	 * @param waypoint_transition_angle Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	 * @param max_yaw_rate Maximum yaw rate setpoint [rad/s]
	 * @return Speed setpoint [m/s].
	 */
	float arrivalSpeed(float cruising_speed, float min_speed, float acc_rad, int curr_wp_type,
			   float waypoint_transition_angle, float max_yaw_rate);

	// uORB subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};

	// uORB publications
	uORB::Publication<rover_position_setpoint_s>    _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};
	uORB::Publication<position_controller_status_s>	_position_controller_status_pub{ORB_ID(position_controller_status)};

	// Variables
	MapProjection _global_ned_proj_ref{}; // Transform global to NED coordinates
	Vector2f _curr_wp_ned{NAN, NAN};
	Vector2f _prev_wp_ned{NAN, NAN};
	Vector2f _next_wp_ned{NAN, NAN};
	Vector2f _curr_pos_ned{NAN, NAN};
	float _acceptance_radius{0.5f};
	float _cruising_speed{0.f};
	float _waypoint_transition_angle{0.f}; // Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	float _max_yaw_rate{NAN};
	float _min_speed{NAN}; // Speed at which the maximum yaw rate limit is enforced given the maximum steer angle and wheel base.
	int _curr_wp_type{position_setpoint_s::SETPOINT_TYPE_IDLE};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_YAW_RATE_LIM>)  _param_ro_yaw_rate_limit,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit,
		(ParamFloat<px4::params::RA_WHEEL_BASE>)    _param_ra_wheel_base,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>)   _param_ra_max_str_ang,
		(ParamFloat<px4::params::NAV_ACC_RAD>)      _param_nav_acc_rad,
		(ParamFloat<px4::params::RA_ACC_RAD_MAX>)   _param_ra_acc_rad_max,
		(ParamFloat<px4::params::RA_ACC_RAD_GAIN>)  _param_ra_acc_rad_gain,
		(ParamFloat<px4::params::RO_SPEED_RED>)     _param_ro_speed_red,
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed
	)
};
