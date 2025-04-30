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
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// Library includes
#include <math.h>
#include <lib/rover_control/RoverControl.hpp>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rover_velocity_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_controller_status.h>

// Local includes
#include "AckermannActControl/AckermannActControl.hpp"
#include "AckermannRateControl/AckermannRateControl.hpp"
#include "AckermannAttControl/AckermannAttControl.hpp"
#include "AckermannVelControl/AckermannVelControl.hpp"
#include "AckermannPosControl/AckermannPosControl.hpp"

class RoverAckermann : public ModuleBase<RoverAckermann>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	/**
	 * @brief Constructor for RoverAckermann
	 */
	RoverAckermann();
	~RoverAckermann() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	void Run() override;

	/**
	 * @brief Handle manual control
	 */
	void manualControl();

	/**
	 * @brief Publish roverThrottleSetpoint and roverSteeringSetpoint from manualControlSetpoint.
	 */
	void manualManualMode();

	/**
	 * @brief Generate and publish roverThrottleSetpoint and RoverRateSetpoint from manualControlSetpoint.
	 */
	void manualAcroMode();

	/**
	 * @brief Generate and publish roverThrottleSetpoint and RoverAttitudeSetpoint from manualControlSetpoint.
	 */
	void manualStabMode();

	/**
	 * @brief Generate and publish roverVelocitySetpoint from manualControlSetpoint.
	 */
	void manualPositionMode();

	/**
	 * @brief Generate and publish roverVelocitySetpoint from positionSetpointTriplet.
	 */
	void autoPositionMode();

	/**
	 * @brief Update global/NED waypoint coordinates and acceptance radius.
	 */
	void autoUpdateWaypointsAndAcceptanceRadius();

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
	float autoUpdateAcceptanceRadius(float waypoint_transition_angle, float default_acceptance_radius,
					 float acceptance_radius_gain, float acceptance_radius_max, float wheel_base, float max_steer_angle);

	/**
	 * @brief Calculate the speed at which the rover should arrive at the current waypoint based on the upcoming corner.
	 * @param cruising_speed Cruising speed [m/s].
	 * @param miss_speed_min Minimum speed setpoint [m/s].
	 * @param acc_rad Acceptance radius of the current waypoint [m].
	 * @param curr_wp_type Type of the current waypoint.
	 * @param waypoint_transition_angle Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	 * @param max_yaw_rate Maximum yaw rate setpoint [rad/s]
	 * @return Speed setpoint [m/s].
	 */
	float autoArrivalSpeed(float cruising_speed, float miss_speed_min, float acc_rad, int curr_wp_type,
			       float waypoint_transition_angle, float max_yaw_rate);

	/**
	 * @brief Calculate the cruising speed setpoint. During cornering the speed is restricted based on the radius of the corner.
	 * @param cruising_speed Cruising speed [m/s].
	 * @param miss_speed_min Minimum speed setpoint [m/s].
	 * @param distance_to_prev_wp Distance to the previous waypoint [m].
	 * @param distance_to_curr_wp Distance to the current waypoint [m].
	 * @param acc_rad Acceptance radius of the current waypoint [m].
	 * @param prev_acc_rad Acceptance radius of the previous waypoint [m].
	 * @param waypoint_transition_angle Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	 * @param prev_waypoint_transition_angle Previous angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	 *  @param max_yaw_rate Maximum yaw rate setpoint [rad/s]
	 * @return Speed setpoint [m/s].
	 */
	float autoCruisingSpeed(float cruising_speed, float miss_speed_min, float distance_to_prev_wp,
				float distance_to_curr_wp, float acc_rad, float prev_acc_rad, float waypoint_transition_angle,
				float prev_waypoint_transition_angle, float max_yaw_rate);

	/**
	 * @brief Translate trajectorySetpoint to roverSetpoints and publish them
	 */
	void offboardControl();

	/**
	 * @brief Update the controllers
	 */
	void updateControllers();

	/**
	 * @brief Check proper parameter setup for the controllers
	 *
	 * Modifies:
	 *
	 *   - _sanity_checks_passed: true if checks for all active controllers pass
	 */
	void runSanityChecks();

	/**
	 * @brief Reset controllers and manual mode variables.
	 */
	void reset();

	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	vehicle_control_mode_s _vehicle_control_mode{};

	// uORB publications
	uORB::Publication<rover_velocity_setpoint_s>    _rover_velocity_setpoint_pub{ORB_ID(rover_velocity_setpoint)};
	uORB::Publication<rover_position_setpoint_s>    _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};
	uORB::Publication<rover_steering_setpoint_s>    _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_throttle_setpoint_s>    _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s>    _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_rate_setpoint_s>        _rover_rate_setpoint_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<position_controller_status_s>	_position_controller_status_pub{ORB_ID(position_controller_status)};

	// Class instances
	AckermannActControl  _ackermann_act_control{this};
	AckermannRateControl _ackermann_rate_control{this};
	AckermannAttControl  _ackermann_att_control{this};
	AckermannVelControl  _ackermann_vel_control{this};
	AckermannPosControl  _ackermann_pos_control{this};

	// Variables
	MapProjection _global_ned_proj_ref{}; // Transform global to NED coordinates
	Quatf _vehicle_attitude_quaternion{};
	float _max_yaw_rate{NAN};
	float _vehicle_yaw{NAN};
	float _min_speed{0.f}; // Speed at which the maximum yaw rate limit is enforced given the maximum steer angle and wheel base.
	int _nav_state{0}; // Navigation state of the vehicle
	bool _sanity_checks_passed{true}; // True if checks for all active controllers pass
	bool _was_armed{false}; // True if the vehicle was armed before the last reset

	// Auto Mode Variables
	Vector2f _curr_wp_ned{};
	Vector2f _prev_wp_ned{};
	Vector2f _next_wp_ned{};
	float _acceptance_radius{0.5f};
	float _prev_acceptance_radius{0.5f};
	float _cruising_speed{0.f};
	float _waypoint_transition_angle{0.f}; // Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	float _prev_waypoint_transition_angle{0.f}; // Previous Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	int _curr_wp_type{position_setpoint_s::SETPOINT_TYPE_IDLE};

	// Manual Mode Variables
	Vector2f _pos_ctl_course_direction{NAN, NAN};
	Vector2f _pos_ctl_start_position_ned{NAN, NAN};
	Vector2f _curr_pos_ned{NAN, NAN};
	float _stab_yaw_setpoint{NAN};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_YAW_RATE_LIM>)  _param_ro_yaw_rate_limit,
		(ParamFloat<px4::params::RO_YAW_P>)         _param_ro_yaw_p,
		(ParamFloat<px4::params::RO_YAW_STICK_DZ>)  _param_ro_yaw_stick_dz,
		(ParamFloat<px4::params::PP_LOOKAHD_MAX>)   _param_pp_lookahd_max,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit,
		(ParamFloat<px4::params::RA_WHEEL_BASE>)    _param_ra_wheel_base,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>)   _param_ra_max_str_ang,
		(ParamFloat<px4::params::NAV_ACC_RAD>)      _param_nav_acc_rad,
		(ParamFloat<px4::params::RA_ACC_RAD_MAX>)   _param_ra_acc_rad_max,
		(ParamFloat<px4::params::RA_ACC_RAD_GAIN>)  _param_ra_acc_rad_gain
	)
};
