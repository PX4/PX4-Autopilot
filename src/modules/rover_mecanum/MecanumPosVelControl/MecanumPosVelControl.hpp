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
#include <px4_platform_common/events.h>

// Libraries
#include <lib/rover_control/RoverControl.hpp>
#include <lib/pid/PID.hpp>
#include <matrix/matrix/math.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <lib/pure_pursuit/PurePursuit.hpp>
#include <lib/geo/geo.h>
#include <math.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_velocity_status.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/pure_pursuit_status.h>

using namespace matrix;

/**
 * @brief Class for mecanum position/velocity control.
 */
class MecanumPosVelControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for MecanumPosVelControl.
	 * @param parent The parent ModuleParams object.
	 */
	MecanumPosVelControl(ModuleParams *parent);
	~MecanumPosVelControl() = default;

	/**
	 * @brief Update position controller.
	 */
	void updatePosControl();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Update uORB subscriptions used in position controller.
	 */
	void updateSubscriptions();

	/**
	 * @brief Generate and publish roverAttitudeSetpoint from manualControlSetpoint (Position Mode) or
	 * 	  trajcetorySetpoint (Offboard position or velocity control) or
	 * 	  positionSetpointTriplet (Auto Mode).
	 */
	void generateAttitudeSetpoint();

	/**
	 * @brief Generate and publish roverThrottleSetpoint and roverAttitudeSetpoint or roverRateSetpoint from manualControlSetpoint.
	 */
	void manualPositionMode();

	/**
	 * @brief Generate and publish roverAttitudeSetpoint from position of trajectorySetpoint.
	 */
	void offboardPositionMode();

	/**
	 * @brief Generate and publish roverAttitudeSetpoint from velocity of trajectorySetpoint.
	 */
	void offboardVelocityMode();

	/**
	 * @brief Generate and publish roverThrottleSetpoint and roverAttitudeSetpoint from positionSetpointTriplet.
	 */
	void autoPositionMode();

	/**
	 * @brief Calculate the velocity magnitude setpoint. During waypoint transition the speed is restricted to
	 * Maximum_speed * (1 - normalized_transition_angle * RM_MISS_VEL_GAIN).
	 * On straight lines it is based on a speed trajectory such that the rover will arrive at the next waypoint transition
	 * with the desired waypoiny transition speed under consideration of the maximum deceleration and jerk.
	 * @param auto_speed Default auto speed [m/s].
	 * @param distance_to_curr_wp Distance to the current waypoint [m].
	 * @param max_decel Maximum allowed deceleration [m/s^2].
	 * @param max_jerk Maximum allowed jerk [m/s^3].
	 * @param waypoint_transition_angle Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	 * @param max_speed Maximum velocity magnitude setpoint [m/s]
	 * @param miss_spd_gain Tuning parameter for the speed reduction during waypoint transition.
	 * @param curr_wp_type Type of the current waypoint.
	 * @return Velocity magnitude setpoint [m/s].
	 */
	float calcVelocityMagnitude(float auto_speed, float distance_to_curr_wp, float max_decel, float max_jerk,
				    float waypoint_transition_angle, float max_speed, float miss_spd_gain, int curr_wp_type);

	/**
	 * @brief Check if the necessary parameters are set.
	 * @return True if all checks pass.
	 */
	bool runSanityChecks();

	// uORB subscriptions
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _rover_steering_setpoint_sub{ORB_ID(rover_steering_setpoint)};
	vehicle_control_mode_s    _vehicle_control_mode{};
	offboard_control_mode_s   _offboard_control_mode{};
	rover_steering_setpoint_s _rover_steering_setpoint{};

	// uORB publications
	uORB::Publication<rover_rate_setpoint_s>        _rover_rate_setpoint_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<rover_throttle_setpoint_s>    _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s>    _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_velocity_status_s>      _rover_velocity_status_pub{ORB_ID(rover_velocity_status)};
	uORB::Publication<pure_pursuit_status_s>	_pure_pursuit_status_pub{ORB_ID(pure_pursuit_status)};

	// Variables
	hrt_abstime _timestamp{0};
	Quatf _vehicle_attitude_quaternion{};
	Vector2f _curr_pos_ned{};
	Vector2f _pos_ctl_course_direction{};
	Vector2f _pos_ctl_start_position_ned{};
	float _vehicle_speed_body_x{0.f};
	float _vehicle_speed_body_y{0.f};
	float _vehicle_yaw{0.f};
	float _max_yaw_rate{0.f};
	float _speed_body_x_setpoint{0.f};
	float _speed_body_y_setpoint{0.f};
	float _pos_ctl_yaw_setpoint{0.f}; // Yaw setpoint for manual position mode, NAN if yaw rate is manually controlled [rad]
	float _dt{0.f};
	float _auto_speed{0.f};
	float _auto_yaw{0.f};
	int _curr_wp_type{position_setpoint_s::SETPOINT_TYPE_IDLE};
	bool _prev_param_check_passed{true};

	// Waypoint variables
	Vector2f _curr_wp_ned{};
	Vector2f _prev_wp_ned{};
	Vector2f _next_wp_ned{};
	float _waypoint_transition_angle{0.f}; // Angle between the prevWP-currWP and currWP-nextWP line segments [rad]

	// Controllers
	PID _pid_speed_x;
	PID _pid_speed_y;
	SlewRate<float> _speed_x_setpoint;
	SlewRate<float> _speed_y_setpoint;

	// Class Instances
	MapProjection _global_ned_proj_ref{}; // Transform global to NED coordinates

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RM_MISS_SPD_GAIN>) _param_rm_miss_spd_gain,
		(ParamFloat<px4::params::RM_COURSE_CTL_TH>) _param_rm_course_ctl_th,
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed,
		(ParamFloat<px4::params::RO_SPEED_P>) 	    _param_ro_speed_p,
		(ParamFloat<px4::params::RO_SPEED_I>)       _param_ro_speed_i,
		(ParamFloat<px4::params::RO_YAW_STICK_DZ>)  _param_ro_yaw_stick_dz,
		(ParamFloat<px4::params::RO_ACCEL_LIM>)     _param_ro_accel_limit,
		(ParamFloat<px4::params::RO_DECEL_LIM>)     _param_ro_decel_limit,
		(ParamFloat<px4::params::RO_JERK_LIM>)      _param_ro_jerk_limit,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit,
		(ParamFloat<px4::params::RO_SPEED_TH>)      _param_ro_speed_th,
		(ParamFloat<px4::params::PP_LOOKAHD_GAIN>)  _param_pp_lookahd_gain,
		(ParamFloat<px4::params::PP_LOOKAHD_MAX>)   _param_pp_lookahd_max,
		(ParamFloat<px4::params::PP_LOOKAHD_MIN>)   _param_pp_lookahd_min,
		(ParamFloat<px4::params::RO_YAW_RATE_LIM>)  _param_ro_yaw_rate_limit,
		(ParamFloat<px4::params::NAV_ACC_RAD>)      _param_nav_acc_rad

	)
};
