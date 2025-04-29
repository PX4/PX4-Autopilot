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
#include <lib/slew_rate/SlewRate.hpp>
#include <math.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_rate_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>

/**
 * @brief Class for mecanum rate control.
 */
class MecanumRateControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for MecanumRateControl.
	 * @param parent The parent ModuleParams object.
	 */
	MecanumRateControl(ModuleParams *parent);
	~MecanumRateControl() = default;

	/**
	 * @brief Update rate controller.
	 */
	void updateRateControl();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:

	/**
	 * @brief Generate and publish roverRateSetpoint and roverThrottleSetpoint from manualControlSetpoint (Acro Mode).
	 */
	void generateRateAndThrottleSetpoint();

	/**
	 * @brief Generate and publish roverSteeringSetpoint from RoverRateSetpoint.
	 */
	void generateSteeringSetpoint();

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
	uORB::Subscription _rover_rate_setpoint_sub{ORB_ID(rover_rate_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	vehicle_control_mode_s  _vehicle_control_mode{};
	offboard_control_mode_s _offboard_control_mode{};
	rover_rate_setpoint_s   _rover_rate_setpoint{};

	// uORB publications
	uORB::Publication<rover_rate_setpoint_s>     _rover_rate_setpoint_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_steering_setpoint_s> _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_rate_status_s>       _rover_rate_status_pub{ORB_ID(rover_rate_status)};

	// Variables
	hrt_abstime _timestamp{0};
	float _max_yaw_rate{0.f};
	float _max_yaw_accel{0.f};
	float _max_yaw_decel{0.f};
	float _vehicle_yaw_rate{0.f};
	float _dt{0.f}; // Time since last update [s].
	bool _prev_param_check_passed{true};

	// Controllers
	PID _pid_yaw_rate;
	SlewRate<float> _adjusted_yaw_rate_setpoint{0.f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RM_WHEEL_TRACK>)   _param_rm_wheel_track,
		(ParamFloat<px4::params::RM_MAX_THR_YAW_R>) _param_rm_max_thr_yaw_r,
		(ParamFloat<px4::params::RO_YAW_RATE_LIM>)  _param_ro_yaw_rate_limit,
		(ParamFloat<px4::params::RO_YAW_RATE_TH>)   _param_ro_yaw_rate_th,
		(ParamFloat<px4::params::RO_YAW_RATE_P>)    _param_ro_yaw_rate_p,
		(ParamFloat<px4::params::RO_YAW_RATE_I>)    _param_ro_yaw_rate_i,
		(ParamFloat<px4::params::RO_YAW_ACCEL_LIM>) _param_ro_yaw_accel_limit,
		(ParamFloat<px4::params::RO_YAW_DECEL_LIM>) _param_ro_yaw_decel_limit
	)
};
