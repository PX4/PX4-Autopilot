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
#include <math.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_velocity_status.h>
#include <uORB/topics/rover_velocity_setpoint.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;

/**
 * @brief Class for mecanum velocity control.
 */
class MecanumVelControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for MecanumVelControl.
	 * @param parent The parent ModuleParams object.
	 */
	MecanumVelControl(ModuleParams *parent);
	~MecanumVelControl() = default;

	/**
	 * @brief Update velocity controller.
	 */
	void updateVelControl();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Update uORB subscriptions used in velocity controller.
	 */
	void updateSubscriptions();

	/**
	 * @brief Generate and publish roverVelocitySetpoint from velocity of trajectorySetpoint.
	 */
	void generateVelocitySetpoint();

	/**
	 * @brief Generate and publish roverAttitudeSetpoint and roverThrottleSetpoint
	 *        from roverVelocitySetpoint.
	 */
	void generateAttitudeAndThrottleSetpoint();

	/**
	 * @brief Check if the necessary parameters are set.
	 * @return True if all checks pass.
	 */
	bool runSanityChecks();

	// uORB subscriptions
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _rover_velocity_setpoint_sub{ORB_ID(rover_velocity_setpoint)};
	uORB::Subscription _rover_attitude_setpoint_sub{ORB_ID(rover_attitude_setpoint)};
	uORB::Subscription _rover_steering_setpoint_sub{ORB_ID(rover_steering_setpoint)};
	vehicle_control_mode_s _vehicle_control_mode{};
	offboard_control_mode_s _offboard_control_mode{};
	rover_steering_setpoint_s _rover_steering_setpoint{};

	// uORB publications
	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_velocity_status_s> _rover_velocity_status_pub{ORB_ID(rover_velocity_status)};
	uORB::Publication<rover_velocity_setpoint_s> _rover_velocity_setpoint_pub{ORB_ID(rover_velocity_setpoint)};
	rover_velocity_setpoint_s _rover_velocity_setpoint{};

	// Variables
	hrt_abstime _timestamp{0};
	hrt_abstime _last_attitude_setpoint_update{0};
	Quatf _vehicle_attitude_quaternion{};
	float _vehicle_speed_body_x{0.f};
	float _vehicle_speed_body_y{0.f};
	float _vehicle_yaw{0.f};
	float _dt{0.f};
	bool _prev_param_check_passed{false};

	// Controllers
	PID _pid_speed_x;
	PID _pid_speed_y;
	SlewRate<float> _speed_x_setpoint;
	SlewRate<float> _speed_y_setpoint;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_MAX_THR_SPEED>) _param_ro_max_thr_speed,
		(ParamFloat<px4::params::RO_SPEED_P>) 	    _param_ro_speed_p,
		(ParamFloat<px4::params::RO_SPEED_I>)       _param_ro_speed_i,
		(ParamFloat<px4::params::RO_ACCEL_LIM>)     _param_ro_accel_limit,
		(ParamFloat<px4::params::RO_DECEL_LIM>)     _param_ro_decel_limit,
		(ParamFloat<px4::params::RO_JERK_LIM>)      _param_ro_jerk_limit,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit,
		(ParamFloat<px4::params::RO_SPEED_TH>)      _param_ro_speed_th

	)
};
