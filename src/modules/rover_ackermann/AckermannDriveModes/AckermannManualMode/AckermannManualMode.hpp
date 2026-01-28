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
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/rover_attitude_setpoint.h>
#include <uORB/topics/rover_speed_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>

/**
 * @brief Class for ackermann manual mode.
 */
class AckermannManualMode : public ModuleParams
{
public:
	/**
	 * @brief Constructor for AckermannManualMode.
	 * @param parent The parent ModuleParams object.
	 */
	AckermannManualMode(ModuleParams *parent);
	~AckermannManualMode() = default;

	/**
	 * @brief Publish roverThrottleSetpoint and roverSteeringSetpoint from manualControlSetpoint.
	 */
	void manual();

	/**
	 * @brief Generate and publish roverThrottleSetpoint and RoverRateSetpoint from manualControlSetpoint.
	 */
	void acro();

	/**
	 * @brief Generate and publish roverThrottleSetpoint and RoverAttitudeSetpoint from manualControlSetpoint.
	 */
	void stab();

	/**
	 * @brief Generate and publish roverSpeedSetpoint/roverRateSetpoint or roverPositionSetpoint from manualControlSetpoint.
	 */
	void position();

	/**
	 * @brief Reset manual mode variables.
	 */
	void reset();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// uORB publications
	uORB::Publication<rover_throttle_setpoint_s> _rover_throttle_setpoint_pub{ORB_ID(rover_throttle_setpoint)};
	uORB::Publication<rover_steering_setpoint_s> _rover_steering_setpoint_pub{ORB_ID(rover_steering_setpoint)};
	uORB::Publication<rover_rate_setpoint_s>     _rover_rate_setpoint_pub{ORB_ID(rover_rate_setpoint)};
	uORB::Publication<rover_attitude_setpoint_s> _rover_attitude_setpoint_pub{ORB_ID(rover_attitude_setpoint)};
	uORB::Publication<rover_speed_setpoint_s>    _rover_speed_setpoint_pub{ORB_ID(rover_speed_setpoint)};
	uORB::Publication<rover_position_setpoint_s> _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};

	// Variables
	MapProjection _global_ned_proj_ref{}; // Transform global to NED coordinates
	Quatf _vehicle_attitude_quaternion{};
	Vector2f _pos_ctl_course_direction{NAN, NAN};
	Vector2f _pos_ctl_start_position_ned{NAN, NAN};
	Vector2f _curr_pos_ned{NAN, NAN};
	float _stab_yaw_setpoint{NAN};
	float _vehicle_yaw{NAN};
	float _max_yaw_rate{NAN};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_YAW_RATE_LIM>)  _param_ro_yaw_rate_limit,
		(ParamFloat<px4::params::RO_YAW_P>)         _param_ro_yaw_p,
		(ParamFloat<px4::params::RO_YAW_STICK_DZ>)  _param_ro_yaw_stick_dz,
		(ParamFloat<px4::params::RO_YAW_EXPO>)      _param_ro_yaw_expo,
		(ParamFloat<px4::params::RO_YAW_SUPEXPO>)   _param_ro_yaw_supexpo,
		(ParamFloat<px4::params::PP_LOOKAHD_MAX>)   _param_pp_lookahd_max,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit
	)
};
