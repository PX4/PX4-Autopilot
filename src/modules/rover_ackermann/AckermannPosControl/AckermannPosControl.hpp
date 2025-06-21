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

// Library includes
#include <matrix/matrix/math.hpp>
#include <lib/pure_pursuit/PurePursuit.hpp>
#include <math.h>
#include <lib/geo/geo.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/rover_velocity_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/pure_pursuit_status.h>

using namespace matrix;

/**
 * @brief Class for ackermann position control.
 */
class AckermannPosControl : public ModuleParams
{
public:
	/**
	 * @brief Constructor for AckermannPosControl.
	 * @param parent The parent ModuleParams object.
	 */
	AckermannPosControl(ModuleParams *parent);
	~AckermannPosControl() = default;

	/**
	 * @brief Generate and publish roverVelocitySetpoint from roverPositionSetpoint.
	 */
	void updatePosControl();

	/**
	 * @brief Check if the necessary parameters are set.
	 * @return True if all checks pass.
	 */
	bool runSanityChecks();

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

	// uORB subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _rover_position_setpoint_sub{ORB_ID(rover_position_setpoint)};
	uORB::Subscription _position_controller_status_sub{ORB_ID(position_controller_status)};
	rover_position_setpoint_s _rover_position_setpoint{};

	// uORB publications
	uORB::Publication<rover_velocity_setpoint_s> _rover_velocity_setpoint_pub{ORB_ID(rover_velocity_setpoint)};
	uORB::Publication<pure_pursuit_status_s>     _pure_pursuit_status_pub{ORB_ID(pure_pursuit_status)};

	// Variables
	Quatf _vehicle_attitude_quaternion{};
	Vector2f _curr_pos_ned{};
	Vector2f _start_ned{};
	float _vehicle_yaw{0.f};
	float _max_yaw_rate{0.f};
	float _acceptance_radius{0.f}; // Acceptance radius for the waypoint.

	// Class Instances
	MapProjection _global_ned_proj_ref{}; // Transform global to NED coordinates

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_DECEL_LIM>)     _param_ro_decel_limit,
		(ParamFloat<px4::params::RO_JERK_LIM>)      _param_ro_jerk_limit,
		(ParamFloat<px4::params::RO_SPEED_LIM>)     _param_ro_speed_limit,
		(ParamFloat<px4::params::PP_LOOKAHD_GAIN>)  _param_pp_lookahd_gain,
		(ParamFloat<px4::params::PP_LOOKAHD_MAX>)   _param_pp_lookahd_max,
		(ParamFloat<px4::params::PP_LOOKAHD_MIN>)   _param_pp_lookahd_min,
		(ParamFloat<px4::params::RO_YAW_RATE_LIM>)  _param_ro_yaw_rate_limit
	)
};
