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
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/pure_pursuit/PurePursuit.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rover_mecanum_setpoint.h>

// Standard libraries
#include <lib/pid/PID.hpp>
#include <matrix/matrix/math.hpp>

// Local includes
#include "RoverMecanumGuidance/RoverMecanumGuidance.hpp"
#include "RoverMecanumControl/RoverMecanumControl.hpp"

// Constants
static constexpr float YAW_RATE_THRESHOLD =
	0.02f; // [rad/s] Threshold for the yaw rate measurement to avoid stuttering when the rover is standing still
static constexpr float SPEED_THRESHOLD =
	0.1f; // [m/s] Threshold for the speed measurement to cut off measurement noise when the rover is standing still
static constexpr float STICK_DEADZONE =
	0.3f; // [0, 1] Percentage of stick input range that will be interpreted as zero around the stick centered value

using namespace time_literals;

class RoverMecanum : public ModuleBase<RoverMecanum>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	RoverMecanum();
	~RoverMecanum() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

protected:
	void updateParams() override;

private:
	void Run() override;

	/**
	 * @brief Update uORB subscriptions.
	 */
	void updateSubscriptions();

	// uORB Subscriptions
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// uORB Publications
	uORB::Publication<rover_mecanum_setpoint_s> _rover_mecanum_setpoint_pub{ORB_ID(rover_mecanum_setpoint)};

	// Instances
	RoverMecanumGuidance _rover_mecanum_guidance{this};
	RoverMecanumControl _rover_mecanum_control{this};
	PurePursuit _posctl_pure_pursuit{this}; // Pure pursuit library

	// Variables
	matrix::Quatf _vehicle_attitude_quaternion{};
	float _vehicle_yaw_rate{0.f};
	float _vehicle_forward_speed{0.f};
	float _vehicle_lateral_speed{0.f};
	float _vehicle_yaw{0.f};
	float _max_yaw_rate{0.f};
	int _nav_state{0};
	bool _yaw_ctl{false}; // Indicates if the rover is doing yaw or yaw rate control in position mode
	float _desired_yaw{0.f}; // Yaw setpoint for position mode
	Vector2f _pos_ctl_start_position_ned{};
	Vector2f _pos_ctl_course_direction{};
	Vector2f _curr_pos_ned{};
	float _prev_throttle{0.f};
	float _prev_roll{0.f};
	bool _armed{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RM_MAX_SPEED>) _param_rm_max_speed,
		(ParamFloat<px4::params::RM_MAN_YAW_SCALE>) _param_rm_man_yaw_scale,
		(ParamFloat<px4::params::RM_MAX_YAW_RATE>) _param_rm_max_yaw_rate,
		(ParamFloat<px4::params::PP_LOOKAHD_MAX>) _param_pp_lookahd_max
	)
};
