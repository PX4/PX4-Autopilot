/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rover_differential_status.h>

// Standard libraries
#include <lib/pid/pid.h>
#include <matrix/matrix/math.hpp>

// Local includes
#include "RoverDifferentialGuidance/RoverDifferentialGuidance.hpp"

using namespace time_literals;

class RoverDifferential : public ModuleBase<RoverDifferential>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	RoverDifferential();
	~RoverDifferential() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/**
	 * @brief Computes motor commands for differential drive.
	 *
	 * @param forward_speed Linear velocity along the x-axis.
	 * @param speed_diff Speed difference between left and right wheels.
	 * @return matrix::Vector2f Motor velocities for the right and left motors.
	 */
	matrix::Vector2f computeMotorCommands(float forward_speed, const float speed_diff);

protected:
	void updateParams() override;

private:
	void Run() override;

	// uORB Subscriptions
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// uORB Publications
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<rover_differential_status_s> _rover_differential_status_pub{ORB_ID(rover_differential_status)};

	// Instances
	RoverDifferentialGuidance _rover_differential_guidance{this};

	// Variables
	float _vehicle_body_yaw_rate{0.f};
	float _vehicle_forward_speed{0.f};
	float _vehicle_yaw{0.f};
	float _max_yaw_rate{0.f};
	int _nav_state{0};
	matrix::Quatf _vehicle_attitude_quaternion{};
	hrt_abstime _timestamp{0};
	PID_t _pid_yaw_rate; // The PID controller for yaw rate
	RoverDifferentialGuidance::differential_setpoint _differential_setpoint;

	// Constants
	static constexpr float YAW_RATE_ERROR_THRESHOLD = 0.1f; // [rad/s] Error threshold for the closed loop yaw rate control

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RD_MAN_YAW_SCALE>) _param_rd_man_yaw_scale,
		(ParamFloat<px4::params::RD_WHEEL_TRACK>) _param_rd_wheel_track,
		(ParamFloat<px4::params::RD_YAW_RATE_P>) _param_rd_p_gain_yaw_rate,
		(ParamFloat<px4::params::RD_YAW_RATE_I>) _param_rd_i_gain_yaw_rate,
		(ParamFloat<px4::params::RD_MAX_SPEED>) _param_rd_max_speed,
		(ParamFloat<px4::params::RD_MAX_YAW_RATE>) _param_rd_max_yaw_rate,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};
