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
#include <lib/slew_rate/SlewRate.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/rover_ackermann_status.h>
#include <uORB/topics/vehicle_local_position.h>


// Standard library includes
#include <math.h>

// Local includes
#include "RoverAckermannGuidance/RoverAckermannGuidance.hpp"
using motor_setpoint_struct = RoverAckermannGuidance::motor_setpoint;

using namespace time_literals;

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
	void updateParams() override;

private:
	void Run() override;

	/**
	 * @brief Update uORB subscriptions.
	 */
	void updateSubscriptions();

	/**
	 * @brief Apply slew rates to motor setpoints.
	 * @param motor_setpoint Normalized steering and throttle setpoints.
	 * @param dt Time since last update [s].
	 * @return Motor setpoint with applied slew rates.
	 */
	motor_setpoint_struct applySlewRates(motor_setpoint_struct motor_setpoint, float dt);

	/**
	 * @brief Publish motor setpoints to ActuatorMotors/ActuatorServos and logging values to RoverAckermannStatus.
	 * @param motor_setpoint_with_slew_rate Normalized motor_setpoint with applied slew rates.
	 */
	void publishMotorSetpoints(motor_setpoint_struct motor_setpoint_with_slew_rates);

	// uORB subscriptions
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};

	// uORB publications
	uORB::PublicationMulti<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<rover_ackermann_status_s> _rover_ackermann_status_pub{ORB_ID(rover_ackermann_status)};

	// Class instances
	RoverAckermannGuidance _ackermann_guidance{this};

	// Variables
	int _nav_state{0};
	motor_setpoint_struct _motor_setpoint;
	hrt_abstime _timestamp{0};
	float _actual_speed{0.f};
	SlewRate<float> _steering_with_rate_limit{0.f};
	SlewRate<float> _throttle_with_accel_limit{0.f};
	bool _armed{false};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev,
		(ParamFloat<px4::params::RA_MAX_STR_ANG>) _param_ra_max_steer_angle,
		(ParamFloat<px4::params::RA_MAX_SPEED>) _param_ra_max_speed,
		(ParamFloat<px4::params::RA_MAX_ACCEL>) _param_ra_max_accel,
		(ParamFloat<px4::params::RA_MAX_STR_RATE>) _param_ra_max_steering_rate
	)
};
