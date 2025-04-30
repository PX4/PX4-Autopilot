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

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rover_velocity_setpoint.h>
#include <uORB/topics/rover_position_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>

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
	 * @brief Handle offboard control
	 * @return True if actuator control needs to be handled by the module
	 */
	bool offboardControl();

	/**
	 * @brief Update the controllers
	 */
	void updateControllers();

	/**
	 * @brief Run sanity checks for the controllers
	 */
	void runSanityChecks();

	// uORB subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _offboard_control_mode_sub{ORB_ID(offboard_control_mode)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	vehicle_control_mode_s _vehicle_control_mode{};

	// uORB publications
	uORB::Publication<rover_velocity_setpoint_s> _rover_velocity_setpoint_pub{ORB_ID(rover_velocity_setpoint)};
	uORB::Publication<rover_position_setpoint_s> _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};

	// Class instances
	AckermannActControl  _ackermann_act_control{this};
	AckermannRateControl _ackermann_rate_control{this};
	AckermannAttControl  _ackermann_att_control{this};
	AckermannVelControl  _ackermann_vel_control{this};
	AckermannPosControl  _ackermann_pos_control{this};

	// Variables
	hrt_abstime _timestamp{0};
	float _dt{0.f};
	int _nav_state{0};
	bool _sanity_checks_passed{true};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev
	)
};
