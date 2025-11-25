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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

// Local includes
#include "AckermannActControl/AckermannActControl.hpp"
#include "AckermannRateControl/AckermannRateControl.hpp"
#include "AckermannAttControl/AckermannAttControl.hpp"
#include "AckermannSpeedControl/AckermannSpeedControl.hpp"
#include "AckermannPosControl/AckermannPosControl.hpp"
#include "AckermannDriveModes/AckermannAutoMode/AckermannAutoMode.hpp"
#include "AckermannDriveModes/AckermannManualMode/AckermannManualMode.hpp"
#include "AckermannDriveModes/AckermannOffboardMode/AckermannOffboardMode.hpp"

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
	 * @brief Generate rover setpoints from supported PX4 internal modes
	 */
	void generateSetpoints();

	/**
	 * @brief Update the active controllers.
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
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	vehicle_control_mode_s _vehicle_control_mode{};

	// Class instances
	AckermannActControl  _ackermann_act_control{this};
	AckermannRateControl _ackermann_rate_control{this};
	AckermannAttControl  _ackermann_att_control{this};
	AckermannSpeedControl  _ackermann_speed_control{this};
	AckermannPosControl  _ackermann_pos_control{this};
	AckermannAutoMode	     _auto_mode{this};
	AckermannManualMode 	     _manual_mode{this};
	AckermannOffboardMode 	     _offboard_mode{this};

	// Variables
	bool _sanity_checks_passed{true}; // True if checks for all active controllers pass
	bool _was_armed{false}; // True if the vehicle was armed before the last reset
};
