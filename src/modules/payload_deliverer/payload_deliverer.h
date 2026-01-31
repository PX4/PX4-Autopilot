/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/**
 * @file payload_deliverer.h
 * @author Junwoo Hwang (junwoo091400@gmail.com)
 */
#include "gripper.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

extern "C" __EXPORT int payload_deliverer_main(int argc, char *argv[]);

// If cached gripper action is set to this value, it means we aren't running any vehicle command
static constexpr int8_t GRIPPER_ACTION_NONE = -1;

/**
 * @brief Payload Deliverer Module
 *
 * Activates a winch / gripper when the DO_WINCH or DO_GRIPPER vehicle command is received,
 * after which the vehicle_command_ack command gets sent upon successful confirmation
 * of the payload deployment.
 *
 * The module runs based on having a subscription callback mechanism to the 'vehicle_command' topic
 * as it only needs to run when we receive the relevant vehicle command.
 *
 * This module communicates with the Navigator which handles publishing such vehicle commands.
 */
class PayloadDeliverer : public ModuleBase<PayloadDeliverer>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PayloadDeliverer();

	/** @see ModuleBase **/
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase. Override "status" output when invoked via Commandline, to give detailed status **/
	int print_status() override;

	// Initializes the module
	bool init();

	// Gripper test commands provoked by custom_command() via CLI interface
	void gripper_test();
	void gripper_open();
	void gripper_close();

private:
	/**
	 * @brief Main Run function that runs when subscription callback is triggered
	 */
	void Run() override;

	/**
	 * @brief Called to update the parameters and initialize gripper instance if necessary
	 */
	void parameter_update();

	/**
	 * @brief Initialize or deinitialize gripper instance based on parameter settings
	 *
	 * Depending on the state of `_gripper` instance, this function will
	 * either try to initialize or de-initialize the gripper appropriately.
	 */
	void configure_gripper();

	/**
	 * @brief Update gipper instance's state and send vehicle command ack
	 *
	 * This updates the gripper instance to check if the gripper has reached the desired state.
	 * And if so, it sends a vehicle command ack to the navigator.
	 *
	 * If the gripper instance isn't valid (i.e. not initialized), it doesn't do anything
	 */
	void gripper_update(const hrt_abstime &now);

	/**
	 * @brief Commands the payload delivery mechanism based on the received vehicle command
	 *
	 * Also handle vehicle command acknowledgement once the delivery is confirmed from the mechanism
	 */
	void handle_vehicle_command(const hrt_abstime &now, const vehicle_command_s *vehicle_command = nullptr);

	/**
	 * @brief Send DO_GRIPPER vehicle command with specified gripper action
	 *
	 * @param gripper_command GRIPPER_ACTION_GRAB or GRIPPER_ACTION_RELEASE
	 */
	bool send_gripper_vehicle_command(const int32_t gripper_action);

	/**
	 * @brief Send ack response to DO_GRIPPER vehicle command with specified parameters
	 *
	 * For the case of VEHICLE_CMD_RESULT_IN_PROGRESS, progress percentage parameter will be filled out
	 */
	bool send_gripper_vehicle_command_ack(const hrt_abstime now, const uint8_t command_result, const uint8_t target_system,
					      const uint8_t target_component);

	Gripper _gripper;

	// Cached values of the currently running vehicle command for the gripper action
	// used for conflicting vehicle commands & successful vehicle command acknowledgements
	int8_t _cur_vcmd_gripper_action{GRIPPER_ACTION_NONE};
	uint8_t _cur_vcmd_target_system{0};
	uint8_t _cur_vcmd_target_component{0};

	// Subscription
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates

	// Publications
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PD_GRIPPER_TO>)	_param_gripper_timeout_s,
		(ParamInt<px4::params::PD_GRIPPER_TYPE>)	_param_gripper_type
	)
};
