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
#include "payload_deliverer.h"

PayloadDeliverer::PayloadDeliverer()
	: ModuleParams(nullptr),
	  ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool PayloadDeliverer::init()
{
	ScheduleOnInterval(100_ms);

	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	configure_gripper();
	return true;
}

void PayloadDeliverer::configure_gripper()
{
	// If gripper instance is invalid, and user enabled the gripper, try initializing it
	if (!_gripper.is_valid()) {
		GripperConfig config{};
		config.type = (GripperConfig::GripperType)_param_gripper_type.get();
		config.sensor = GripperConfig::GripperSensorType::NONE; // Feedback sensor isn't supported for now
		config.timeout_us = hrt_abstime(_param_gripper_timeout_s.get() * 1000000ULL);
		_gripper.init(config);

		if (!_gripper.is_valid()) {
			PX4_DEBUG("Gripper object initialization failed!");
			return;

		} else {
			// Command the gripper to grab position by default
			if (!_gripper.grabbed() && !_gripper.grabbing()) {
				PX4_DEBUG("Gripper intialize: putting to grab position!");
				send_gripper_vehicle_command(vehicle_command_s::GRIPPER_ACTION_GRAB);
			}

			return;
		}
	}

	// TODO: Support changing gripper sensor type / gripper type configuration
}

void PayloadDeliverer::parameter_update()
{
	updateParams();
	configure_gripper();
}

void PayloadDeliverer::Run()
{
	const hrt_abstime now = hrt_absolute_time();
	vehicle_command_s vcmd{};

	if (should_exit()) {
		ScheduleClear();
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update_dummy;
		_parameter_update_sub.copy(&param_update_dummy);
		parameter_update();
	}

	gripper_update(now);

	if (_vehicle_command_sub.update(&vcmd)) {
		handle_vehicle_command(now, &vcmd);

	} else {
		handle_vehicle_command(now);

	}
}

void PayloadDeliverer::gripper_update(const hrt_abstime &now)
{
	if (!_gripper.is_valid()) {
		return;
	}

	_gripper.update();

	// Publish a successful gripper release / grab acknowledgement, and set the current gripper
	// action to GRIPPER_ACTION_NONE to signify we aren't executing any vehicle command now
	if (_gripper.released_read_once()) {
		send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, _cur_vcmd_target_system,
						 _cur_vcmd_target_component);
		PX4_DEBUG("Payload Release Successful Ack Sent!");

		_cur_vcmd_gripper_action = GRIPPER_ACTION_NONE;

	} else if (_gripper.grabbed_read_once()) {
		send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, _cur_vcmd_target_system,
						 _cur_vcmd_target_component);
		PX4_DEBUG("Payload Grab Successful Ack Sent!");

		_cur_vcmd_gripper_action = GRIPPER_ACTION_NONE;

	}
}

void PayloadDeliverer::handle_vehicle_command(const hrt_abstime &now,  const vehicle_command_s *vehicle_command)
{
	// If there's no vehicle command to process, just return
	if (vehicle_command == nullptr) {
		return;
	}

	// Process DO_GRIPPER vehicle command
	if (vehicle_command->command == vehicle_command_s::VEHICLE_CMD_DO_GRIPPER) {
		if (!_gripper.is_valid()) {
			PX4_WARN("Gripper instance not valid but DO_GRIPPER vehicle command was received. Gripper won't work!");
			send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED, vehicle_command->source_system,
							 vehicle_command->source_component);
			return;
		}

		const int32_t gripper_action = (int32_t)roundf(vehicle_command->param2);

		// Flag to indicate if we can process the new gripper command
		bool process_current_gripper_cmd{false};

		// We are currently in the middle of executing a previous vehicle command. Handle the conflicts
		if (_cur_vcmd_gripper_action != GRIPPER_ACTION_NONE) {

			if (gripper_action != _cur_vcmd_gripper_action) {
				// If different action is commanded, cancel the previous vehicle command
				send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED, _cur_vcmd_target_system,
								 _cur_vcmd_target_component);

				// NOTE: Does this make sense when the same entity sends two conflicting commands?
				// For the previous command, the external entity will received CANCELLED result, but then
				// would receive IN_PROGRESS for the new command. As the COMMAND_CANCEL isn't defined properly
				// in MAVLink yet(https://mavlink.io/en/messages/common.html#COMMAND_CANCEL), this can lead to
				// Ground stations to thinking it's new command has been canceled instead of the previous one!

				process_current_gripper_cmd = true;

			} else {
				// If same action is commanded, temporarily reject current command (since it's already in progress)
				send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
								 vehicle_command->source_system, vehicle_command->source_component);
			}

		} else {
			// If we aren't processing any gripper commands, always process the new command
			process_current_gripper_cmd = true;

		}

		// Flag to indicate if we executed the current gripper command
		bool current_gripper_command_executed{false};

		if (process_current_gripper_cmd) {
			if ((_gripper.grabbed() && (gripper_action == vehicle_command_s::GRIPPER_ACTION_GRAB)) ||
			    (_gripper.released() && (gripper_action == vehicle_command_s::GRIPPER_ACTION_RELEASE))) {
				// First check if we already satisfied the requested command. If so, acknowledge as accepted and don't execute the command
				send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
								 vehicle_command->source_system, vehicle_command->source_component);

			} else {
				// Check if the gripper action is valid
				switch (gripper_action) {
				case vehicle_command_s::GRIPPER_ACTION_GRAB:
					_gripper.grab();
					current_gripper_command_executed = true;
					break;

				case vehicle_command_s::GRIPPER_ACTION_RELEASE:
					_gripper.release();
					current_gripper_command_executed = true;
					break;

				default:
					// INVALID GRIPPER ACTION. Deny the command.
					send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED, vehicle_command->source_system,
									 vehicle_command->source_component);
				}
			}
		}

		// Send acknowledgement for successfully executed gripper commands
		if (current_gripper_command_executed) {
			send_gripper_vehicle_command_ack(now, vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS,
							 vehicle_command->source_system, vehicle_command->source_component);

			// Cache the last executed vehicle command information for later acknowledgement
			_cur_vcmd_gripper_action = gripper_action;
			_cur_vcmd_target_system = vehicle_command->source_system;
			_cur_vcmd_target_component = vehicle_command->source_component;
		}
	}
}

bool PayloadDeliverer::send_gripper_vehicle_command(const int32_t gripper_action)
{
	vehicle_command_s vcmd;
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
	vcmd.param2 = gripper_action;
	// Note: Integer type GRIPPER_ACTION gets formatted into a floating point here.
	return _vehicle_command_pub.publish(vcmd);
}

bool PayloadDeliverer::send_gripper_vehicle_command_ack(const hrt_abstime now, const uint8_t command_result,
		const uint8_t target_system, const uint8_t target_component)
{
	vehicle_command_ack_s vcmd_ack{};
	vcmd_ack.timestamp = now;
	vcmd_ack.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
	vcmd_ack.result = command_result;

	switch (command_result) {
	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS:
		// Fill in the progress percentage field for IN_PROGRESS ack message
		vcmd_ack.result_param1 = UINT8_MAX;
		break;
	}

	vcmd_ack.target_system = target_system;
	vcmd_ack.target_component = target_component;
	return _vehicle_command_ack_pub.publish(vcmd_ack);
}

void PayloadDeliverer::gripper_test()
{
	if (!_gripper.is_valid()) {
		PX4_INFO("Gripper is not initialized correctly!");
		return;
	}

	PX4_INFO("Test: Opening the Gripper!");
	send_gripper_vehicle_command(vehicle_command_s::GRIPPER_ACTION_RELEASE);

	px4_usleep(5_s);

	PX4_INFO("Test: Closing the Gripper!");
	send_gripper_vehicle_command(vehicle_command_s::GRIPPER_ACTION_GRAB);
}

void PayloadDeliverer::gripper_open()
{
	if (!_gripper.is_valid()) {
		PX4_INFO("Gripper is not initialized correctly!");
		return;
	}

	send_gripper_vehicle_command(vehicle_command_s::GRIPPER_ACTION_RELEASE);
}

void PayloadDeliverer::gripper_close()
{
	if (!_gripper.is_valid()) {
		PX4_INFO("Gripper is not initialized correctly!");
		return;
	}

	send_gripper_vehicle_command(vehicle_command_s::GRIPPER_ACTION_GRAB);
}

int PayloadDeliverer::print_status()
{
	// Gripper status
	PX4_INFO("Gripper valid: %s", _gripper.is_valid() ? "True" : "False");

	if (_gripper.is_valid()) {
		PX4_INFO("Gripper state: %s", _gripper.get_state_str());
	}

	return 0;
}

int PayloadDeliverer::custom_command(int argc, char *argv[])
{
	if (argc >= 1) {
		// Tests the basic payload open / close ability
		if (strcmp(argv[0], "gripper_test") == 0) {
			get_instance()->gripper_test();
			return 0;

		} else if (strcmp(argv[0], "gripper_open") == 0) {
			get_instance()->gripper_open();
			return 0;

		} else if (strcmp(argv[0], "gripper_close") == 0) {
			get_instance()->gripper_close();
			return 0;
		}
	}

	return print_usage("Unrecognized command");
}

int PayloadDeliverer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Handles payload delivery with either Gripper or a Winch with an appropriate timeout / feedback sensor setting,
and communicates back the delivery result as an acknowledgement internally

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("payload_deliverer", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gripper_test", "Tests the Gripper's release & grabbing sequence");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gripper_open", "Opens the gripper");
	PRINT_MODULE_USAGE_COMMAND_DESCR("gripper_close", "Closes the gripper");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int PayloadDeliverer::task_spawn(int argc, char *argv[])
{
	PayloadDeliverer *instance = new PayloadDeliverer();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("Alloc failed");
	}

	// Cleanup instance in memory and mark this module as invalid to run
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int payload_deliverer_main(int argc, char *argv[])
{
	return PayloadDeliverer::main(argc, argv);
}
