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
	ScheduleOnInterval(1_s);

	// Additionallly to 1Hz scheduling, add cb for vehicle_command message
	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	initialize_gripper();
	return true;
}

bool PayloadDeliverer::initialize_gripper()
{
	// If gripper instance is invalid, try initializing it
	if (!_gripper.is_valid() && _param_gripper_enable.get()) {
		GripperConfig config{};
		config.type = (GripperConfig::GripperType)_param_gripper_type.get();
		config.sensor = GripperConfig::GripperSensorType::NONE; // Feedback sensor isn't supported for now
		config.timeout_us = hrt_abstime(_param_gripper_timeout_s.get() * 1000000ULL);
		_gripper.init(config);
	}

	// NOTE: Support for changing gripper sensor type / gripper type configuration when
	// the parameter change is detected isn't added as we don't have actual use case for that
	// yet!

	if (!_gripper.is_valid()) {
		PX4_DEBUG("Gripper object initialization invalid!");
		return false;

	} else {
		_gripper.update();

		// If gripper wasn't commanded to go to grab position, command.
		if (!_gripper.grabbed() && !_gripper.grabbing()) {
			PX4_DEBUG("Gripper intialize: putting to grab position!");
			send_gripper_vehicle_command(vehicle_command_s::GRIPPER_ACTION_GRAB);
		}

		return true;
	}

}

void PayloadDeliverer::parameter_update()
{
	updateParams();
	initialize_gripper();
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

	// Update delivery mechanism's state
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
		// Try initializing gripper
		initialize_gripper();
		return;
	}

	_gripper.update();

	// Publish a successful gripper release acknowledgement
	if (_gripper.released_read_once()) {
		vehicle_command_ack_s vcmd_ack{};
		vcmd_ack.timestamp = now;
		vcmd_ack.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
		vcmd_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
		// Ideally, we need to fill out target_system and target_component to match the vehicle_command we are acknowledging for
		// But since we are not tracking the vehicle command's source system & component, we don't fill it out for now.
		_vehicle_command_ack_pub.publish(vcmd_ack);
		PX4_DEBUG("Payload Drop Successful Ack Sent!");
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
			// If gripper instance is invalid, send ACK command with FAILED result, and warn the user
			vehicle_command_ack_s vcmd_ack{};
			vcmd_ack.timestamp = now;
			vcmd_ack.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
			vcmd_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
			vcmd_ack.target_system = vehicle_command->source_system;
			vcmd_ack.target_component = vehicle_command->source_component;
			_vehicle_command_ack_pub.publish(vcmd_ack);
			PX4_WARN("Gripper instance not valid but DO_GRIPPER vehicle command was received. Gripper won't work!");
			return;
		}

		const int32_t gripper_action = (int32_t)vehicle_command->param2;
		// Note: although the 'param2' is a floating point type, the enums for GRIPPER_ACTION are converted into
		// floating point, then gets sent by the Ground Control Station to trigger gripper action for example.
		// This is an inherent MAVLink standard limitation (converting enums into floats)!

		switch (gripper_action) {
		case vehicle_command_s::GRIPPER_ACTION_GRAB:
			_gripper.grab();
			break;

		case vehicle_command_s::GRIPPER_ACTION_RELEASE:
			_gripper.release();
			break;
		}

		// Send IN_PROGRESS vehicle_command_ack message to alert that DO_GRIPPER command is getting processed
		// Since QGC/AMC doesn't impose timeout for IN_PROGRESS vehicle command acks, it is suffice to send it only once.
		vehicle_command_ack_s vcmd_ack{};
		vcmd_ack.timestamp = now;
		vcmd_ack.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
		vcmd_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;
		vcmd_ack.result_param1 = UINT8_MAX; // Set progress percentage to UINT8_MAX, indicating it is unkonwn
		vcmd_ack.target_system = vehicle_command->source_system;
		vcmd_ack.target_component = vehicle_command->source_component;
		_vehicle_command_ack_pub.publish(vcmd_ack);
		PX4_DEBUG("Payload Drop In-Progress Ack Sent!");
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
