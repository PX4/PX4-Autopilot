/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "FailureInjectionManager.hpp"

#include <cmath>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

ModuleBase::Descriptor FailureInjectionManager::desc{task_spawn, custom_command, print_usage};

FailureInjectionManager::FailureInjectionManager() :
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_handle_sys_failure_en = param_find("SYS_FAILURE_EN");
}

bool FailureInjectionManager::init()
{
	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_failure_injection_pub.advertise();

	return true;
}

bool FailureInjectionManager::failureInjectionEnabled()
{
	int32_t enabled = 0;
	param_get(_handle_sys_failure_en, &enabled);
	return enabled != 0;
}

void FailureInjectionManager::Run()
{
	if (should_exit()) {
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_command_s cmd;

	while (_vehicle_command_sub.update(&cmd)) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			handleCommand(cmd);
		}
	}

	// Republish only when the configuration actually changed
	if (_table.changed()) {
		failure_injection_s msg{};
		_table.fill(msg);
		msg.timestamp = hrt_absolute_time();
		_failure_injection_pub.publish(msg);
		_table.clearChanged();
	}
}

void FailureInjectionManager::handleCommand(const vehicle_command_s &cmd)
{
	// Defense in depth: the mavlink receiver and the `failure` command already
	// gate on SYS_FAILURE_EN, but enforce it here too as the sole consumer.
	if (!failureInjectionEnabled()) {
		publishAck(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
		return;
	}

	const uint8_t unit = static_cast<uint8_t>(lroundf(cmd.param1));
	const uint8_t type = static_cast<uint8_t>(lroundf(cmd.param2));
	const uint8_t instance = static_cast<uint8_t>(lroundf(cmd.param3));

	uint8_t result;

	switch (_table.inject(unit, type, instance)) {
	case failure_injection::FailureTable::AckResult::Accepted:
		result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
		break;

	case failure_injection::FailureTable::AckResult::Rejected:
		result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
		break;

	case failure_injection::FailureTable::AckResult::Unsupported:
	default:
		result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		break;
	}

	publishAck(cmd, result);
}

void FailureInjectionManager::publishAck(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s ack{};
	ack.command = cmd.command;
	ack.from_external = false;
	ack.result = result;
	ack.timestamp = hrt_absolute_time();
	_command_ack_pub.publish(ack);
}

int FailureInjectionManager::task_spawn(int argc, char *argv[])
{
	FailureInjectionManager *instance = new FailureInjectionManager();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int FailureInjectionManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FailureInjectionManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

The failure injection manager is the single subscriber to `vehicle_command` for
`MAV_CMD_INJECT_FAILURE`. It maintains the set of currently active failures and
publishes the `failure_injection` topic, republishing only when the configuration
changes so that command spam cannot propagate to the consumers that apply the
failures. It also produces the central `vehicle_command_ack`.

Failure injection is gated by the `SYS_FAILURE_EN` parameter.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("failure_injection_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int failure_injection_manager_main(int argc, char *argv[])
{
	return ModuleBase::main(FailureInjectionManager::desc, argc, argv);
}
