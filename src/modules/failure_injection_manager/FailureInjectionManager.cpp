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
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

ModuleBase::Descriptor FailureInjectionManager::desc{task_spawn, custom_command, print_usage};

FailureInjectionManager::FailureInjectionManager() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool FailureInjectionManager::init()
{
	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("vehicle_command callback registration failed");
		return false;
	}

	if (_param_sys_fail_rc_src.get() != 0) {
		if (!_manual_control_setpoint_sub.registerCallback()) {
			PX4_ERR("manual_control_setpoint callback registration failed");
			return false;
		}
	}

	_failure_injection_pub.advertise();

	return true;
}

void FailureInjectionManager::Run()
{
	if (should_exit()) {
		_vehicle_command_sub.unregisterCallback();
		_manual_control_setpoint_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_command_s cmd;

	for (int i = 0; i < vehicle_command_s::ORB_QUEUE_LENGTH && _vehicle_command_sub.update(&cmd); i++) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			handleCommand(cmd);
		}
	}

	evaluateRcInjection();

	// Republish only when the configuration actually changed
	if (_table.changed()) {
		failure_injection_s msg{};
		_table.fill(msg);
		msg.timestamp = hrt_absolute_time();
		_failure_injection_pub.publish(msg);
		_table.clearChanged();
	}
}

void FailureInjectionManager::evaluateRcInjection()
{
	if (_param_sys_fail_rc_src.get() == 0) {
		return;
	}

	manual_control_setpoint_s manual_control_setpoint;

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		float value = NAN;

		if (manual_control_setpoint.valid) {
			switch (_param_sys_fail_rc_src.get()) {
			case 1: value = manual_control_setpoint.aux1; break;

			case 2: value = manual_control_setpoint.aux2; break;

			case 3: value = manual_control_setpoint.aux3; break;

			case 4: value = manual_control_setpoint.aux4; break;

			case 5: value = manual_control_setpoint.aux5; break;

			case 6: value = manual_control_setpoint.aux6; break;

			default: break; // 0 = disabled, or out of range
			}
		}

		const bool triggered = value > 0.5f;

		if (triggered && !_rc_active) {
			_rc_active_unit = static_cast<uint8_t>(_param_sys_fail_rc_unit.get());
			_rc_active_instance = static_cast<uint8_t>(_param_sys_fail_rc_inst.get());
			_rc_active = true;

			_table.inject(_rc_active_unit, static_cast<uint8_t>(_param_sys_fail_rc_mode.get()), _rc_active_instance);

		} else if (!triggered && _rc_active) {
			_table.inject(_rc_active_unit, failure_injection_s::FAILURE_TYPE_OK, _rc_active_instance);
			_rc_active = false;
		}
	}
}

void FailureInjectionManager::handleCommand(const vehicle_command_s &cmd)
{
	const uint8_t unit = static_cast<uint8_t>(lroundf(cmd.param1));
	const uint8_t type = static_cast<uint8_t>(lroundf(cmd.param2));

	failure_injection::FailureTable::AckResult ack;

	if (PX4_ISFINITE(cmd.param3)) {
		ack = _table.inject(unit, type, static_cast<uint8_t>(lroundf(cmd.param3)));

	} else {
		const uint16_t mask = PX4_ISFINITE(cmd.param4) ? static_cast<uint16_t>(lroundf(cmd.param4)) : 0;
		ack = _table.injectMask(unit, type, mask);
	}

	uint8_t result;

	switch (ack) {
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

Central module for handling failure injection. It collects failure requests, tracks
the set of active failures, and publishes them on the `failure_injection` topic for
the apply-sites to act on.

Failures can be triggered through:
- `MAV_CMD_INJECT_FAILURE` over MAVLink (e.g. from MAVSDK)
- the `failure` console command
- an RC switch: `SYS_FAIL_RC_SRC` selects the aux input, and `SYS_FAIL_RC_UNIT` /
  `SYS_FAIL_RC_MODE` / `SYS_FAIL_RC_INST` define the failure applied while it is on

Requires `SYS_FAILURE_EN` to be set; the startup script only starts this module when it is.

Failures can be applied both in simulation and on real hardware, where the apply-sites are
compiled in alongside this module.

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
