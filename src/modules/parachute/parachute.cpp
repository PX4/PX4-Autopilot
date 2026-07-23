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
#include "parachute.h"

#include <px4_platform_common/events.h>

ModuleBase::Descriptor Parachute::desc{task_spawn, custom_command, print_usage};

Parachute::Parachute()
	: WorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool Parachute::init()
{
	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// advertise on startup so that the topic exists when the logger checks for it
	_parachute_pub.advertise();

	return true;
}

void Parachute::Run()
{
	if (should_exit()) {
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE) {
			handle_parachute_command(vehicle_command);
		}
	}
}

void Parachute::handle_parachute_command(const vehicle_command_s &vehicle_command)
{
	const int32_t parachute_action = (int32_t)roundf(vehicle_command.param1);

	switch (parachute_action) {
	case vehicle_command_s::PARACHUTE_ACTION_RELEASE: {
			// never release the parachute on the ground (also denied when
			// the land detector has not published yet). In-air releases stay possible in any
			// arming state, e.g. after an in-air kill.
			vehicle_land_detected_s land_detected{};
			const bool landed = !_vehicle_land_detected_sub.copy(&land_detected) || land_detected.landed;

			if (landed) {
				events::send(events::ID("parachute_release_denied_landed"), events::Log::Warning,
					     "Parachute release denied: vehicle is landed");
				send_vehicle_command_ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
							 vehicle_command.source_system, vehicle_command.source_component);
				break;
			}

			parachute_s parachute{};
			parachute.timestamp = hrt_absolute_time();
			parachute.command = parachute_s::COMMAND_RELEASE;
			_parachute_pub.publish(parachute);
			_released = true;

			events::send(events::ID("parachute_release"), events::Log::Info, "Releasing parachute");
			send_vehicle_command_ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED,
						 vehicle_command.source_system, vehicle_command.source_component);
			break;
		}

	default:
		// PARACHUTE_ACTION_ENABLE / DISABLE are not supported by the parachute output function
		send_vehicle_command_ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED,
					 vehicle_command.source_system, vehicle_command.source_component);
		break;
	}
}

bool Parachute::send_vehicle_command_ack(const uint8_t command_result, const uint8_t target_system,
		const uint8_t target_component)
{
	vehicle_command_ack_s vehicle_command_ack{};
	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
	vehicle_command_ack.result = command_result;
	vehicle_command_ack.target_system = target_system;
	vehicle_command_ack.target_component = target_component;
	return _vehicle_command_ack_pub.publish(vehicle_command_ack);
}

void Parachute::release()
{
	// Send the release as a vehicle command so that external parachute systems
	// listening for DO_PARACHUTE are triggered as well
	vehicle_command_s vehicle_command{};
	vehicle_command.timestamp = hrt_absolute_time();
	vehicle_command.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
	vehicle_command.param1 = vehicle_command_s::PARACHUTE_ACTION_RELEASE;
	_vehicle_command_pub.publish(vehicle_command);
}

int Parachute::print_status()
{
	PX4_INFO("Parachute released: %s", _released ? "True" : "False");
	return 0;
}

int Parachute::custom_command(int argc, char *argv[])
{
	if (argc >= 1) {
		if (strcmp(argv[0], "release") == 0) {
			get_instance<Parachute>(desc)->release();
			return 0;
		}
	}

	return print_usage("Unrecognized command");
}

int Parachute::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Releases the parachute output function when the DO_PARACHUTE vehicle command is
received, without going through flight termination. The release is latching:
once released, the parachute stays released until reboot.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("parachute", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("release", "Releases the parachute");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Parachute::task_spawn(int argc, char *argv[])
{
	Parachute *instance = new Parachute();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("Alloc failed");
	}

	// Cleanup instance in memory and mark this module as invalid to run
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int parachute_main(int argc, char *argv[])
{
	return ModuleBase::main(Parachute::desc, argc, argv);
}
