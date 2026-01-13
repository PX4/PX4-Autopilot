/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file SpacecraftHandler.cpp
 *
 * Spacecraft control handler.
 *
 * @author Pedro Roque <padr@kth.se>
 */

#include "SpacecraftHandler.hpp"


using namespace time_literals;

SpacecraftHandler::SpacecraftHandler() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool SpacecraftHandler::init()
{
	ScheduleOnInterval(4_ms); // 250 Hz
	return true;
}

void SpacecraftHandler::updateParams()
{
	ModuleParams::updateParams();
}

void SpacecraftHandler::Run()
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	_spacecraft_position_control.updatePositionControl();
	_spacecraft_attitude_control.updateAttitudeControl();
	_spacecraft_rate_control.updateRateControl();

}

int SpacecraftHandler::task_spawn(int argc, char *argv[])
{
	SpacecraftHandler *instance = new SpacecraftHandler();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SpacecraftHandler::print_status()
{
	PX4_INFO("Running");

	return 0;
}

int SpacecraftHandler::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SpacecraftHandler::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
	### Description
	This implements control allocation for spacecraft vehicles.
	It takes torque and thrust setpoints as inputs and outputs
	actuator setpoint messages.
	)DESCR_STR"
	);

	PRINT_MODULE_USAGE_NAME("spacecraft", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int spacecraft_main(int argc, char *argv[]);

int spacecraft_main(int argc, char *argv[])
{
	return SpacecraftHandler::main(argc, argv);
}
