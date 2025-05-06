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

#include "RoverMecanum.hpp"

using namespace time_literals;

RoverMecanum::RoverMecanum() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool RoverMecanum::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverMecanum::updateParams()
{
	ModuleParams::updateParams();
}

void RoverMecanum::Run()
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	_mecanum_pos_control.updatePosControl();
	_mecanum_vel_control.updateVelControl();
	_mecanum_att_control.updateAttControl();
	_mecanum_rate_control.updateRateControl();

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	const bool full_manual_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
					      && !_vehicle_control_mode.flag_control_position_enabled && !_vehicle_control_mode.flag_control_attitude_enabled
					      && !_vehicle_control_mode.flag_control_rates_enabled;

	if (full_manual_mode_enabled) { // Manual mode
		_mecanum_act_control.manualManualMode();
	}

	if (_vehicle_control_mode.flag_armed) {
		_mecanum_act_control.updateActControl();

	}

}

int RoverMecanum::task_spawn(int argc, char *argv[])
{
	RoverMecanum *instance = new RoverMecanum();

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

int RoverMecanum::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverMecanum::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover mecanum module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_mecanum", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_mecanum_main(int argc, char *argv[])
{
	return RoverMecanum::main(argc, argv);
}
