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

#include "RoverAckermann.hpp"

using namespace time_literals;

RoverAckermann::RoverAckermann() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

bool RoverAckermann::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverAckermann::updateParams()
{
	ModuleParams::updateParams();
}

void RoverAckermann::Run()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
		runSanityChecks();
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode{};
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);

		// Run sanity checks if the control mode changes (Note: This has to be done this way, because the topic is periodically updated at 2 Hz)
		if (_vehicle_control_mode.flag_control_position_enabled != vehicle_control_mode.flag_control_position_enabled ||
		    _vehicle_control_mode.flag_control_velocity_enabled != vehicle_control_mode.flag_control_velocity_enabled ||
		    _vehicle_control_mode.flag_control_attitude_enabled != vehicle_control_mode.flag_control_attitude_enabled ||
		    _vehicle_control_mode.flag_control_rates_enabled != vehicle_control_mode.flag_control_rates_enabled ||
		    _vehicle_control_mode.flag_control_allocation_enabled != vehicle_control_mode.flag_control_allocation_enabled) {
			_vehicle_control_mode = vehicle_control_mode;
			runSanityChecks();
			reset();

		} else {
			_vehicle_control_mode = vehicle_control_mode;
		}
	}

	if (_vehicle_control_mode.flag_armed && _sanity_checks_passed) {
		_was_armed = true;
		generateSetpoints();
		updateControllers();

	} else if (_was_armed) { // Reset all controllers and stop the vehicle
		reset();
		_ackermann_act_control.stopVehicle();
		_was_armed = false;
	}

}

void RoverAckermann::generateSetpoints()
{
	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	switch (vehicle_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		_auto_mode.autoControl();
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		_offboard_mode.offboardControl();
		break;

	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_manual_mode.manual();
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		_manual_mode.acro();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		_manual_mode.stab();
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		_manual_mode.position();
		break;

	default:
		break;
	}

}

void RoverAckermann::updateControllers()
{
	if (_vehicle_control_mode.flag_control_position_enabled) {
		_ackermann_pos_control.updatePosControl();
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled) {
		_ackermann_speed_control.updateSpeedControl();
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled) {
		_ackermann_att_control.updateAttControl();
	}

	if (_vehicle_control_mode.flag_control_rates_enabled) {
		_ackermann_rate_control.updateRateControl();
	}

	if (_vehicle_control_mode.flag_control_allocation_enabled) {
		_ackermann_act_control.updateActControl();
	}
}

void RoverAckermann::runSanityChecks()
{
	if (_vehicle_control_mode.flag_control_rates_enabled && !_ackermann_rate_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_attitude_enabled && !_ackermann_att_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled && !_ackermann_speed_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_position_enabled && !_ackermann_pos_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	_sanity_checks_passed = true;
}

void RoverAckermann::reset()
{
	_ackermann_pos_control.reset();
	_ackermann_speed_control.reset();
	_ackermann_att_control.reset();
	_ackermann_rate_control.reset();
	_manual_mode.reset();
}

int RoverAckermann::task_spawn(int argc, char *argv[])
{
	RoverAckermann *instance = new RoverAckermann();

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

int RoverAckermann::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverAckermann::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover ackermann module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_ackermann", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_ackermann_main(int argc, char *argv[])
{
	return RoverAckermann::main(argc, argv);
}
