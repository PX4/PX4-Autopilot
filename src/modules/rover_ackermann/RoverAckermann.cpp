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

ModuleBase::Descriptor RoverAckermann::desc{task_spawn, custom_command, print_usage};

RoverAckermann::RoverAckermann() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

bool RoverAckermann::init()
{
	// The module registers its gyro callback first and keeps it, so the setpoints it generates reach the controllers in the same cycle.
	// The controllers only check their trigger here, they register it while enabled, see enableControllers().
	if (!_vehicle_angular_velocity_sub.registerCallback()
	    || !_ackermann_pos_control.init()
	    || !_ackermann_speed_control.init()
	    || !_ackermann_att_control.init()
	    || !_ackermann_rate_control.init()
	    || !_ackermann_act_control.init()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void RoverAckermann::updateParams()
{
	ModuleParams::updateParams();
}

void RoverAckermann::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	// Drain the trigger. The gyro paces the setpoint generation; its value is not used here.
	vehicle_angular_velocity_s angular_velocity;
	_vehicle_angular_velocity_sub.update(&angular_velocity);

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
		enableControllers(true);

	} else if (_was_armed) { // Reset all controllers and stop the vehicle
		enableControllers(false);
		reset();
		_ackermann_act_control.stopVehicle();
		_was_armed = false;

	} else {
		enableControllers(false);
		_ackermann_act_control.stopVehicle();
	}

	// reschedule backup
	ScheduleDelayed(100_ms);
}

void RoverAckermann::generateSetpoints()
{
	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	// Each entry into Guided Course takes the speed the rover is driving at
	if (vehicle_status.nav_state != _nav_state) {
		_nav_state = vehicle_status.nav_state;
		_auto_mode.reset();
	}

	// Navigator stores and acks the speed, Guided Course applies it right away
	vehicle_command_s vehicle_command{};

	while (_vehicle_command_sub.update(&vehicle_command)) {
		// Zero stops driving. Negative is -1 "no change" or -2 "default" in MAVLink,
		// and the speed the mode was entered with is the default there, so both keep the speed.
		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED
		    && _nav_state == vehicle_status_s::NAVIGATION_STATE_GUIDED_COURSE && vehicle_command.param2 > -FLT_EPSILON) {
			_auto_mode.setCourseSpeed(vehicle_command.param2);
		}
	}

	switch (vehicle_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		_auto_mode.autoControl();
		break;

	case vehicle_status_s::NAVIGATION_STATE_GUIDED_COURSE:
		_auto_mode.guidedCourse();
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		_offboard_mode.offboardControl();
		break;

	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_manual_mode.manual();
		break;

	case vehicle_status_s::NAVIGATION_STATE_MANUAL_PARKING:
		_manual_mode.manualParking();
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

void RoverAckermann::enableControllers(const bool enabled)
{
	const bool pos_was_enabled = _ackermann_pos_control.enabled();
	const bool speed_was_enabled = _ackermann_speed_control.enabled();
	const bool rate_was_enabled = _ackermann_rate_control.enabled();
	const bool act_was_enabled = _ackermann_act_control.enabled();

	// Enable top-down, so newly enabled controllers run in cascade order in this cycle
	bool registered = _ackermann_pos_control.setEnabled(enabled && _vehicle_control_mode.flag_control_position_enabled);
	registered = _ackermann_speed_control.setEnabled(enabled && _vehicle_control_mode.flag_control_velocity_enabled) && registered;
	registered = _ackermann_att_control.setEnabled(enabled && _vehicle_control_mode.flag_control_attitude_enabled) && registered;
	registered = _ackermann_rate_control.setEnabled(enabled && _vehicle_control_mode.flag_control_rates_enabled) && registered;
	registered = _ackermann_act_control.setEnabled(enabled && _vehicle_control_mode.flag_control_allocation_enabled) && registered;

	// Callbacks sharing a trigger topic run in registration order: keep the position controller ahead of the speed
	// controller (vehicle_local_position) and the rate controller ahead of the actuator controller (vehicle_angular_velocity)
	if (!pos_was_enabled && _ackermann_pos_control.enabled() && speed_was_enabled && _ackermann_speed_control.enabled()) {
		registered = _ackermann_speed_control.reregisterTrigger() && registered;
	}

	if (!rate_was_enabled && _ackermann_rate_control.enabled() && act_was_enabled && _ackermann_act_control.enabled()) {
		registered = _ackermann_act_control.reregisterTrigger() && registered;
	}

	// Report a failure once, the controllers retry on every cycle
	if (!registered && !_controller_registration_failed) {
		PX4_ERR("controller callback registration failed, retrying");
	}

	_controller_registration_failed = !registered;
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
	_auto_mode.reset();
}

int RoverAckermann::task_spawn(int argc, char *argv[])
{
	RoverAckermann *instance = new RoverAckermann();

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
	return ModuleBase::main(RoverAckermann::desc, argc, argv);
}
