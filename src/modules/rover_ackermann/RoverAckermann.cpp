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
	hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
		runSanityChecks();
	}

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode{};
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);

		// Run sanity checks if the control mode changes (Note: This has to be done this way, because the topic is periodically updated and not on changes)
		if (vehicle_control_mode.flag_control_position_enabled != _vehicle_control_mode.flag_control_position_enabled ||
		    vehicle_control_mode.flag_control_velocity_enabled != _vehicle_control_mode.flag_control_velocity_enabled ||
		    vehicle_control_mode.flag_control_attitude_enabled != _vehicle_control_mode.flag_control_attitude_enabled ||
		    vehicle_control_mode.flag_control_rates_enabled != _vehicle_control_mode.flag_control_rates_enabled) {
			_vehicle_control_mode = vehicle_control_mode;
			runSanityChecks();

		} else {
			_vehicle_control_mode = vehicle_control_mode;
		}

	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.nav_state != _nav_state) {
			_ackermann_pos_control.reset();
			_ackermann_vel_control.reset();
			_ackermann_att_control.reset();
			_ackermann_rate_control.reset();
		}

		_nav_state = vehicle_status.nav_state;
	}

	if (_vehicle_control_mode.flag_armed && _sanity_checks_passed) {
		// Generate setpoints
		if (_vehicle_control_mode.flag_control_manual_enabled) {
			manualControl();

		} else if (_vehicle_control_mode.flag_control_auto_enabled) {
			_ackermann_pos_control.autoPositionMode();

		} else if (_vehicle_control_mode.flag_control_offboard_enabled) {
			offboardControl();
		}

		updateControllers();

	} else { // Reset all controllers and stop motors
		_ackermann_pos_control.reset();
		_ackermann_vel_control.reset();
		_ackermann_att_control.reset();
		_ackermann_rate_control.reset();
		actuator_motors_s actuator_motors{};
		actuator_motors.reversible_flags = _param_r_rev.get();
		actuator_motors.control[0] = 0.f;
		actuator_motors.timestamp = _timestamp;
		_actuator_motors_pub.publish(actuator_motors);
		actuator_servos_s actuator_servos{};
		actuator_servos.control[0] = 0.f;
		actuator_servos.timestamp = _timestamp;
		_actuator_servos_pub.publish(actuator_servos);
	}

}

void RoverAckermann::manualControl()
{
	switch (_nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		_ackermann_act_control.manualMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		_ackermann_rate_control.acroMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		_ackermann_att_control.stabMode();
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		_ackermann_pos_control.manualPositionMode();
		break;
	}
}

bool RoverAckermann::offboardControl()
{
	offboard_control_mode_s offboard_control_mode{};
	_offboard_control_mode_sub.copy(&offboard_control_mode);

	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	if (offboard_control_mode.position) {
		rover_position_setpoint_s rover_position_setpoint{};
		rover_position_setpoint.timestamp = _timestamp;
		rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
		rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
		rover_position_setpoint.start_ned[0] = NAN;
		rover_position_setpoint.start_ned[1] = NAN;
		rover_position_setpoint.cruising_speed = NAN;
		rover_position_setpoint.arrival_speed = NAN;
		rover_position_setpoint.yaw = NAN;
		_rover_position_setpoint_pub.publish(rover_position_setpoint);

	} else if (offboard_control_mode.velocity) {
		const Vector2f velocity_ned(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);
		rover_velocity_setpoint_s rover_velocity_setpoint{};
		rover_velocity_setpoint.timestamp = _timestamp;
		rover_velocity_setpoint.speed = velocity_ned.norm();
		rover_velocity_setpoint.bearing = atan2f(velocity_ned(1), velocity_ned(0));
		_rover_velocity_setpoint_pub.publish(rover_velocity_setpoint);

	}

	return !offboard_control_mode.direct_actuator;
}

void RoverAckermann::updateControllers()
{
	if (_vehicle_control_mode.flag_control_position_enabled) {
		_ackermann_pos_control.updatePosControl();
	}

	if (_vehicle_control_mode.flag_control_velocity_enabled) {
		_ackermann_vel_control.updateVelControl();
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

	if (_vehicle_control_mode.flag_control_velocity_enabled && !_ackermann_vel_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	if (_vehicle_control_mode.flag_control_position_enabled && !_ackermann_pos_control.runSanityChecks()) {
		_sanity_checks_passed = false;
		return;
	}

	_sanity_checks_passed = true;
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
