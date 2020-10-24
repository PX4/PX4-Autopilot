/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "FlightModeManager.hpp"

#include <lib/mathlib/mathlib.h>

using namespace time_literals;

FlightModeManager::FlightModeManager() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

FlightModeManager::~FlightModeManager()
{
	perf_free(_loop_perf);
}

bool FlightModeManager::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed!");
		return false;
	}

	// limit to every other vehicle_local_position update (~62.5 Hz)
	_vehicle_local_position_sub.set_interval_us(16_ms);
	_time_stamp_last_loop = hrt_absolute_time();
	return true;
}

void FlightModeManager::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		_flight_tasks.handleParameterUpdate();
	}

	// generate setpoints on local position changes
	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		_home_position_sub.update();
		_vehicle_control_mode_sub.update();
		_vehicle_land_detected_sub.update();
		_vehicle_local_position_setpoint_sub.update();
		_vehicle_status_sub.update();

		const hrt_abstime time_stamp_now = hrt_absolute_time();
		// Guard against too small (< 0.2ms) and too large (> 100ms) dt's.
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) / 1e6f), 0.0002f, 0.1f);
		_time_stamp_last_loop = time_stamp_now;

		start_flight_task();

		if (_flight_tasks.isAnyTaskActive()) {
			generateTrajectorySetpoint(dt, vehicle_local_position);
		}

	}

	perf_end(_loop_perf);
}

void FlightModeManager::start_flight_task()
{
	bool task_failure = false;
	bool should_disable_task = true;
	int prev_failure_count = _task_failure_count;

	// Do not run any flight task for VTOLs in fixed-wing mode
	if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
		return;
	}

	if (_vehicle_status_sub.get().in_transition_mode) {

		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Transition);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Transition activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

		return;
	}

	// offboard
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	    && (_vehicle_control_mode_sub.get().flag_control_altitude_enabled ||
		_vehicle_control_mode_sub.get().flag_control_position_enabled ||
		_vehicle_control_mode_sub.get().flag_control_climb_rate_enabled ||
		_vehicle_control_mode_sub.get().flag_control_velocity_enabled ||
		_vehicle_control_mode_sub.get().flag_control_acceleration_enabled)) {

		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Offboard);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Offboard activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// Auto-follow me
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		should_disable_task = false;
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::AutoFollowMe);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Follow-Me activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_vehicle_control_mode_sub.get().flag_control_auto_enabled) {
		// Auto related maneuvers
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::AutoLineSmoothVel);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Auto activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {

		// Emergency descend
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		error =  _flight_tasks.switchTask(FlightTaskIndex::Descend);

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Descend activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	}

	// manual position control
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Position-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}

	// manual altitude control
	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		should_disable_task = false;
		FlightTaskError error = FlightTaskError::NoError;

		switch (_param_mpc_pos_mode.get()) {
		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmooth);
			break;

		case 3:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitudeSmoothVel);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualAltitude);
			break;
		}

		if (error != FlightTaskError::NoError) {
			if (prev_failure_count == 0) {
				PX4_WARN("Altitude-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}

			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_ALTCTL);
			task_failure = false;
		}
	}

	if (_vehicle_status_sub.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ORBIT) {
		should_disable_task = false;
	}

	// check task failure
	if (task_failure) {

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		FlightTaskError error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != FlightTaskError::NoError) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}

	} else if (should_disable_task) {
		_flight_tasks.switchTask(FlightTaskIndex::None);
	}
}

void FlightModeManager::check_failure(bool task_failure, uint8_t nav_state)
{
	if (!task_failure) {
		// we want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// tell commander to switch mode
		PX4_WARN("Previous flight task failed, switching to mode %d", nav_state);
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}

void FlightModeManager::send_vehicle_cmd_do(uint8_t nav_state)
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// set the main mode
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	default: //vehicle_status_s::NAVIGATION_STATE_POSCTL
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;
	}

	// publish the vehicle command
	_vehicle_command_pub.publish(command);
}

void FlightModeManager::generateTrajectorySetpoint(const float dt,
		const vehicle_local_position_s &vehicle_local_position)
{
	// Inform FlightTask about the input and output of the velocity controller
	// This is used to properly initialize the velocity setpoint when onpening the position loop (position unlock)
	_flight_tasks.updateVelocityControllerIO(Vector3f(_vehicle_local_position_setpoint_sub.get().vx,
			_vehicle_local_position_setpoint_sub.get().vy, _vehicle_local_position_setpoint_sub.get().vz),
			Vector3f(_vehicle_local_position_setpoint_sub.get().acceleration));

	// setpoints and constraints for the position controller from flighttask or failsafe
	vehicle_local_position_setpoint_s setpoint = FlightTask::empty_setpoint;
	vehicle_constraints_s constraints = FlightTask::empty_constraints;

	_flight_tasks.setYawHandler(_wv_controller);

	if (_flight_tasks.update()) {
		setpoint = _flight_tasks.getPositionSetpoint();
		constraints = _flight_tasks.getConstraints();

		//_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now); TODO

	} else {
		// FAILSAFE
		// Task was not able to update correctly. Do Failsafe.
		// by sending out an empty setpoint that is not executable to make the position controller failsafe
		// TODO test failsafe
	}

	landing_gear_s landing_gear = _flight_tasks.getGear();

	// limit altitude according to land detector
	limitAltitude(setpoint, vehicle_local_position);

	const bool not_taken_off = _takeoff.getTakeoffState() < TakeoffState::rampup;
	const bool flying = _takeoff.getTakeoffState() >= TakeoffState::flight;
	const bool flying_but_ground_contact = flying && _vehicle_land_detected_sub.get().ground_contact;

	if (not_taken_off || flying_but_ground_contact) {
		// we are not flying yet and need to avoid any corrections
		reset_setpoint_to_nan(setpoint);
		Vector3f(0.f, 0.f, 100.f).copyTo(setpoint.acceleration); // High downwards acceleration to make sure there's no thrust
		// set yaw-sp to current yaw
		setpoint.yawspeed = 0.f;
		// prevent any integrator windup
		// _control.resetIntegral(); TODO
	}

	_trajectory_setpoint_pub.publish(setpoint);

	// if (flying) { TODO
	// 	_control.setThrustLimits(_param_mpc_thr_min.get(), _param_mpc_thr_max.get());

	// } else {
	// 	// allow zero thrust when taking off and landing
	// 	_control.setThrustLimits(0.f, _param_mpc_thr_max.get());
	// }

	// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
	// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
	if (!PX4_ISFINITE(constraints.speed_up) || (constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
		constraints.speed_up = _param_mpc_z_vel_max_up.get();
	}

	// limit tilt during takeoff ramupup
	if (_takeoff.getTakeoffState() < TakeoffState::flight) {
		constraints.tilt = math::radians(_param_mpc_tiltmax_lnd.get());
	}

	// handle smooth takeoff
	_takeoff.updateTakeoffState(_vehicle_control_mode_sub.get().flag_armed, _vehicle_land_detected_sub.get().landed,
				    constraints.want_takeoff, constraints.speed_up, !_vehicle_control_mode_sub.get().flag_control_climb_rate_enabled,
				    _time_stamp_last_loop);
	constraints.speed_up = _takeoff.updateRamp(dt, constraints.speed_up);

	_vehicle_constraints_pub.publish(constraints);

	if (not_taken_off) {
		// reactivate the task which will reset the setpoint to current state
		_flight_tasks.reActivate();
	}

	// if there's any change in landing gear setpoint publish it
	if (landing_gear.landing_gear != _old_landing_gear_position
	    && landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
		landing_gear.timestamp = _time_stamp_last_loop;
		_landing_gear_pub.publish(landing_gear);
	}

	_old_landing_gear_position = landing_gear.landing_gear;
}

void FlightModeManager::limitAltitude(vehicle_local_position_setpoint_s &setpoint,
				      const vehicle_local_position_s &vehicle_local_position)
{
	if (_vehicle_land_detected_sub.get().alt_max < 0.0f || !_home_position_sub.get().valid_alt
	    || !vehicle_local_position.z_valid || !vehicle_local_position.v_z_valid) {
		// there is no altitude limitation present or the required information not available
		return;
	}

	// maximum altitude == minimal z-value (NED)
	const float min_z = _home_position_sub.get().z + (-_vehicle_land_detected_sub.get().alt_max);

	if (vehicle_local_position.z < min_z) {
		// above maximum altitude, only allow downwards flight == positive vz-setpoints (NED)
		setpoint.z = min_z;
		setpoint.vz = math::max(setpoint.vz, 0.f);
	}
}

void FlightModeManager::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

int FlightModeManager::task_spawn(int argc, char *argv[])
{
	FlightModeManager *instance = new FlightModeManager();

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

int FlightModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlightModeManager::print_status()
{
	if (_flight_tasks.isAnyTaskActive()) {
		PX4_INFO("Running, active flight task: %i", _flight_tasks.getActiveTask());

	} else {
		PX4_INFO("Running, no flight task active");
	}

	perf_print_counter(_loop_perf);
	return 0;
}

int FlightModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flight_mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flight_mode_manager_main(int argc, char *argv[])
{
	return FlightModeManager::main(argc, argv);
}
