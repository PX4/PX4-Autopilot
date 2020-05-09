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
 * @file airship_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 * @author Daniel Robinson  <daniel@flycloudline.com>
 */

#include "airship_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

AirshipAttitudeControl::AirshipAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "airship_att_control"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;

	_rates_sp.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	parameters_updated();
}

AirshipAttitudeControl::~AirshipAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
AirshipAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
AirshipAttitudeControl::parameters_updated()
{

}

void
AirshipAttitudeControl::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void
AirshipAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_actuators_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);
				_attitude_sp_id = ORB_ID(mc_virtual_attitude_setpoint);

				int32_t vt_type = -1;

				if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
					_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
				}

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_sp_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

bool
AirshipAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	const uint8_t prev_quat_reset_counter = _v_att.quat_reset_counter;

	if (_v_att_sub.update(&_v_att)) {
		// Check for a heading reset
		if (prev_quat_reset_counter != _v_att.quat_reset_counter) {
			// we only extract the heading change from the delta quaternion
			_man_yaw_sp += Eulerf(Quatf(_v_att.delta_q_reset)).psi();
		}

		return true;
	}

	return false;
}

float
AirshipAttitudeControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_vehicle_land_detected.landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

void
AirshipAttitudeControl::publish_rates_setpoint()
{
	_v_rates_sp.roll = _rates_sp(0);
	_v_rates_sp.pitch = -_rates_sp(1);
	_v_rates_sp.yaw = _rates_sp(2);
	_v_rates_sp.thrust_body[0] = 0.0f;
	_v_rates_sp.thrust_body[1] = 0.0f;
	_v_rates_sp.thrust_body[2] = -_thrust_sp;
	_v_rates_sp.timestamp = hrt_absolute_time();

	_v_rates_sp_pub.publish(_v_rates_sp);
}

void
AirshipAttitudeControl::publish_rate_controller_status()
{
	rate_ctrl_status_s rate_ctrl_status = {};
	rate_ctrl_status.timestamp = hrt_absolute_time();
	_controller_status_pub.publish(rate_ctrl_status);
}

void
AirshipAttitudeControl::publish_actuator_controls()
{
	_actuators.control[0] = 0.0f;
	_actuators.control[1] = _manual_control_sp.x;
	_actuators.control[2] = _manual_control_sp.r;
	_actuators.control[3] = _manual_control_sp.z;
	_actuators.control[7] = (float)_landing_gear.landing_gear;
	// note: _actuators.timestamp_sample is set in AirshipAttitudeControl::Run()
	_actuators.timestamp = hrt_absolute_time();

	if (!_actuators_0_circuit_breaker_enabled) {
		orb_publish_auto(_actuators_id, &_actuators_0_pub, &_actuators, nullptr, ORB_PRIO_DEFAULT);
	}
}

void
AirshipAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = hrt_absolute_time();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};

		_actuators.timestamp_sample = angular_velocity.timestamp_sample;

		/* run the rate controller immediately after a gyro update */
		if (_v_control_mode.flag_control_rates_enabled) {
			publish_actuator_controls();
			publish_rate_controller_status();
		}

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);
		_battery_status_sub.update(&_battery_status);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		_landing_gear_sub.update(&_landing_gear);
		vehicle_status_poll();
		const bool manual_control_updated = _manual_control_sp_sub.update(&_manual_control_sp);
		const bool attitude_updated = vehicle_attitude_poll();

		_attitude_dt += dt;

		bool attitude_setpoint_generated = _v_control_mode.flag_control_altitude_enabled
						   || _v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_position_enabled;

		if (attitude_setpoint_generated) {
			if (attitude_updated) {

				if (_v_control_mode.flag_control_yawrate_override_enabled) {
					/* Yaw rate override enabled, overwrite the yaw setpoint */
					_v_rates_sp_sub.update(&_v_rates_sp);
					const auto yawrate_reference = _v_rates_sp.yaw;
					_rates_sp(2) = yawrate_reference;
				}

				publish_rates_setpoint();
			}

		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			if (_v_control_mode.flag_control_manual_enabled) {
				if (manual_control_updated) {
					/* manual control - feed RC commands to actuators directly */
					_v_att_sp.thrust_body[0] = _manual_control_sp.x;
					_v_att_sp.thrust_body[2] = _manual_control_sp.z;
					_rates_sp(2) = _manual_control_sp.r;

					// PX4_INFO("\nManual X: %.2f\nManual Z: %.2f\nManual R: %.2f\n",
					// 	(double)_manual_control_sp.x, (double)_manual_control_sp.z,
					// 	(double)_manual_control_sp.r);

					publish_rates_setpoint();
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_rates_sp_sub.update(&_v_rates_sp)) {
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = -_v_rates_sp.thrust_body[2];
				}
			}
		}


		if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				_rates_sp.zero();
				_thrust_sp = 0.0f;
				_att_control.zero();
				publish_actuator_controls();
			}
		}

		/* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
		if (!_v_control_mode.flag_armed || (now - _task_start) < 3300000) {
			_dt_accumulator += dt;
			++_loop_counter;

			if (_dt_accumulator > 1.f) {
				const float loop_update_rate = (float)_loop_counter / _dt_accumulator;
				_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
				_dt_accumulator = 0;
				_loop_counter = 0;
			}
		}

		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int AirshipAttitudeControl::task_spawn(int argc, char *argv[])
{
	AirshipAttitudeControl *instance = new AirshipAttitudeControl();

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

int AirshipAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	print_message(_actuators);

	return 0;
}

int AirshipAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AirshipAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airship_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int airship_att_control_main(int argc, char *argv[])
{
	return AirshipAttitudeControl::main(argc, argv);
}
