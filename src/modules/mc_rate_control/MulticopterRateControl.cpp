/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuator_controls_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0))
{
	parameters_updated();

	_controller_status_pub.advertise();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));
}

void MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	perf_begin(_loop_perf);

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_rate_control_enabled = vehicle_control_mode.flag_control_rates_enabled;
			_attitude_control_enabled = vehicle_control_mode.flag_control_attitude_enabled;
			_manual_enabled = vehicle_control_mode.flag_control_manual_enabled;
		}
	}

	// run controller on gyro changes
	vehicle_angular_velocity_s angular_velocity;
	vehicle_angular_acceleration_s angular_acceleration;

	if (_rate_control_enabled && _vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		if (!_vehicle_angular_acceleration_sub.copy(&angular_acceleration)) {
			Vector3f().copyTo(angular_acceleration.xyz);
		}

		{
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed || vehicle_land_detected.maybe_landed;
			}
		}

		{
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.update(&vehicle_status)) {
				_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
			}
		}

		{
			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				if (!control_allocator_status.torque_setpoint_achieved) {

					Vector<bool, 3> saturation_positive{};
					Vector<bool, 3> saturation_negative{};

					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}

					// TODO: send the unallocated value directly for better anti-windup
					_rate_control.setSaturationStatus(saturation_positive, saturation_negative);

				} else {
					_rate_control.clearSaturationStatus();
				}
			}
		}

		if (_manual_enabled && !_attitude_control_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				// manual rate control acro mode rate limits
				const Vector3f acro_rate_max(radians(_param_mc_acro_r_max.get()),
							     radians(_param_mc_acro_p_max.get()),
							     radians(_param_mc_acro_y_max.get()));

				_rates_setpoint = man_rate_sp.emult(acro_rate_max);

				_thrust_setpoint(0) = 0.f;
				_thrust_setpoint(1) = 0.f;
				_thrust_setpoint(2) = -math::constrain(manual_control_setpoint.z, 0.f, 1.f);

				// publish rate setpoint
				vehicle_rates_setpoint_s vehicle_rates_setpoint;
				vehicle_rates_setpoint.timestamp_sample = manual_control_setpoint.timestamp_sample;
				vehicle_rates_setpoint.roll  = _rates_setpoint(0);
				vehicle_rates_setpoint.pitch = _rates_setpoint(1);
				vehicle_rates_setpoint.yaw   = _rates_setpoint(2);
				_thrust_setpoint.copyTo(vehicle_rates_setpoint.thrust_body);
				vehicle_rates_setpoint.timestamp = hrt_absolute_time();
				_vehicle_rates_setpoint_pub.publish(vehicle_rates_setpoint);


				publishThrustSetpoint(manual_control_setpoint.timestamp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s vehicle_rates_setpoint;

			if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
				_rates_setpoint(0) = vehicle_rates_setpoint.roll;
				_rates_setpoint(1) = vehicle_rates_setpoint.pitch;
				_rates_setpoint(2) = vehicle_rates_setpoint.yaw;
				_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);

				publishThrustSetpoint(vehicle_rates_setpoint.timestamp);
			}
		}

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((angular_velocity.timestamp_sample - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = angular_velocity.timestamp_sample;

		// reset integral if disarmed
		if (!_armed || !_rotary_wing) {
			_rate_control.resetIntegral();
		}

		// run rate controller
		const Vector3f torque_sp{_rate_control.update(Vector3f(angular_velocity.xyz), _rates_setpoint, Vector3f(angular_acceleration.xyz), dt, _landed)};

		publishTorqueSetpoint(angular_velocity.timestamp_sample, torque_sp);
		publishRateControllerStatus(angular_velocity.timestamp_sample);

		publishActuatorControls(angular_velocity.timestamp_sample, torque_sp, dt);
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	if (!_vtol) {
		vehicle_thrust_setpoint_s vehicle_thrust_setpoint;
		vehicle_thrust_setpoint.timestamp_sample = timestamp_sample;
		_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
		vehicle_thrust_setpoint.timestamp = hrt_absolute_time();

		_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);
	}
}

void MulticopterRateControl::publishTorqueSetpoint(const hrt_abstime &timestamp_sample, const Vector3f &torque_sp)
{
	if (!_vtol) {
		vehicle_torque_setpoint_s vehicle_torque_setpoint;
		vehicle_torque_setpoint.timestamp_sample = timestamp_sample;
		torque_sp.copyTo(vehicle_torque_setpoint.xyz);
		vehicle_torque_setpoint.timestamp = hrt_absolute_time();

		_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);
	}
}

void MulticopterRateControl::publishRateControllerStatus(const hrt_abstime &timestamp_sample)
{
	rate_ctrl_status_s rate_ctrl_status;
	rate_ctrl_status.timestamp_sample  = timestamp_sample;
	rate_ctrl_status.rollspeed_integ   = _rate_control.getIntegral()(0);
	rate_ctrl_status.pitchspeed_integ  = _rate_control.getIntegral()(1);
	rate_ctrl_status.yawspeed_integ    = _rate_control.getIntegral()(2);
	rate_ctrl_status.additional_integ1 = 0;
	rate_ctrl_status.timestamp         = hrt_absolute_time();
	_controller_status_pub.publish(rate_ctrl_status);
}

void MulticopterRateControl::publishActuatorControls(const hrt_abstime &timestamp_sample, const Vector3f &torque_sp,
		float dt)
{
	// scale effort by battery status if enabled
	if (_param_mc_bat_scale_en.get() && _battery_status_sub.updated()) {
		battery_status_s battery_status;

		if (_battery_status_sub.copy(&battery_status)) {
			if (battery_status.connected && (battery_status.scale > 0.f)) {
				_battery_status_scale = battery_status.scale;
			}
		}
	}

	{
		// landing gear passed through actuator controls
		landing_gear_s landing_gear;

		if (_landing_gear_sub.update(&landing_gear)) {
			if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
				if (landing_gear.landing_gear == landing_gear_s::GEAR_UP && _landed) {
					mavlink_log_critical(&_mavlink_log_pub, "Landed, unable to retract landing gear\t");
					events::send(events::ID("mc_rate_control_not_retract_landing_gear_landed"),
					{events::Log::Error, events::LogInternal::Info},
					"Landed, unable to retract landing gear");

				} else {
					_landing_gear = landing_gear.landing_gear;
				}
			}
		}
	}

	// publish actuator controls
	actuator_controls_s actuators{};
	actuators.timestamp_sample = timestamp_sample;
	actuators.control[actuator_controls_s::INDEX_ROLL]  = torque_sp(0) * _battery_status_scale;
	actuators.control[actuator_controls_s::INDEX_PITCH] = torque_sp(1) * _battery_status_scale;
	actuators.control[actuator_controls_s::INDEX_YAW]   = torque_sp(2) * _battery_status_scale;
	actuators.control[actuator_controls_s::INDEX_THROTTLE] = -_thrust_setpoint(2) * _battery_status_scale;
	actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
	actuators.timestamp = hrt_absolute_time();
	_actuator_controls_0_pub.publish(actuators);

	updateActuatorControlsStatus(actuators, dt);
}

void MulticopterRateControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

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

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
