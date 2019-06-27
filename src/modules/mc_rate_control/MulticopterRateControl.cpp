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
 * @file mc_rate_control_main.cpp
 * Multicopter rate controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

using namespace matrix;

MulticopterRateControl::MulticopterRateControl() :
	ModuleParams(nullptr),
	WorkItem(px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_rate_control"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	_rates_prev.zero();
	_rates_prev_filtered.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	parameters_updated();
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.register_callback()) {
		PX4_ERR("vehicle angular velocity callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate gains
	_rate_p = Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get());
	_rate_i = Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get());
	_rate_int_lim = Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get());
	_rate_d = Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get());
	_rate_ff = Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get());

	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	_rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	if (fabsf(_lp_filters_d.get_cutoff_freq() - _param_mc_dterm_cutoff.get()) > 0.01f) {
		_lp_filters_d.set_cutoff_frequency(_loop_update_rate_hz, _param_mc_dterm_cutoff.get());
		_lp_filters_d.reset(_rates_prev);
	}

	// manual rate control acro mode rate limits
	using math::radians;
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);
}

void
MulticopterRateControl::parameter_update_poll()
{
	/* Check if parameters have changed */
	parameter_update_s param_update;

	if (_params_sub.update(&param_update)) {
		updateParams();
		parameters_updated();
	}
}

void
MulticopterRateControl::vehicle_status_poll()
{
	/* check if there is new status information */
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_actuators_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterRateControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	multirotor_motor_limits_s motor_limits{};

	if (_motor_limits_sub.update(&motor_limits)) {
		_saturation_status.value = motor_limits.saturation_status;
	}
}

float
MulticopterRateControl::get_landing_gear_state()
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

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Vector3f
MulticopterRateControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_thrust_sp) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	Vector3f pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterRateControl::control_attitude_rates(float dt, const Vector3f &rates)
{
	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_rates_int.zero();
	}

	Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(_param_mc_tpa_break_p.get(), _param_mc_tpa_rate_p.get()));
	Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(_param_mc_tpa_break_i.get(), _param_mc_tpa_rate_i.get()));
	Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(_param_mc_tpa_break_d.get(), _param_mc_tpa_rate_d.get()));

	/* angular rates error */
	Vector3f rates_err = _rates_sp - rates;

	/* apply low-pass filtering to the rates for D-term */
	Vector3f rates_filtered(_lp_filters_d.apply(rates));

	_att_control = _rate_k.emult(rates_p_scaled.emult(rates_err) +
				     _rates_int -
				     rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt) +
		       _rate_ff.emult(_rates_sp);

	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* update integral only if we are not landed */
	if (!_vehicle_land_detected.maybe_landed && !_vehicle_land_detected.landed) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// I term factor: reduce the I gain with increasing rate error.
			// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
			// change (noticeable in a bounce-back effect after a flip).
			// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
			// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
			// and up to 200 deg error leads to <25% reduction of I.
			float i_factor = rates_err(i) / math::radians(400.f);
			i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

			// Perform the integration using a first order method and do not propagate the result if out of range or invalid
			float rate_i = _rates_int(i) + i_factor * rates_i_scaled(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));
	}
}

void
MulticopterRateControl::publish_rate_controller_status()
{
	rate_ctrl_status_s rate_ctrl_status = {};
	rate_ctrl_status.timestamp = hrt_absolute_time();
	rate_ctrl_status.rollspeed_integ = _rates_int(0);
	rate_ctrl_status.pitchspeed_integ = _rates_int(1);
	rate_ctrl_status.yawspeed_integ = _rates_int(2);

	_controller_status_pub.publish(rate_ctrl_status);
}

void
MulticopterRateControl::publish_actuator_controls()
{
	_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
	_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
	_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
	_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
	_actuators.control[7] = (float)_landing_gear.landing_gear;
	_actuators.timestamp = hrt_absolute_time();

	/* scale effort by battery status */
	_battery_status_sub.update(&_battery_status);

	if (_param_mc_bat_scale_en.get() && _battery_status.scale > 0.0f) {
		for (int i = 0; i < 4; i++) {
			_actuators.control[i] *= _battery_status.scale;
		}
	}

	if (!_actuators_0_circuit_breaker_enabled) {
		orb_publish_auto(_actuators_id, &_actuators_0_pub, &_actuators, nullptr, ORB_PRIO_DEFAULT);
	}
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregister_callback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	parameter_update_poll();

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = hrt_absolute_time();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.rollspeed, angular_velocity.pitchspeed, angular_velocity.yawspeed};

		_actuators.timestamp_sample = angular_velocity.timestamp_sample;

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		vehicle_status_poll();
		vehicle_motor_limits_poll();

		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;

		// landing gear controlled from stick inputs if we are in Manual/Stabilized mode
		if (_v_control_mode.flag_control_manual_enabled &&
		    !_v_control_mode.flag_control_altitude_enabled &&
		    !_v_control_mode.flag_control_velocity_enabled &&
		    !_v_control_mode.flag_control_position_enabled) {

			_landing_gear.landing_gear = get_landing_gear_state();
			_landing_gear.timestamp = hrt_absolute_time();
			_landing_gear_pub.publish(_landing_gear);

		} else {
			_landing_gear_sub.update(&_landing_gear);
		}

		/* attitude controller disabled, poll rates setpoint topic */
		if (_v_control_mode.flag_control_manual_enabled && is_hovering) {
			if (_manual_control_sp_sub.update(&_manual_control_sp)) {
				/* manual rates control - ACRO mode */
				Vector3f man_rate_sp(
					math::superexpo(_manual_control_sp.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-_manual_control_sp.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(_manual_control_sp.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get()));

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = _manual_control_sp.z;

				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			vehicle_rates_setpoint_s v_rates_sp{};

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = v_rates_sp.roll;
				_rates_sp(1) = v_rates_sp.pitch;
				_rates_sp(2) = v_rates_sp.yaw;
				_thrust_sp = -v_rates_sp.thrust_body[2];
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
				_lp_filters_d.set_cutoff_frequency(_loop_update_rate_hz, _param_mc_dterm_cutoff.get());
			}
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled) {
			control_attitude_rates(dt, rates);
			publish_rate_controller_status();
			publish_actuator_controls();

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				_rates_sp.zero();
				_rates_int.zero();
				_thrust_sp = 0.0f;
				_att_control.zero();
				publish_actuator_controls();
			}
		}
	}

	perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	MulticopterRateControl *instance = new MulticopterRateControl();

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

int MulticopterRateControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	return 0;
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

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Multicopter rate control app start / stop handling function
 */
extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[]);

int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
