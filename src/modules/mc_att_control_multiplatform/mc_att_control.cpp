/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control.cpp
 * Multicopter attitude controller.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Roman Bapst <bapstr@ethz.ch>
 */

#include "mc_att_control.h"
#include "mc_att_control_params.h"
#include "math.h"

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f

namespace mc_att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

}

MulticopterAttitudeControlMultiplatform::MulticopterAttitudeControlMultiplatform() :
	MulticopterAttitudeControlBase(),
	_actuators_0_circuit_breaker_enabled(false),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_n(_appState),

	/* parameters */
	_params_handles({
			.roll_p		= px4::ParameterFloat("MP_ROLL_P", PARAM_MP_ROLL_P_DEFAULT),
			.roll_rate_p	= px4::ParameterFloat("MP_ROLLRATE_P", PARAM_MP_ROLLRATE_P_DEFAULT),
			.roll_rate_i	= px4::ParameterFloat("MP_ROLLRATE_I", PARAM_MP_ROLLRATE_I_DEFAULT),
			.roll_rate_d	= px4::ParameterFloat("MP_ROLLRATE_D", PARAM_MP_ROLLRATE_D_DEFAULT),
			.pitch_p	= px4::ParameterFloat("MP_PITCH_P", PARAM_MP_PITCH_P_DEFAULT),
			.pitch_rate_p	= px4::ParameterFloat("MP_PITCHRATE_P", PARAM_MP_PITCHRATE_P_DEFAULT),
			.pitch_rate_i	= px4::ParameterFloat("MP_PITCHRATE_I", PARAM_MP_PITCHRATE_I_DEFAULT),
			.pitch_rate_d	= px4::ParameterFloat("MP_PITCHRATE_D", PARAM_MP_PITCHRATE_D_DEFAULT),
			.yaw_p		= px4::ParameterFloat("MP_YAW_P", PARAM_MP_YAW_P_DEFAULT),
			.yaw_rate_p	= px4::ParameterFloat("MP_YAWRATE_P", PARAM_MP_YAWRATE_P_DEFAULT),
			.yaw_rate_i	= px4::ParameterFloat("MP_YAWRATE_I", PARAM_MP_YAWRATE_I_DEFAULT),
			.yaw_rate_d	= px4::ParameterFloat("MP_YAWRATE_D", PARAM_MP_YAWRATE_D_DEFAULT),
			.yaw_ff		= px4::ParameterFloat("MP_YAW_FF", PARAM_MP_YAW_FF_DEFAULT),
			.yaw_rate_max	= px4::ParameterFloat("MP_YAWRATE_MAX", PARAM_MP_YAWRATE_MAX_DEFAULT),
			.acro_roll_max	= px4::ParameterFloat("MP_ACRO_R_MAX", PARAM_MP_ACRO_R_MAX_DEFAULT),
			.acro_pitch_max	= px4::ParameterFloat("MP_ACRO_P_MAX", PARAM_MP_ACRO_P_MAX_DEFAULT),
			.acro_yaw_max	= px4::ParameterFloat("MP_ACRO_Y_MAX", PARAM_MP_ACRO_Y_MAX_DEFAULT)
	}),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))

{
	/* fetch initial parameter values */
	parameters_update();

	/*
	 * do subscriptions
	 */
	_v_att = _n.subscribe<px4_vehicle_attitude>(&MulticopterAttitudeControlMultiplatform::handle_vehicle_attitude, this, 0);
	_v_att_sp = _n.subscribe<px4_vehicle_attitude_setpoint>(0);
	_v_rates_sp = _n.subscribe<px4_vehicle_rates_setpoint>(0);
	_v_control_mode = _n.subscribe<px4_vehicle_control_mode>(0);
	_parameter_update = _n.subscribe<px4_parameter_update>(
			&MulticopterAttitudeControlMultiplatform::handle_parameter_update, this, 1000);
	_manual_control_sp = _n.subscribe<px4_manual_control_setpoint>(0);
	_armed = _n.subscribe<px4_actuator_armed>(0);
	_v_status = _n.subscribe<px4_vehicle_status>(0);
}

MulticopterAttitudeControlMultiplatform::~MulticopterAttitudeControlMultiplatform()
{
}

int
MulticopterAttitudeControlMultiplatform::parameters_update()
{
	/* roll gains */
	_params.att_p(0) = _params_handles.roll_p.update();
	_params.rate_p(0) = _params_handles.roll_rate_p.update();
	_params.rate_i(0) = _params_handles.roll_rate_i.update();
	_params.rate_d(0) = _params_handles.roll_rate_d.update();

	/* pitch gains */
	_params.att_p(1) = _params_handles.pitch_p.update();
	_params.rate_p(1) = _params_handles.pitch_rate_p.update();
	_params.rate_i(1) = _params_handles.pitch_rate_i.update();
	_params.rate_d(1) = _params_handles.pitch_rate_d.update();

	/* yaw gains */
	_params.att_p(2) = _params_handles.yaw_p.update();
	_params.rate_p(2) = _params_handles.yaw_rate_p.update();
	_params.rate_i(2) = _params_handles.yaw_rate_i.update();
	_params.rate_d(2) = _params_handles.yaw_rate_d.update();

	_params.yaw_ff = _params_handles.yaw_ff.update();
	_params.yaw_rate_max = math::radians(_params_handles.yaw_rate_max.update());

	/* acro control scale */
	_params.acro_rate_max(0) = math::radians(_params_handles.acro_roll_max.update());
	_params.acro_rate_max(1) = math::radians(_params_handles.acro_pitch_max.update());
	_params.acro_rate_max(2) = math::radians(_params_handles.acro_yaw_max.update());

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void MulticopterAttitudeControlMultiplatform::handle_parameter_update(const px4_parameter_update &msg)
{
	parameters_update();
}

void  MulticopterAttitudeControlMultiplatform::handle_vehicle_attitude(const px4_vehicle_attitude &msg) {

	perf_begin(_loop_perf);

	/* run controller on attitude changes */
	static uint64_t last_run = 0;
	float dt = (px4::get_time_micros() - last_run) / 1000000.0f;
	last_run = px4::get_time_micros();

	/* guard against too small (< 2ms) and too large (> 20ms) dt's */
	if (dt < 0.002f) {
		dt = 0.002f;

	} else if (dt > 0.02f) {
		dt = 0.02f;
	}

	if (_v_control_mode->data().flag_control_attitude_enabled) {
		control_attitude(dt);

		/* publish attitude rates setpoint */
		_v_rates_sp_mod.data().roll = _rates_sp(0);
		_v_rates_sp_mod.data().pitch = _rates_sp(1);
		_v_rates_sp_mod.data().yaw = _rates_sp(2);
		_v_rates_sp_mod.data().thrust = _thrust_sp;
		_v_rates_sp_mod.data().timestamp = px4::get_time_micros();

		if (_v_rates_sp_pub != nullptr) {
			_v_rates_sp_pub->publish(_v_rates_sp_mod);
		} else {
			if (_v_status->data().is_vtol) {
				//XXX add a second publisher?
				// _v_rates_sp_pub = _n.advertise<px4_mc_virtual_rates_setpoint>();
			} else {
				_v_rates_sp_pub = _n.advertise<px4_vehicle_rates_setpoint>();
			}
		}

	} else {
		/* attitude controller disabled, poll rates setpoint topic */
		if (_v_control_mode->data().flag_control_manual_enabled) {
			/* manual rates control - ACRO mode */
			_rates_sp = math::Vector<3>(_manual_control_sp->data().y, -_manual_control_sp->data().x,
						    _manual_control_sp->data().r).emult(_params.acro_rate_max);
			_thrust_sp = _manual_control_sp->data().z;

			/* publish attitude rates setpoint */
			_v_rates_sp_mod.data().roll = _rates_sp(0);
			_v_rates_sp_mod.data().pitch = _rates_sp(1);
			_v_rates_sp_mod.data().yaw = _rates_sp(2);
			_v_rates_sp_mod.data().thrust = _thrust_sp;
			_v_rates_sp_mod.data().timestamp = px4::get_time_micros();

			if (_v_rates_sp_pub != nullptr) {
				_v_rates_sp_pub->publish(_v_rates_sp_mod);

			} else {
				if (_v_status->data().is_vtol) {
					//XXX add a second publisher?
					// _v_rates_sp_pub = _n.advertise<px4_mc_virtual_rates_setpoint>();
				} else {
					_v_rates_sp_pub = _n.advertise<px4_vehicle_rates_setpoint>();
				}
			}

		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			_rates_sp(0) = _v_rates_sp->data().roll;
			_rates_sp(1) = _v_rates_sp->data().pitch;
			_rates_sp(2) = _v_rates_sp->data().yaw;
			_thrust_sp = _v_rates_sp->data().thrust;
		}
	}

	if (_v_control_mode->data().flag_control_rates_enabled) {
		control_attitude_rates(dt);

		/* publish actuator controls */
		_actuators.data().control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
		_actuators.data().control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
		_actuators.data().control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
		_actuators.data().control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
		_actuators.data().timestamp = px4::get_time_micros();

		if (!_actuators_0_circuit_breaker_enabled) {
			if (_actuators_0_pub != nullptr) {
				_actuators_0_pub->publish(_actuators);

			} else {
				if (_v_status->data().is_vtol) {
					//XXX add a second publisher?
					// _actuators_0_pub = _n.advertise<px4_actuator_controls_virtual_mc>();
				} else {
					_actuators_0_pub = _n.advertise<px4_actuator_controls_0>();
				}
			}
		}
	}
}
