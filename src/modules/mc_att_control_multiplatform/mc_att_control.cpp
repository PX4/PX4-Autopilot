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

MulticopterAttitudeControl::MulticopterAttitudeControl() :
	MulticopterAttitudeControlBase(),
	_task_should_exit(false),
	_actuators_0_circuit_breaker_enabled(false),

	/* publications */
	_att_sp_pub(nullptr),
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_n(),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))

{
	_params_handles.roll_p			= 	PX4_PARAM_INIT(MC_ROLL_P);
	_params_handles.roll_rate_p		= 	PX4_PARAM_INIT(MC_ROLLRATE_P);
	_params_handles.roll_rate_i		= 	PX4_PARAM_INIT(MC_ROLLRATE_I);
	_params_handles.roll_rate_d		= 	PX4_PARAM_INIT(MC_ROLLRATE_D);
	_params_handles.pitch_p			= 	PX4_PARAM_INIT(MC_PITCH_P);
	_params_handles.pitch_rate_p		= 	PX4_PARAM_INIT(MC_PITCHRATE_P);
	_params_handles.pitch_rate_i		= 	PX4_PARAM_INIT(MC_PITCHRATE_I);
	_params_handles.pitch_rate_d		= 	PX4_PARAM_INIT(MC_PITCHRATE_D);
	_params_handles.yaw_p			=	PX4_PARAM_INIT(MC_YAW_P);
	_params_handles.yaw_rate_p		= 	PX4_PARAM_INIT(MC_YAWRATE_P);
	_params_handles.yaw_rate_i		= 	PX4_PARAM_INIT(MC_YAWRATE_I);
	_params_handles.yaw_rate_d		= 	PX4_PARAM_INIT(MC_YAWRATE_D);
	_params_handles.yaw_ff			= 	PX4_PARAM_INIT(MC_YAW_FF);
	_params_handles.yaw_rate_max		= 	PX4_PARAM_INIT(MC_YAWRATE_MAX);
	_params_handles.man_roll_max		= 	PX4_PARAM_INIT(MC_MAN_R_MAX);
	_params_handles.man_pitch_max		= 	PX4_PARAM_INIT(MC_MAN_P_MAX);
	_params_handles.man_yaw_max		= 	PX4_PARAM_INIT(MC_MAN_Y_MAX);
	_params_handles.acro_roll_max		= 	PX4_PARAM_INIT(MC_ACRO_R_MAX);
	_params_handles.acro_pitch_max		= 	PX4_PARAM_INIT(MC_ACRO_P_MAX);
	_params_handles.acro_yaw_max		= 	PX4_PARAM_INIT(MC_ACRO_Y_MAX);

	/* fetch initial parameter values */
	parameters_update();

	/*
	 * do subscriptions
	 */
	_v_att = _n.subscribe<px4_vehicle_attitude>(&MulticopterAttitudeControl::handle_vehicle_attitude, this, 0);
	_v_att_sp = _n.subscribe<px4_vehicle_attitude_setpoint>(0);
	_v_rates_sp = _n.subscribe<px4_vehicle_rates_setpoint>(0);
	_v_control_mode = _n.subscribe<px4_vehicle_control_mode>(0);
	_parameter_update = _n.subscribe<px4_parameter_update>(
			&MulticopterAttitudeControl::handle_parameter_update, this, 1000);
	_manual_control_sp = _n.subscribe<px4_manual_control_setpoint>(0);
	_armed = _n.subscribe<px4_actuator_armed>(0);
	_v_status = _n.subscribe<px4_vehicle_status>(0);

}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
}

int
MulticopterAttitudeControl::parameters_update()
{
	float v;

	/* roll gains */
	PX4_PARAM_GET(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	PX4_PARAM_GET(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	PX4_PARAM_GET(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	PX4_PARAM_GET(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v;

	/* pitch gains */
	PX4_PARAM_GET(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	PX4_PARAM_GET(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	PX4_PARAM_GET(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	PX4_PARAM_GET(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v;

	/* yaw gains */
	PX4_PARAM_GET(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	PX4_PARAM_GET(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	PX4_PARAM_GET(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	PX4_PARAM_GET(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;

	PX4_PARAM_GET(_params_handles.yaw_ff, &_params.yaw_ff);
	PX4_PARAM_GET(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.yaw_rate_max = math::radians(_params.yaw_rate_max);

	/* manual control scale */
	PX4_PARAM_GET(_params_handles.man_roll_max, &_params.man_roll_max);
	PX4_PARAM_GET(_params_handles.man_pitch_max, &_params.man_pitch_max);
	PX4_PARAM_GET(_params_handles.man_yaw_max, &_params.man_yaw_max);
	_params.man_roll_max = math::radians(_params.man_roll_max);
	_params.man_pitch_max = math::radians(_params.man_pitch_max);
	_params.man_yaw_max = math::radians(_params.man_yaw_max);

	/* acro control scale */
	PX4_PARAM_GET(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	PX4_PARAM_GET(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	PX4_PARAM_GET(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void MulticopterAttitudeControl::handle_parameter_update(const px4_parameter_update &msg)
{
	parameters_update();
}

void  MulticopterAttitudeControl::handle_vehicle_attitude(const px4_vehicle_attitude &msg) {

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

		/* publish the attitude setpoint if needed */
		if (_publish_att_sp && _v_status->data().is_rotary_wing) {
			_v_att_sp_mod.data().timestamp = px4::get_time_micros();

			if (_att_sp_pub != nullptr) {
				_att_sp_pub->publish(_v_att_sp_mod);

			} else {
				_att_sp_pub = _n.advertise<px4_vehicle_attitude_setpoint>();
			}
		}

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

			/* reset yaw setpoint after ACRO */
			_reset_yaw_sp = true;

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
		_actuators.data().control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
		_actuators.data().control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
		_actuators.data().control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
		_actuators.data().control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
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
