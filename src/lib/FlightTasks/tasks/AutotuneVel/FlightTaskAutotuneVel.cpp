/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutotunevel.cpp
 */

#include "FlightTaskAutotuneVel.hpp"
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

bool FlightTaskAutotuneVel::initializeSubscriptions(SubscriptionArray &subscription_array)
{
	if (!FlightTaskManual::initializeSubscriptions(subscription_array)) {
		return false;
	}

	return true;
}

bool FlightTaskAutotuneVel::updateInitialize()
{
	bool ret = FlightTaskManual::updateInitialize();
	// require valid position / velocity in xy
	return ret && PX4_ISFINITE(_position(0))
	       && PX4_ISFINITE(_position(1))
	       && PX4_ISFINITE(_velocity(0))
	       && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskAutotuneVel::activate()
{
	bool ret = FlightTaskManual::activate();
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = 0.0f;
	_thrust_setpoint = matrix::Vector3f(0.0f, 0.0f, NAN);
	_current_position_xy = matrix::Vector2f(_position);
	_position_setpoint(2) = _position(2);
	_velocity_setpoint(2) = 0.0f;
	_setDefaultConstraints();

	_ku = 1.5f;
	_period_u = 0.f;
	_convergence_counter = 0;
	_peak_counter = 0;
	_peak_counter_state = SignalState::unknown;
	_done = false;

	return ret;
}

bool FlightTaskAutotuneVel::_checkSticks()
{
	// Abort if the user moves the sticks
	if (Vector3f(&_sticks(0)).length() > 0.2f) {
		return true;

	} else {
		return false;
	}
}

void FlightTaskAutotuneVel::_updateSetpoints()
{
	_thrust = -_velocity(0) * _ku;

	// Saturation block
	if (_thrust > _thrust_max) {
		_thrust_sat = _thrust_max;

	} else if (_thrust < -_thrust_max) {
		_thrust_sat = -_thrust_max;

	} else {
		_thrust_sat = _thrust;
	}

	// Loose position controller
	const matrix::Vector2f pos_control_xy = (_current_position_xy - matrix::Vector2f(_position)) * 0.002f;

	_thrust_setpoint(0) = _thrust_sat + pos_control_xy(0);
	_thrust_setpoint(1) = 0.f + pos_control_xy(1);
	_thrust_setpoint(2) = NAN;
	_constraints.rescale_xy_thrust = false;
}

void FlightTaskAutotuneVel::_updateUltimateGain()
{
	// epsilon << alpha
	float ku_dot = -_param_mpc_atune_gain.get() * fabsf(_thrust - _thrust_sat) + _epsilon;
	_ku += ku_dot;

	_jerk_setpoint(0) = _ku;
	if (fabs(ku_dot) < _param_mpc_atune_thr.get() && _ku < 1.4f) {
		_convergence_counter++;

	} else {
		_convergence_counter = 0;
	}
}

void FlightTaskAutotuneVel::_measureUltimatePeriod()
{
	switch (_peak_counter_state) {
	case SignalState::unknown:
		if (_thrust > 0.9f * _thrust_max) {
			_peak_counter_state = SignalState::high;
			_start_time = hrt_absolute_time();

		} else if (_thrust < -0.9f * _thrust_max) {
			_peak_counter_state = SignalState::low;
			_start_time = hrt_absolute_time();
		}

		break;

	case SignalState::high:
		if (_thrust < -0.9f * _thrust_max) {
			_peak_counter_state = SignalState::low;
			_peak_counter++;
		}

		break;

	case SignalState::low:
		if (_thrust > 0.9f * _thrust_max) {
			_peak_counter_state = SignalState::high;
			_peak_counter++;
		}

		break;
	}

	if (_peak_counter == 11) {
		// 5 periods
		_period_u = hrt_elapsed_time(&_start_time) * 2e-7;
	}
}

void FlightTaskAutotuneVel::_computeControlGains()
{
	// Compute Kp, Ki and Kd using robust Ziegler-Nichols rules
	float kp = 0.3375f * _ku;
	float ki = 0.3f * _ku / _period_u;
	float kd = 0.0375f * _ku * _period_u;

	if (kp > 0.f && kp < 1.f && ki > 0.f && ki < 4.f && kd > 0.f && kd < 0.1f) {
		_param_mpc_xy_vel_p.set(kp);
		_param_mpc_xy_vel_p.commit();
		_param_mpc_xy_vel_i.set(ki);
		_param_mpc_xy_vel_i.commit();
		_param_mpc_xy_vel_d.set(kd);
		_param_mpc_xy_vel_d.commit();

	} else {
		// Failed
	}
}

void FlightTaskAutotuneVel::_exit()
{
	_param_mpc_xy_atune.set(false);
	_param_mpc_xy_atune.commit();
	_done = true;
}

bool FlightTaskAutotuneVel::update()
{
	if (_done) {
		return false;
	}

	if (_checkSticks()) {
		_exit();
		return false;
	}

	_updateSetpoints();

	if (_convergence_counter < _param_mpc_atune_cnt.get()) {
		_updateUltimateGain();

	} else {
		_measureUltimatePeriod();
	}

	if (_peak_counter >= 11) {
		// Ultimate gain found, start counting peaks
		_computeControlGains();
		// Done, exit flight task
		_exit();
		return false;
	}

	return true;
}
