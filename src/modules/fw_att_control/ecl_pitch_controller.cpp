/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple orthogonal pitch PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_pitch_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

float ECL_PitchController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.airspeed))) {

		return _rate_setpoint;
	}

	/* Calculate the error */
	float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  pitch_error / _tc;

	return _rate_setpoint;
}

float ECL_PitchController::control_bodyrate(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_y_rate;

	if (!ctl_data.lock_integrator && _k_i > 0.0f) {

		/* Integral term scales with 1/IAS^2 */
		float id = _rate_error * dt * ctl_data.scaler * ctl_data.scaler;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	}

	// Calculate the contribution of the rate gyro to the actuator demand and high pass filter
	// The high pass filter reduces the contribution of the attitude control loop to the slew rate
	// metric and reduces the likelihood that a loss of stability caused by a too small value of
	// _tc would result in a an undesirable reduction in gyro rate feedback
	// Note: high pass filter is implemented as HPF_output = input - LPF_output
	const float p_term = _rate_error * _k_p * ctl_data.scaler * ctl_data.scaler;
	float filter_input;
	if (_k_ff > 0.0f) {
		// If feed forward is being used then we can monitor the rate error term
		// because the feed forward should be doing most of the work
		filter_input = p_term;
	} else {
		// Some users will use a zero feed forward gain and rely on the rate error term
		// to do all the work, resulting in large rate error tranisents. In this scenario
		// it is better to monitor the gyro feedback contribution only
		filter_input = ctl_data.body_y_rate * _k_p * ctl_data.scaler * ctl_data.scaler;
	}
	const float filt_tconst = _tc / 2.0f;
	const float filt_coef = fminf(dt, filt_tconst) / filt_tconst;
	_gyro_contribution_lpf = (1.0f - filt_coef) * _gyro_contribution_lpf + filt_coef * filter_input;
	const float gyro_term_hpf = filter_input - _gyro_contribution_lpf;

	// Calculate the actuator slew rate due to gyro feedback
	const float gyro_term_slew_rate = (gyro_term_hpf - _last_gyro_term_hpf) / dt;
	_last_gyro_term_hpf = gyro_term_hpf;

	/* calculate gain compression factor required to prevent the rate feedback */
	/*  term exceeding the actuator slew rate limit */
	if (_output_slew_rate_limit > 0.0f) {
		// apply a peak hold decaying envelope filter to the slew rate in the positive and negative direction
		const float decay_tconst = _tc * 2.0f;
		if (gyro_term_slew_rate > _max_pos_slew_rate) {
			_max_pos_slew_rate = gyro_term_slew_rate;
		} else {
			_max_pos_slew_rate *= (1.0f - fminf(dt, decay_tconst) / decay_tconst);
		}
		if (gyro_term_slew_rate < -_max_neg_slew_rate) {
			_max_neg_slew_rate = -gyro_term_slew_rate;
		} else {
			_max_neg_slew_rate *= (1.0f - fminf(dt, decay_tconst) / decay_tconst);
		}

		// calculate the peak slew rate of the oscillation using the smallest of the slew rate
		// in each direction to prevent single direction transients being detected as a limit cycle
		const float limit_cycle_slew_rate = fminf(_max_pos_slew_rate,_max_neg_slew_rate);
		if (limit_cycle_slew_rate > _output_slew_rate_limit) {
			// reduce gain inversely to the exceedance ratio but limit the lower value to prevent
			// possible loss of control due to vibration affecting the gyro signal
			_gain_compression_factor = fmaxf(_output_slew_rate_limit / limit_cycle_slew_rate, 0.25f);
		} else {
			_gain_compression_factor = 1.0f;
		}
	} else {
		_gain_compression_factor = 1.0f;
	}

	/* Apply PI rate controller and store non-limited output */
	/* FF terms scales with 1/TAS and P,I with 1/IAS^2 */
	_last_output = _bodyrate_setpoint * _k_ff * ctl_data.scaler + p_term * _gain_compression_factor + _integrator;

	return math::constrain(_last_output, -1.0f, 1.0f);
}

float ECL_PitchController::control_euler_rate(const float dt, const ECL_ControlData &ctl_data, float bodyrate_ff)
{
	/* Transform setpoint to body angular rates (jacobian) */
	_bodyrate_setpoint = cosf(ctl_data.roll) * _rate_setpoint +
			     cosf(ctl_data.pitch) * sinf(ctl_data.roll) * ctl_data.yaw_rate_setpoint + bodyrate_ff;

	set_bodyrate_setpoint(_bodyrate_setpoint);

	return control_bodyrate(dt, ctl_data);
}
