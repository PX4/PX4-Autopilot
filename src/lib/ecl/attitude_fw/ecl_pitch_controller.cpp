/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
#include <math.h>
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>

ECL_PitchController::ECL_PitchController() :
	_last_run(0),
	_tc(0.1f),
	_k_p(0.0f),
	_k_i(0.0f),
	_k_ff(0.0f),
	_integrator_max(0.0f),
	_max_rate_pos(0.0f),
	_max_rate_neg(0.0f),
	_roll_ff(0.0f),
	_last_output(0.0f),
	_integrator(0.0f),
	_rate_error(0.0f),
	_rate_setpoint(0.0f),
	_bodyrate_setpoint(0.0f),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fw att control pitch nonfinite input"))
{
}

ECL_PitchController::~ECL_PitchController()
{
	perf_free(_nonfinite_input_perf);
}

float ECL_PitchController::control_attitude(float pitch_setpoint, float roll, float pitch, float airspeed)
{
	/* Do not calculate control signal with bad inputs */
	if (!(isfinite(pitch_setpoint) && isfinite(roll) && isfinite(pitch) && isfinite(airspeed))) {
		perf_count(_nonfinite_input_perf);
		warnx("not controlling pitch");
		return _rate_setpoint;
	}

	/* flying inverted (wings upside down) ? */
	bool inverted = false;

	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(roll) < math::radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		roll = math::constrain(roll, math::radians(-80.0f), math::radians(80.0f));
	} else {
		/* inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity */

		/* note: the ranges are extended by 10 deg here to avoid numeric resolution effects */
		if (roll > 0.0f) {
			/* right hemisphere */
			roll = math::constrain(roll, math::radians(100.0f), math::radians(180.0f));
		} else {
			/* left hemisphere */
			roll = math::constrain(roll, math::radians(-100.0f), math::radians(-180.0f));
		}
	}

	/* calculate the offset in the rate resulting from rolling  */
	//xxx needs explanation and conversion to body angular rates or should be removed
	float turn_offset = fabsf((CONSTANTS_ONE_G / airspeed) *
				tanf(roll) * sinf(roll)) * _roll_ff;
	if (inverted)
		turn_offset = -turn_offset;

	/* Calculate the error */
	float pitch_error = pitch_setpoint - pitch;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  pitch_error / _tc;

	/* add turn offset */
	_rate_setpoint += turn_offset;

	/* limit the rate */ //XXX: move to body angluar rates
	if (_max_rate_pos > 0.01f && _max_rate_neg > 0.01f) {
		if (_rate_setpoint > 0.0f) {
			_rate_setpoint = (_rate_setpoint > _max_rate_pos) ? _max_rate_pos : _rate_setpoint;
		} else {
			_rate_setpoint = (_rate_setpoint < -_max_rate_neg) ? -_max_rate_neg : _rate_setpoint;
		}

	}

	return _rate_setpoint;
}

float ECL_PitchController::control_bodyrate(float roll, float pitch,
		float pitch_rate, float yaw_rate,
		float yaw_rate_setpoint,
		float airspeed_min, float airspeed_max, float airspeed, float scaler, bool lock_integrator)
{
	/* Do not calculate control signal with bad inputs */
	if (!(isfinite(roll) && isfinite(pitch) && isfinite(pitch_rate) && isfinite(yaw_rate) &&
				isfinite(yaw_rate_setpoint) && isfinite(airspeed_min) &&
				isfinite(airspeed_max) && isfinite(scaler))) {
		perf_count(_nonfinite_input_perf);
		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* get the usual dt estimate */
	uint64_t dt_micros = ecl_elapsed_time(&_last_run);
	_last_run = ecl_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	if (dt_micros > 500000)
		lock_integrator = true;

//	float k_ff = math::max((_k_p - _k_i * _tc) * _tc - _k_d, 0.0f);
	float k_ff = 0;

	/* input conditioning */
	if (!isfinite(airspeed)) {
		/* airspeed is NaN, +- INF or not available, pick center of band */
		airspeed = 0.5f * (airspeed_min + airspeed_max);
	} else if (airspeed < airspeed_min) {
		airspeed = airspeed_min;
	}

	/* Transform setpoint to body angular rates */
	_bodyrate_setpoint = cosf(roll) * _rate_setpoint + cosf(pitch) * sinf(roll) * yaw_rate_setpoint; //jacobian

	/* Transform estimation to body angular rates */
	float pitch_bodyrate = cosf(roll) * pitch_rate + cosf(pitch) * sinf(roll) * yaw_rate; //jacobian

	_rate_error = _bodyrate_setpoint - pitch_bodyrate;

	if (!lock_integrator && _k_i > 0.0f && airspeed > 0.5f * airspeed_min) {

		float id = _rate_error * dt;

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

		_integrator += id;
	}

	/* integrator limit */
	//xxx: until start detection is available: integral part in control signal is limited here
	float integrator_constrained = math::constrain(_integrator * _k_i, -_integrator_max, _integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = (_bodyrate_setpoint * _k_ff +_rate_error * _k_p + integrator_constrained) * scaler * scaler;  //scaler is proportional to 1/airspeed
//	warnx("pitch: _integrator: %.4f, _integrator_max: %.4f, airspeed %.4f, _k_i %.4f, _k_p: %.4f", (double)_integrator, (double)_integrator_max, (double)airspeed, (double)_k_i, (double)_k_p);
//	warnx("roll: _last_output %.4f", (double)_last_output);
	return math::constrain(_last_output, -1.0f, 1.0f);
}

void ECL_PitchController::reset_integrator()
{
	_integrator = 0.0f;
}
