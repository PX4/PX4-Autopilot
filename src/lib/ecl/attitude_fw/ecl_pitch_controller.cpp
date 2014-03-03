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

ECL_PitchController::ECL_PitchController() :
	_last_run(0),
	_last_output(0.0f),
	_integrator(0.0f),
	_rate_error(0.0f),
	_rate_setpoint(0.0f),
	_max_deflection_rad(math::radians(45.0f))
{
}

float ECL_PitchController::control(float pitch_setpoint, float pitch, float pitch_rate, float roll, float scaler,
				   bool lock_integrator, float airspeed_min, float airspeed_max, float airspeed)
{
	/* get the usual dt estimate */
	uint64_t dt_micros = ecl_elapsed_time(&_last_run);
	_last_run = ecl_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	if (dt_micros > 500000)
		lock_integrator = true;

	float k_roll_ff = math::max((_k_p - _k_i * _tc) * _tc - _k_d, 0.0f);
	float k_i_rate = _k_i * _tc;

	/* input conditioning */
	if (!isfinite(airspeed)) {
		/* airspeed is NaN, +- INF or not available, pick center of band */
		airspeed = 0.5f * (airspeed_min + airspeed_max);
	} else if (airspeed < airspeed_min) {
		airspeed = airspeed_min;
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
	float turn_offset = fabsf((CONSTANTS_ONE_G / airspeed) *
				tanf(roll) * sinf(roll)) * _roll_ff;
	if (inverted)
		turn_offset = -turn_offset;

	float pitch_error = pitch_setpoint - pitch;
	/* rate setpoint from current error and time constant */
	_rate_setpoint = pitch_error / _tc;

	/* add turn offset */
	_rate_setpoint += turn_offset;

	_rate_error = _rate_setpoint - pitch_rate;

	float ilimit_scaled = _integrator_max * scaler;

	if (!lock_integrator && k_i_rate > 0.0f && airspeed > 0.5f * airspeed_min) {

		float id = _rate_error * k_i_rate * dt * scaler;

		/*
		 * anti-windup: do not allow integrator to increase into the
		 * wrong direction if actuator is at limit
		 */
		if (_last_output < -_max_deflection_rad) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);
		} else if (_last_output > _max_deflection_rad) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		_integrator += id;
	}

	/* integrator limit */
	_integrator = math::constrain(_integrator, -ilimit_scaled, ilimit_scaled);
	/* store non-limited output */
	_last_output = ((_rate_error * _k_d * scaler) + _integrator + (_rate_setpoint * k_roll_ff)) * scaler;

	return math::constrain(_last_output, -_max_deflection_rad, _max_deflection_rad);
}

void ECL_PitchController::reset_integrator()
{
	_integrator = 0.0f;
}
