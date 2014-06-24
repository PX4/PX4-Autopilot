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
 * @file ecl_roll_controller.cpp
 * Implementation of a simple orthogonal roll PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "../ecl.h"
#include "ecl_roll_controller.h"
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>

ECL_RollController::ECL_RollController() :
	_last_run(0),
	_tc(0.1f),
	_k_p(0.0f),
	_k_i(0.0f),
	_k_ff(0.0f),
	_integrator_max(0.0f),
	_max_rate(0.0f),
	_last_output(0.0f),
	_integrator(0.0f),
	_rate_error(0.0f),
	_rate_setpoint(0.0f),
	_bodyrate_setpoint(0.0f),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fw att control roll nonfinite input"))
{
}

ECL_RollController::~ECL_RollController()
{
	perf_free(_nonfinite_input_perf);
}

float ECL_RollController::control_attitude(float roll_setpoint, float roll)
{
	/* Do not calculate control signal with bad inputs */
	if (!(isfinite(roll_setpoint) && isfinite(roll))) {
		perf_count(_nonfinite_input_perf);
		return _rate_setpoint;
	}

	/* Calculate error */
	float roll_error = roll_setpoint - roll;

	/* Apply P controller */
	_rate_setpoint = roll_error / _tc;

	/* limit the rate */ //XXX: move to body angluar rates
	if (_max_rate > 0.01f) {
		_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;
		_rate_setpoint = (_rate_setpoint < -_max_rate) ? -_max_rate : _rate_setpoint;
	}

	return _rate_setpoint;
}

float ECL_RollController::control_bodyrate(float pitch,
		float roll_rate, float yaw_rate,
		float yaw_rate_setpoint,
		float airspeed_min, float airspeed_max, float airspeed, float scaler, bool lock_integrator)
{
	/* Do not calculate control signal with bad inputs */
	if (!(isfinite(pitch) && isfinite(roll_rate) && isfinite(yaw_rate) && isfinite(yaw_rate_setpoint) &&
			isfinite(airspeed_min) && isfinite(airspeed_max) &&
			isfinite(scaler))) {
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
	float k_ff = 0; //xxx: param

	/* input conditioning */
//	warnx("airspeed pre %.4f", (double)airspeed);
	if (!isfinite(airspeed)) {
		/* airspeed is NaN, +- INF or not available, pick center of band */
		airspeed = 0.5f * (airspeed_min + airspeed_max);
	} else if (airspeed < airspeed_min) {
		airspeed = airspeed_min;
	}


	/* Transform setpoint to body angular rates */
	_bodyrate_setpoint = _rate_setpoint - sinf(pitch) * yaw_rate_setpoint; //jacobian

	/* Transform estimation to body angular rates */
	float roll_bodyrate = roll_rate - sinf(pitch) * yaw_rate; //jacobian

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - roll_bodyrate; //body angular rate error

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
	//warnx("roll: _integrator: %.4f, _integrator_max: %.4f", (double)_integrator, (double)_integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = (_bodyrate_setpoint * _k_ff + _rate_error * _k_p + integrator_constrained) * scaler * scaler;  //scaler is proportional to 1/airspeed

	return math::constrain(_last_output, -1.0f, 1.0f);
}

void ECL_RollController::reset_integrator()
{
	_integrator = 0.0f;
}

