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
 * @file ecl_yaw_controller.cpp
 * Implementation of a simple orthogonal coordinated turn yaw PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_yaw_controller.h"
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>

ECL_YawController::ECL_YawController() :
	_last_run(0),
	_tc(0.1f),
	_last_output(0.0f),
	_integrator(0.0f),
	_rate_error(0.0f),
	_rate_setpoint(0.0f),
	_bodyrate_setpoint(0.0f),
	_max_deflection_rad(math::radians(45.0f)),
	_coordinated(1.0f)

{

}

float ECL_YawController::control_attitude(float roll, float pitch,
		float speed_body_u, float speed_body_w,
		float roll_rate_setpoint, float pitch_rate_setpoint)
{
//	static int counter = 0;
	/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
	_rate_setpoint = 0.0f;
	if (_coordinated > 0.1) {
		float denumerator = (speed_body_u * cosf(roll) * cosf(pitch) + speed_body_w * sinf(pitch));
		if(denumerator != 0.0f) { //XXX: floating point comparison
			_rate_setpoint = (speed_body_w * roll_rate_setpoint + 9.81f * sinf(roll) * cosf(pitch) + speed_body_u * pitch_rate_setpoint * sinf(roll)) / denumerator;
		}

//		if(counter % 20 == 0) {
//			warnx("denumerator: %.4f, speed_body_u: %.4f, speed_body_w: %.4f, cosf(roll): %.4f, cosf(pitch): %.4f, sinf(pitch): %.4f", (double)denumerator, (double)speed_body_u, (double)speed_body_w, (double)cosf(roll), (double)cosf(pitch), (double)sinf(pitch));
//		}
	}

	/* limit the rate */ //XXX: move to body angluar rates
	if (_max_rate > 0.01f) {
	_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;
	_rate_setpoint = (_rate_setpoint < -_max_rate) ? -_max_rate : _rate_setpoint;
	}


//	counter++;

	return _rate_setpoint;
}

float ECL_YawController::control_bodyrate(float roll, float pitch,
		float pitch_rate, float yaw_rate,
		float pitch_rate_setpoint,
		float airspeed_min, float airspeed_max, float airspeed, float scaler, bool lock_integrator)
{
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
	_bodyrate_setpoint = -sinf(roll) * pitch_rate_setpoint * cosf(roll)*cosf(pitch) * _rate_setpoint; //jacobian

	/* Transform estimation to body angular rates */
	float yaw_bodyrate = -sinf(roll) * pitch_rate * cosf(roll)*cosf(pitch) * yaw_rate; //jacobian

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - yaw_bodyrate; //body angular rate error

	if (!lock_integrator && _k_i > 0.0f && airspeed > 0.5f * airspeed_min) {

	float id = _rate_error * dt;

	/*
	 * anti-windup: do not allow integrator to increase if actuator is at limit
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
	_integrator = math::constrain(_integrator, -_integrator_max, _integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = (_rate_error * _k_p + _integrator * _k_i * _rate_setpoint * k_ff) * scaler * scaler;  //scaler^2 is proportional to 1/airspeed^2

	return math::constrain(_last_output, -_max_deflection_rad, _max_deflection_rad);
}

void ECL_YawController::reset_integrator()
{
	_integrator = 0.0f;
}
