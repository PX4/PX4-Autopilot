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
 * Implementation of a simple orthogonal yaw PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_yaw_controller.h"
#include "fast_sincosf.h"
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
static perf_counter_t _nonfinite_input_perf = perf_alloc(PC_COUNT, "fw att control nonfinite input");

ECL_YawController::ECL_YawController() :
	_last_run(0),
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
	_coordinated_min_speed(1.0f)
{
}

float ECL_YawController::control_attitude(float roll, float pitch,
		float speed_body_u, float speed_body_v, float speed_body_w,
		float roll_rate_setpoint, float pitch_rate_setpoint)
{
//	static int counter = 0;
	/* PR #975: do not calculate control signal with bad inputs */
	if (!(isfinite(roll) && isfinite(pitch) && isfinite(speed_body_u) && isfinite(speed_body_v) && isfinite(speed_body_w) && isfinite(roll_rate_setpoint) && isfinite(pitch_rate_setpoint))) {
		perf_count(_nonfinite_input_perf);
		return _rate_setpoint;
	}
	/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
	_rate_setpoint = 0.0f;
	if (sqrtf(speed_body_u * speed_body_u + speed_body_v * speed_body_v + speed_body_w * speed_body_w) > _coordinated_min_speed) {
		/* PROFILE-DRIVEN OPT: cosf(pitch) and sinf(roll) were each computed twice; sin+cos of
		   roll and of pitch are both needed -> two fast_sincosf calls instead of 6 separate trig. */
		float sin_roll, cos_roll, sin_pitch, cos_pitch;
		fast_sincosf(roll, &sin_roll, &cos_roll);
		fast_sincosf(pitch, &sin_pitch, &cos_pitch);
		float denumerator = (speed_body_u * cos_roll * cos_pitch + speed_body_w * sin_pitch);
		if (denumerator != 0.0f) { //XXX: floating point comparison
			_rate_setpoint = (speed_body_w * roll_rate_setpoint + 9.81f * sin_roll * cos_pitch + speed_body_u * pitch_rate_setpoint * sin_roll) / denumerator;
//			warnx("yaw: speed_body_u %.f speed_body_w %1.f roll %.1f pitch %.1f denumerator %.1f _rate_setpoint %.1f", speed_body_u, speed_body_w, denumerator, _rate_setpoint);
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

	if(!isfinite(_rate_setpoint)){
		warnx("yaw rate sepoint not finite");
		_rate_setpoint = 0.0f;
	}

	return _rate_setpoint;
}

float ECL_YawController::control_bodyrate(float roll, float pitch,
		float pitch_rate, float yaw_rate,
		float pitch_rate_setpoint,
		float airspeed_min, float airspeed_max, float airspeed, float scaler, bool lock_integrator)
{
	/* PR #975: do not calculate control signal with bad inputs */
	if (!(isfinite(roll) && isfinite(pitch) && isfinite(pitch_rate) && isfinite(yaw_rate) && isfinite(pitch_rate_setpoint) && isfinite(airspeed_min) && isfinite(airspeed_max) && isfinite(scaler))) {
		perf_count(_nonfinite_input_perf);
		return math::constrain(_last_output, -1.0f, 1.0f);
	}
	/* get the usual dt estimate */
	/* PROFILE-DRIVEN OPT: the base reads the clock TWICE (ecl_elapsed_time then
	   ecl_absolute_time) -- the dominant cost in the profile. Read once: one
	   consistent timestamp, half the timer reads. */
	const uint64_t _now = ecl_absolute_time();
	uint64_t dt_micros = _now - _last_run;
	_last_run = _now;
	float dt = (float)(uint32_t)dt_micros * 1e-6f;

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
	/* PROFILE-DRIVEN OPT: sinf(roll), cosf(roll), cosf(pitch) were each computed twice.
	   Compute once; cos+sin of roll together via fast_sincosf. */
	float sin_roll, cos_roll;
	fast_sincosf(roll, &sin_roll, &cos_roll);
	const float cr_cp = cos_roll * cosf(pitch);
	_bodyrate_setpoint = -sin_roll * pitch_rate_setpoint + cr_cp * _rate_setpoint; //jacobian

	/* Transform estimation to body angular rates */
	float yaw_bodyrate = -sin_roll * pitch_rate + cr_cp * yaw_rate; //jacobian

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - yaw_bodyrate; //body angular rate error

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
	_last_output = (_bodyrate_setpoint * _k_ff + _rate_error * _k_p + integrator_constrained) * scaler * scaler;  //scaler is proportional to 1/airspeed


	return math::constrain(_last_output, -1.0f, 1.0f);
}

void ECL_YawController::reset_integrator()
{
	_integrator = 0.0f;
}
