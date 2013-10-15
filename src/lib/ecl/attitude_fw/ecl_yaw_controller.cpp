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

ECL_YawController::ECL_YawController() :
	_last_run(0),
	_last_output(0.0f),
	_rate_setpoint(0.0f),
	_max_deflection_rad(math::radians(45.0f))

{

}

float ECL_YawController::control(float roll, float yaw_rate, float accel_y, float scaler, bool lock_integrator,
				 float airspeed_min, float airspeed_max, float aspeed)
{
	/* get the usual dt estimate */
	uint64_t dt_micros = ecl_elapsed_time(&_last_run);
	_last_run = ecl_absolute_time();

	float dt = (dt_micros > 500000) ? 0.0f : dt_micros / 1000000;

//	float psi_dot = 0.0f;
//		float denumerator = (speed_body[0] * cosf(att_sp->roll_body) * cosf(att_sp->pitch_body) + speed_body[2] * sinf(att_sp->pitch_body));
//		if(denumerator != 0.0f) {
//			psi_dot = (speed_body[2] * phi_dot + 9.81f * sinf(att_sp->roll_body) * cosf(att_sp->pitch_body) + speed_body[0] * theta_dot * sinf(att_sp->roll_body))
//				/ (speed_body[0] * cosf(att_sp->roll_body) * cosf(att_sp->pitch_body) + speed_body[2] * sinf(att_sp->pitch_body));
//		}

	/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
	_last_output = 0.0f;
	float denumerator = (speed_body_u * cosf(roll) * cosf(pitch) + speed_body_w * sinf(pitch));
	if(denumerator != 0.0f) { //XXX: floating point comparison
		_last_output = (speed_body_w * roll_rate_desired + 9.81f * sinf(roll) * cosf(pitch) + speed_body_u * pitch_rate_desired * sinf(roll)) / denumerator;
	}

	return math::constrain(_last_output, -_max_deflection_rad, _max_deflection_rad);
}

void ECL_YawController::reset_integrator()
{
	_integrator = 0.0f;
}
