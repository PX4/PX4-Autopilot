/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file fw_yaw_controller.cpp
 * Implementation of a simple coordinated turn yaw controller.
 */

#include "fw_yaw_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

float YawController::control_yaw(float roll_setpoint, float euler_pitch_rate_setpoint, float roll, float pitch,
				 float airspeed)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(roll_setpoint) &&
	      PX4_ISFINITE(roll) &&
	      PX4_ISFINITE(pitch) &&
	      PX4_ISFINITE(euler_pitch_rate_setpoint) &&
	      PX4_ISFINITE(airspeed))) {

		return _body_rate_setpoint;
	}

	float constrained_roll;
	bool inverted = false;

	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(roll) < math::radians(90.f)) {
		/* not inverted, but numerically still potentially close to infinity */
		constrained_roll = math::constrain(roll, math::radians(-80.f), math::radians(80.f));

	} else {
		inverted = true;

		// inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
		//note: the ranges are extended by 10 deg here to avoid numeric resolution effects
		if (roll > 0.f) {
			/* right hemisphere */
			constrained_roll = math::constrain(roll, math::radians(100.f), math::radians(180.f));

		} else {
			/* left hemisphere */
			constrained_roll = math::constrain(roll, math::radians(-180.f), math::radians(-100.f));
		}
	}

	constrained_roll = math::constrain(constrained_roll, -fabsf(roll_setpoint), fabsf(roll_setpoint));


	if (!inverted) {
		/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
		_euler_rate_setpoint = tanf(constrained_roll) * cosf(pitch) * CONSTANTS_ONE_G / airspeed;

		/* Transform setpoint to body angular rates (jacobian) */
		const float yaw_body_rate_setpoint_raw = -sinf(roll) * euler_pitch_rate_setpoint +
				cosf(roll) * cosf(pitch) * _euler_rate_setpoint;
		_body_rate_setpoint = math::constrain(yaw_body_rate_setpoint_raw, -_max_rate, _max_rate);
	}

	if (!PX4_ISFINITE(_body_rate_setpoint)) {
		_body_rate_setpoint = 0.f;
	}

	return _body_rate_setpoint;
}
