/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
 * @file fw_roll_controller.cpp
 * Implementation of a simple roll P controller.
 */

#include "fw_roll_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

float RollController::control_roll(float roll_setpoint, float euler_yaw_rate_setpoint, float roll, float pitch)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(roll_setpoint) &&
	      PX4_ISFINITE(euler_yaw_rate_setpoint) &&
	      PX4_ISFINITE(pitch) &&
	      PX4_ISFINITE(roll))) {

		return _body_rate_setpoint;
	}

	const float roll_error = roll_setpoint - roll;
	_euler_rate_setpoint = roll_error / _tc;

	/* Transform setpoint to body angular rates (jacobian) */
	const float roll_body_rate_setpoint_raw = _euler_rate_setpoint - sinf(pitch) *
			euler_yaw_rate_setpoint;
	_body_rate_setpoint = math::constrain(roll_body_rate_setpoint_raw, -_max_rate, _max_rate);

	return _body_rate_setpoint;
}
