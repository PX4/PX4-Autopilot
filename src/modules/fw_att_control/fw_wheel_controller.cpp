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
 * @file fw_wheel_controller.cpp
 * Implementation of a simple PID wheel controller for heading tracking.
 */

#include "fw_wheel_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using matrix::wrap_pi;

float WheelController::control_bodyrate(float dt, float body_z_rate, float groundspeed, float groundspeed_scaler)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(body_z_rate) &&
	      PX4_ISFINITE(groundspeed) &&
	      PX4_ISFINITE(groundspeed_scaler))) {

		return math::constrain(_last_output, -1.f, 1.f);
	}

	const float rate_error = _body_rate_setpoint - body_z_rate;

	if (_k_i > 0.f && groundspeed > 1.f) { // only start integrating when above 1m/s

		float id = rate_error * dt * groundspeed_scaler;

		if (_last_output < -1.f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.f);

		} else if (_last_output > 1.f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.f);
		}

		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	}

	/* Apply PI rate controller and store non-limited output */
	_last_output = _body_rate_setpoint * _k_ff * groundspeed_scaler +
		       groundspeed_scaler * groundspeed_scaler * (rate_error * _k_p + _integrator);

	return math::constrain(_last_output, -1.f, 1.f);
}

float WheelController::control_attitude(float yaw_setpoint, float yaw)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(yaw_setpoint) &&
	      PX4_ISFINITE(yaw))) {

		return _body_rate_setpoint;
	}

	const float yaw_error = wrap_pi(yaw_setpoint - yaw);

	_body_rate_setpoint = yaw_error / _tc; // assume 0 pitch and roll angle, thus jacobian is simply identity matrix

	if (_max_rate > 0.01f) {
		if (_body_rate_setpoint > 0.f) {
			_body_rate_setpoint = (_body_rate_setpoint > _max_rate) ? _max_rate : _body_rate_setpoint;

		} else {
			_body_rate_setpoint = (_body_rate_setpoint < -_max_rate) ? -_max_rate : _body_rate_setpoint;
		}

	}

	return _body_rate_setpoint;
}
