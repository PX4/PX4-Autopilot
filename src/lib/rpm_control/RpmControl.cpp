/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include "RpmControl.hpp"

#include <mathlib/mathlib.h>

void RpmControl::reset(int motor)
{
	if (!inRange(motor)) {
		return;
	}

	_sp_rpm[motor]        = 0.f;
	_meas_rpm[motor]      = 0.f;
	_integral[motor]      = 0.f;
	_prev_err[motor]      = 0.f;
	_last_cmd_norm[motor] = 0.f;
}

float RpmControl::update(int motor, float sp_norm, float meas_rpm, float dt)
{
	if (!inRange(motor)) {
		return math::constrain(sp_norm, 0.f, 1.f);
	}

	sp_norm = math::constrain(sp_norm, 0.f, 1.f);

	const float sp_rpm = sp_norm * _max_rpm;
	const float error  = (sp_rpm - meas_rpm) / _max_rpm;      // normalized to [-1, 1]

	// Integrate with anti-windup clamp.
	_integral[motor] = math::constrain(_integral[motor] + error * dt,
					   -_gains.i_lim, _gains.i_lim);

	// Derivative on error; guard against zero/negative dt.
	const float d_err = (dt > 1e-6f) ? (error - _prev_err[motor]) / dt : 0.f;
	_prev_err[motor]  = error;

	const float cmd = sp_norm
			  + _gains.kp * error
			  + _gains.ki * _integral[motor]
			  + _gains.kd * d_err;

	const float cmd_norm = math::constrain(cmd, 0.f, 1.f);

	_sp_rpm[motor]        = sp_rpm;
	_meas_rpm[motor]      = meas_rpm;
	_last_cmd_norm[motor] = cmd_norm;

	return cmd_norm;
}
