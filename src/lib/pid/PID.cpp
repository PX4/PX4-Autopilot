/****************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
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

#include "PID.hpp"
#include "lib/mathlib/math/Functions.hpp"

void PID::setOutputLimit(const float lower_limit, const float upper_limit)
{
	_lower_limit_output = lower_limit;
	_upper_limit_output = upper_limit;
}

void PID::setIntegralLimit(const float lower_limit, const float upper_limit)
{
	_lower_limit_integral = lower_limit;
	_upper_limit_integral = upper_limit;
}

void PID::setGains(const float P, const float I, const float D)
{
	_gain_proportional = P;
	_gain_integral = I;
	_gain_derivative = D;
}

float PID::update(const float feedback, const float dt, const bool update_integral)
{
	const float error = _setpoint - feedback;
	const float derivative = updateDerivative(feedback, dt);
	const float output = (_gain_feedforward * _setpoint) + (_gain_proportional * error) + _integral -
			     (_gain_derivative * derivative);
	const float output_constrained = math::constrain(output, _lower_limit_output, _upper_limit_output);

	if (update_integral) {
		updateIntegral(error, output - output_constrained, dt);
	}

	_last_feedback = feedback;
	return output_constrained;
}

void PID::updateIntegral(float error, const float saturation, const float dt)
{
	// Anti-windup: stop integrating in the direction that drives the output
	// further into saturation (conditional integration)
	if ((saturation > FLT_EPSILON && error > 0.f) || (saturation < -FLT_EPSILON && error < 0.f)) {
		error = 0.f;
	}

	const float integral_new = _integral + _gain_integral * error * dt;

	if (std::isfinite(integral_new)) {
		_integral = math::constrain(integral_new, _lower_limit_integral, _upper_limit_integral);
	}
}

float PID::updateDerivative(float feedback, const float dt)
{
	float feedback_change = 0.f;

	if ((dt > FLT_EPSILON) && std::isfinite(_last_feedback)) {
		feedback_change = (feedback - _last_feedback) / dt;
	}

	return feedback_change;
}
