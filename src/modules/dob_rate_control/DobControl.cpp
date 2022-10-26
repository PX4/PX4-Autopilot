/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file DobControl.cpp
 */

#include "DobControl.hpp"
#include <px4_platform_common/defines.h>

using namespace matrix;

void DobControl::setGains(const Vector3f &tau)
{
	_gain_tau = tau;
}

void DobControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				     const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

Vector3f DobControl::update(const Vector3f &rate,
			    const Vector3f actuator_sp, const float dt, const bool landed)
{
	// angular rates error
	///TODO: Use angular acceleration instead of differentiator
	Vector3f diff = differentiator(rate, dt);

	matrix::Vector3f integ_input = actuator_sp - diff;

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(integ_input, dt);
	}

	const matrix::Vector3f torque = integ_input + _input_rate;
	return torque;
}

Vector3f DobControl::differentiator(const matrix::Vector3f &input, const float dt)
{
	Vector3f output = (1 - _c0) * _prev_output + (_c0 / dt) * (input - _prev_input);
	_prev_input = input;
	_prev_output = output;
	return output;
}

void DobControl::updateIntegral(Vector3f &integ_input, const float dt)
{
	_input_rate = _input_rate + _gain_tau.emult(integ_input);

}

void DobControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _input_rate(0);
	rate_ctrl_status.pitchspeed_integ = _input_rate(1);
	rate_ctrl_status.yawspeed_integ = _input_rate(2);
}
