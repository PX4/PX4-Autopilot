/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include <cmath>

class PID
{
public:
	PID() = default;
	virtual ~PID() = default;
	void setOutputLimit(const float limit) { _limit_output = limit; }
	void setIntegralLimit(const float limit) { _limit_integral = limit; }
	void setGains(const float P, const float I, const float D);
	void setSetpoint(const float setpoint) { _setpoint = setpoint; }
	float update(const float feedback, const float dt, const bool update_integral = true);
	float getIntegral() { return _integral; }
	void resetIntegral() { _integral = 0.f; };
	void resetDerivative() { _last_feedback = NAN; };
private:
	void updateIntegral(float error, const float dt);
	float updateDerivative(float feedback, const float dt);

	float _setpoint{0.f}; ///< current setpoint to track
	float _integral{0.f}; ///< integral state
	float _last_feedback{NAN};

	// Gains, Limits
	float _gain_proportional{0.f};
	float _gain_integral{0.f};
	float _gain_derivative{0.f};
	float _limit_integral{0.f};
	float _limit_output{0.f};
};
