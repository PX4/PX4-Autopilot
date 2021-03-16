/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ecl_pitch_controller.h
 * Definition of a simple orthogonal pitch PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef ECL_PITCH_CONTROLLER_H
#define ECL_PITCH_CONTROLLER_H

#include <mathlib/mathlib.h>

#include "ecl_controller.h"

class ECL_PitchController :
	public ECL_Controller
{
public:
	ECL_PitchController() = default;
	~ECL_PitchController() = default;

	float control_attitude(const float dt, const ECL_ControlData &ctl_data) override;
	float control_euler_rate(const float dt, const ECL_ControlData &ctl_data, float bodyrate_ff) override;
	float control_bodyrate(const float dt, const ECL_ControlData &ctl_data) override;

	/* Additional Setters */
	void set_max_rate_pos(float max_rate_pos)
	{
		_max_rate = max_rate_pos;
	}

	void set_max_rate_neg(float max_rate_neg)
	{
		_max_rate_neg = max_rate_neg;
	}

	void set_bodyrate_setpoint(float rate)
	{
		_bodyrate_setpoint = math::constrain(rate, -_max_rate_neg, _max_rate);
	}

	float get_gain_compression_factor() const { return _gain_compression_factor; }

protected:
	float _max_rate_neg{0.0f};
	float _roll_ff{0.0f};
	float _gyro_contribution_lpf;	// contribution of the gyro feedback to the actuator demand after low pass filtering
	float _max_pos_slew_rate{0.0f}; // peak slew rate of the gyro induced actuator demand in the positive direction
	float _max_neg_slew_rate{0.0f}; // peak slew rate of the gyro induced actuator demand in the negative direction
	float _gain_compression_factor{1.0f}; // factor applied to the rate error control gain _k_p
};

#endif // ECL_PITCH_CONTROLLER_H
