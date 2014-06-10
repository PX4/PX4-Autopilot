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

#include <stdbool.h>
#include <stdint.h>
#include <systemlib/perf_counter.h>

class __EXPORT ECL_PitchController //XXX: create controller superclass
{
public:
	ECL_PitchController();

	~ECL_PitchController();

	float control_attitude(float pitch_setpoint, float roll, float pitch, float airspeed);


	float control_bodyrate(float roll, float pitch,
			float pitch_rate, float yaw_rate,
			float yaw_rate_setpoint,
			float airspeed_min = 0.0f, float airspeed_max = 0.0f, float airspeed = (0.0f / 0.0f), float scaler = 1.0f, bool lock_integrator = false);

	void reset_integrator();

	void set_time_constant(float time_constant) {
		_tc = time_constant;
	}
	void set_k_p(float k_p) {
		_k_p = k_p;
	}

	void set_k_i(float k_i) {
		_k_i = k_i;
	}

	void set_k_ff(float k_ff) {
		_k_ff = k_ff;
	}

	void set_integrator_max(float max) {
		_integrator_max = max;
	}

	void set_max_rate_pos(float max_rate_pos) {
		_max_rate_pos = max_rate_pos;
	}

	void set_max_rate_neg(float max_rate_neg) {
		_max_rate_neg = max_rate_neg;
	}

	void set_roll_ff(float roll_ff) {
		_roll_ff = roll_ff;
	}

	float get_rate_error() {
		return _rate_error;
	}

	float get_desired_rate() {
		return _rate_setpoint;
	}

	float get_desired_bodyrate() {
		return _bodyrate_setpoint;
	}

private:

	uint64_t _last_run;
	float _tc;
	float _k_p;
	float _k_i;
	float _k_ff;
	float _integrator_max;
	float _max_rate_pos;
	float _max_rate_neg;
	float _roll_ff;
	float _last_output;
	float _integrator;
	float _rate_error;
	float _rate_setpoint;
	float _bodyrate_setpoint;
	perf_counter_t _nonfinite_input_perf;
};

#endif // ECL_PITCH_CONTROLLER_H
