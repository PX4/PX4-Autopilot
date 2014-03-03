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
 * @file ecl_yaw_controller.h
 * Definition of a simple orthogonal coordinated turn yaw PID controller.
 *
 */
#ifndef ECL_YAW_CONTROLLER_H
#define ECL_YAW_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

class __EXPORT ECL_YawController
{
public:
	ECL_YawController();

	float control(float roll, float yaw_rate, float accel_y, float scaler = 1.0f, bool lock_integrator = false,
		      float airspeed_min = 0, float airspeed_max = 0, float aspeed = (0.0f / 0.0f));

	void reset_integrator();

	void set_k_side(float k_a) {
		_k_side = k_a;
	}
	void set_k_i(float k_i) {
		_k_i = k_i;
	}
	void set_k_d(float k_d) {
		_k_d = k_d;
	}
	void set_k_roll_ff(float k_roll_ff) {
		_k_roll_ff = k_roll_ff;
	}
	void set_integrator_max(float max) {
		_integrator_max = max;
	}

private:
	uint64_t _last_run;

	float _k_side;
	float _k_i;
	float _k_d;
	float _k_roll_ff;
	float _integrator_max;

	float _last_error;
	float _last_output;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _k_d_last;
	float _integrator;

};

#endif // ECL_YAW_CONTROLLER_H
