/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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

/*
 * @file params.h
 *
 * Definition of parameters for fixedwing example
 */

#include <parameters/param.h>
#include <matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>


struct params {
	float hdng_p;
	float roll_p;
	float pitch_p;
};

struct param_handles {
	param_t hdng_p;
	param_t roll_p;
	param_t pitch_p;
};

hrt_abstime _last_run{0};

//From ecl_controller.h in fw_att_control folder
/*
class ECL_Controller
{
public:
	ECL_Controller();
	virtual ~ECL_Controller() = default;


	//!!!Change to designed controllers!!!
	virtual float control_attitude(const float dt, const Control_Data &ctl_data) = 0;
	virtual float control_euler_rate(const float dt, const Control_Data &ctl_data) = 0;
	virtual float control_bodyrate(const float dt, const Control_Data &ctl_data) = 0;

	// Setters
	void set_time_constant(float time_constant);
	void set_k_p(float k_p);
	void set_k_i(float k_i);
	void set_k_ff(float k_ff);
	void set_integrator_max(float max);
	void set_max_rate(float max_rate);
	void set_bodyrate_setpoint(float rate);

	// Getters
	float get_rate_error();
	float get_desired_rate();
	float get_desired_bodyrate();
	float get_integrator();

	void reset_integrator();


protected:
	uint64_t _last_run;
	float _tc;
	float _k_p;
	float _k_i;
	float _k_ff;
	float _integrator_max;
	float _max_rate;
	float _last_output;
	matrix::Vector2f _integrator;
	float _rate_error;
	float _rate_setpoint;
	float _bodyrate_setpoint;
	float constrain_airspeed(float airspeed, float minspeed, float maxspeed);
};
*/
