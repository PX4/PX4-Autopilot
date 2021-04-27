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
 * @file ecl_controller.h
 * Definition of base class for other controllers
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

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_log.h>
#include "BlockHighPass.h"

struct ECL_ControlData {
	float roll;
	float pitch;
	float yaw;
	float body_x_rate;
	float body_y_rate;
	float body_z_rate;
	float roll_setpoint;
	float pitch_setpoint;
	float yaw_setpoint;
	float roll_rate_setpoint;
	float pitch_rate_setpoint;
	float yaw_rate_setpoint;
	float airspeed_min;
	float airspeed_max;
	float airspeed;
	float scaler;
	float groundspeed;
	float groundspeed_scaler;
	bool lock_integrator;
};

struct Control_Data {
	float phi;//roll
	float theta;//pitch
	float psi;//yaw
	float p;//roll rate
	float q;//pitch rate
	float r;//yaw rate
	float airspeed;
	float alpha;
	float beta;
	float posx;
	float posy;
	float posz;//check is posz is positive upwards or downwards
	float h_dot;
	float Cw;
	float Bw;
	float y;
	float ydot;
	float velx_I;
	float vely_I;
	float velz_I;
};

class ECL_Controller
{
public:
	ECL_Controller();
	virtual ~ECL_Controller() = default;

	//virtual float control_attitude(const float dt, const ECL_ControlData &ctl_data) = 0;
	//virtual float control_euler_rate(const float dt, const ECL_ControlData &ctl_data) = 0;
	//virtual float control_bodyrate(const float dt, const ECL_ControlData &ctl_data) = 0;

	/* Setters */
	void set_time_constant(float time_constant);
	void set_k_p(float k_p);
	void set_k_i(float k_i);
	void set_k_ff(float k_ff);
	void set_integrator_max(float max);
	void set_max_rate(float max_rate);
	void set_bodyrate_setpoint(float rate);

	/* Getters */
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
	float _hdot_max;//altitude controller saturation block max limit
	float _hdot_min;//altitude controller saturation block min limit
	float _phi_max;//heading controller roll angle saturation block max limit
	float _phi_min;//heading controller roll angle saturation block max limit
	float _max_rate;
	float _last_output;
	float _integrator;
	float _integrator_max;
	float _integrator_I[2][1];//Airspeed and Climb Rate integrator
	float _integrator_RA;//Roll Angle Controller intergrator
	float _x_guide;//intial value must be 0
	int _loc_guide;//initial value must be 1(location of first destination)
	float K_q;//pitch rate damper gain
	float K_ASCR[2][6];//airspeed and climbrate controller gain
	float K_h;//altitude controller gain
	float K_P_RAC;//roll angle controller proportional control gain
	float K_I_RAC;//roll angle controller intergrator control gain
	float K_psi;//heading controller gain
	float K_y;//guidance controller gain
	float vbar_trim;//trim airspeed
	float alpha_trim;//trim alpha(and theta)
	float _rate_error;
	float _rate_setpoint;
	float _bodyrate_setpoint;
	float constrain_airspeed(float airspeed, float minspeed, float maxspeed);
	float Ki_AS;
	float Kp_AS;
	float _intergrator_AS;
	BlockHighPass NSADLC_HPF;
	float _intergrator_DLC;
	float Ki_f;
	float _intergrator_NSA;
	float Nc;
	float Ki_e;
	float Kc;
	float Kq;
	float Km;
	float _intergrator_CR;
	float Ki_CR;
	float Kp_CR;
	float Kp_alt;
	float _intergrator_LSA;
	float Ki_LSA;
	float K_r;
	float K_B;
	float _intergrator_RR;
	float Ki_RR;
	float Kp_RR;
	float Kp_RA;
	float Kp_G;
	float Kd_G;
	float _intergrator_yaw;
	float Ki_yaw;
	float Kp_yaw;
};
