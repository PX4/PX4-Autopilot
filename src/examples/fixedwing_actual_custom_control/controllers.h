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
 * @file ecl_roll_controller.h
 * Definition of a simple orthogonal roll PID controller.
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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "ecl_controller.h"


class Controllers: public ECL_Controller{ //Figure out how to import BlockHighPass class
public:
	Controllers() = default;
	~Controllers() = default;

	//float control_attitude(const float dt, const ECL_ControlData &ctl_data) override;
	//float control_euler_rate(const float dt, const ECL_ControlData &ctl_data) override;
	//float control_bodyrate(const float dt, const ECL_ControlData &ctl_data) override;

	void init_ECL_variables();
	float pitch_rate_damper_controller(const Control_Data &state_data,float dE);
	void airspeed_climb_rate_controller(const Control_Data &state_data,float vbar_ref,float hdot_ref,const float dt,float *defl);
	float altitude_controller(const Control_Data &state_data,float h_ref);
	void initialise_NSADLC_HPF();
	float dutch_roll_damper_controller(const Control_Data &state_data,float dt);
	float roll_angle_controller(const Control_Data &state_data,float phi_ref,const float dt);
	float heading_controller(const Control_Data &state_data,float psi_ref);
	void guide_axis_alg(float S[2],float D[2],const Control_Data &state_data,float out[4]);
	void waypoint_scheduler(float Destinations[][2],int Destination_size,const Control_Data &state_data,float out[2][2]);
	float guidance_controller(float D[][2],int Destination_size,const Control_Data &state_data);

	//Actual Controllers
	float airspeed_controller(const Control_Data &state_data,float vbar_ref,const float dt);
	void NSADLC_controller(const Control_Data &state_data,float Cw_ref,const float dt,float *defl);
	float climb_rate_controller(const Control_Data &state_data,float hdot_ref,const float dt);
	float altitude_controller(const Control_Data &state_data,float h_ref,const float dt);
	float LSA(const Control_Data &state_data,float Bw_ref,const float dt);
	float roll_rate_controller(const Control_Data &state_data,float p_ref,const float dt);
	float roll_angle_controller(const Control_Data &state_data,float phi_ref,const float dt);
	float guidance_controller_1(const Control_Data &state_data,float phibar_ref,float y_ref,const float dt);
	float yaw_controller(const Control_Data &state_data,float psi_ref,const float dt);
};

#endif // ECL_ROLL_CONTROLLER_H
