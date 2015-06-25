/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file tiltrotor.h
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 *
 */

#ifndef TILTROTOR_H
#define TILTROTOR_H
#include "vtol_type.h"
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>

class Tiltrotor : public VtolType
{

public:

	Tiltrotor(VtolAttitudeControl * _att_controller);
	~Tiltrotor();

	void update_vtol_state();
	void update_mc_state();
	void process_mc_data();
	void update_fw_state();
	void process_fw_data();
	void update_transition_state();
	void update_external_state();

private:

	struct {
		float front_trans_dur;
		float back_trans_dur;
		float tilt_mc;
		float tilt_transition;
		float tilt_fw;
		float airspeed_trans;
		int elevons_mc_lock;			// lock elevons in multicopter mode
	} _params_tiltrotor;

	struct {
		param_t front_trans_dur;
		param_t back_trans_dur;
		param_t tilt_mc;
		param_t tilt_transition;
		param_t tilt_fw;
		param_t airspeed_trans;
		param_t elevons_mc_lock;
	} _params_handles_tiltrotor;

	enum vtol_mode {
		MC_MODE = 0,
		TRANSITION_FRONT_P1,
		TRANSITION_FRONT_P2,
		TRANSITION_BACK,
		FW_MODE
	};

	struct {
		vtol_mode flight_mode;			// indicates in which mode the vehicle is in
		hrt_abstime transition_start;	// at what time did we start a transition (front- or backtransition)
	}_vtol_schedule;

	bool flag_max_mc;
	float _tilt_control;
	float _roll_weight_mc;

	void fill_mc_att_control_output();
	void fill_fw_att_control_output();
	void set_max_mc();
	void set_max_fw(unsigned pwm_value);

	int parameters_update();

};
#endif
