/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
* @file standard.h
* VTOL with fixed multirotor motor configurations (such as quad) and a pusher
* (or puller aka tractor) motor for forward flight.
*
* @author Simon Wilks 		<simon@uaventure.com>
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
* @author Sander Smeets 	<sander@droneslab.com>
*
*/

#ifndef STANDARD_H
#define STANDARD_H
#include "vtol_type.h"

class Standard : public VtolType
{

public:

	Standard(VtolAttitudeControl *_att_controller);
	~Standard() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void update_mc_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;

private:

	enum class vtol_mode {
		MC_MODE = 0,
		TRANSITION_TO_FW,
		TRANSITION_TO_MC,
		FW_MODE
	};

	struct {
		vtol_mode flight_mode;			// indicates in which mode the vehicle is in
		hrt_abstime transition_start;	// at what time did we start a transition (front- or backtransition)
	} _vtol_schedule;

	float _pusher_throttle{0.0f};
	float _reverse_output{0.0f};
	float _airspeed_trans_blend_margin{0.0f};

	void parameters_update() override;

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::VT_PSHER_RMP_DT>) _param_vt_psher_rmp_dt,
					(ParamFloat<px4::params::VT_B_TRANS_RAMP>) _param_vt_b_trans_ramp,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
					(ParamFloat<px4::params::VT_B_REV_OUT>) _param_vt_b_rev_out,
					(ParamFloat<px4::params::VT_B_REV_DEL>) _param_vt_b_rev_del
				       )
};
#endif
