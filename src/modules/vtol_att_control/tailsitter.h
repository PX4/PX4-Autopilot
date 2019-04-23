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
* @file tailsitter.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#ifndef TAILSITTER_H
#define TAILSITTER_H

#include "vtol_type.h"
#include "ILC_DATA.h"
#include <perf/perf_counter.h>  /** is it necsacery? **/
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/math/EulerFromQuat.hpp>
#include <uORB/topics/vehicle_local_position.h>

class Tailsitter : public VtolType
{

public:
	Tailsitter(VtolAttitudeControl *_att_controller);
	~Tailsitter() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;

	virtual float control_altitude(float time_since_trans_start, float alt_cmd);
	virtual float thr_from_acc_cmd(float vert_acc_cmd, float airspeed, float pitch_ang, float aoa);
	virtual float get_CL(float aoa);

private:

	struct {
		float front_trans_dur_p2;
		float fw_pitch_sp_offset;
		float sys_ident_input;
		int   sys_ident_num;
	} _params_tailsitter{};

	struct {
		param_t front_trans_dur_p2;
		param_t fw_pitch_sp_offset;
		param_t sys_ident_input;
		param_t sys_ident_num;
	} _params_handles_tailsitter{};

	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	enum sweep_type {
		NO_SWEEP = 0,
		PITCH_RATE,
		ROLL_RATE,
		YAW_RATE,
		THRUST
	};

	enum wall_suck_type {
		NO_SUCK = 0,
		TOP_WALL,
		SIDE_WALL,
	};

	struct {
		vtol_mode   flight_mode;	    /**< vtol flight mode, defined by enum vtol_mode */
		float       ctrl_out_trans_end; /**< MC controller output at the end of front transition */
		hrt_abstime fw_start;           /**< absoulte time at which fw mode started, this time will be used to smooth the controller output */
		hrt_abstime sweep_start;
		hrt_abstime f_trans_start_t;	/**< absoulte time at which front transition started */
		hrt_abstime b_trans_start_t;
	} _vtol_schedule;

	matrix::Quatf _q_trans_start;
	matrix::Quatf _q_trans_sp;
	matrix::Vector3f _trans_rot_axis;
	matrix::Vector3f _trans_roll_axis;

	float _alt_sp;
	float _vert_i_term;
	float _mc_hover_thrust;
	float _trans_end_thrust;
	float _trans_pitch_rot;
	float _trans_roll_rot;
	float _trans_start_x;
	float _trans_start_y;
	float _CL_Degree[NUM_CL_POINTS+1];
	float _target_alt;

	void parameters_update() override;

};
#endif
