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
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#ifndef TAILSITTER_H
#define TAILSITTER_H

#include "vtol_type.h"
#include <systemlib/perf_counter.h>  /** is it necsacery? **/
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>

class Tailsitter : public VtolType
{

public:
	Tailsitter(VtolAttitudeControl *_att_controller);
	~Tailsitter();

	virtual void update_vtol_state();
	virtual void update_transition_state();
	virtual void update_mc_state();
	virtual void update_fw_state();
	virtual void fill_actuator_outputs();

private:

	struct {
		float front_trans_dur;			/**< duration of first part of front transition */
		float front_trans_dur_p2;
		float back_trans_dur;			/**< duration of back transition */
		float airspeed_trans;			/**< airspeed at which we switch to fw mode after transition */
		float airspeed_blend_start;		/**< airspeed at which we start blending mc/fw controls */
		int elevons_mc_lock;			/**< lock elevons in multicopter mode */

	} _params_tailsitter;

	struct {
		param_t front_trans_dur;
		param_t front_trans_dur_p2;
		param_t back_trans_dur;
		param_t airspeed_trans;
		param_t airspeed_blend_start;
		param_t elevons_mc_lock;

	} _params_handles_tailsitter;

	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	struct {
		vtol_mode flight_mode;			/**< vtol flight mode, defined by enum vtol_mode */
		hrt_abstime transition_start;	/**< absoulte time at which front transition started */
	} _vtol_schedule;

	float _airspeed_tot; 		/** speed estimation for propwash controlled surfaces */

	/** not sure about it yet ?! **/
	float _min_front_trans_dur;	/**< min possible time in which rotors are rotated into the first position */


	/** should this anouncement stay? **/
	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */

	/**
	 * Speed estimation for propwash controlled surfaces.
	 */
	void calc_tot_airspeed();


	/** is this one still needed? */
	void scale_mc_output();

	/**
	 * Update parameters.
	 */
	int parameters_update();

};
#endif
