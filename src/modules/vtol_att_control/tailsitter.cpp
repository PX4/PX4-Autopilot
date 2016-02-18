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
* @file tailsitter.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author David Vorsin     <davidvorsin@gmail.com>
*
*/

#include "tailsitter.h"
#include "vtol_att_control_main.h"

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition
#define THROTTLE_TRANSITION_MAX 0.25f	// maximum added thrust above last value in transition
#define PITCH_TRANSITION_FRONT_P1 -1.1f	// pitch angle to switch to TRANSITION_P2
#define PITCH_TRANSITION_FRONT_P2 -1.2f	// pitch angle to switch to FW
#define PITCH_TRANSITION_BACK -0.25f	// pitch angle to switch to MC

Tailsitter::Tailsitter(VtolAttitudeControl *attc) :
	VtolType(attc),
	_airspeed_tot(0.0f),
	_min_front_trans_dur(0.5f),
	_thrust_transition_start(0.0f),
	_yaw_transition(0.0f),
	_pitch_transition_start(0.0f),
	_loop_perf(perf_alloc(PC_ELAPSED, "vtol_att_control-tailsitter")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "vtol att control-tailsitter nonfinite input"))
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tailsitter.front_trans_dur = param_find("VT_F_TRANS_DUR");
	_params_handles_tailsitter.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");
	_params_handles_tailsitter.back_trans_dur = param_find("VT_B_TRANS_DUR");
	_params_handles_tailsitter.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_tailsitter.airspeed_blend_start = param_find("VT_ARSP_BLEND");
	_params_handles_tailsitter.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");

}

Tailsitter::~Tailsitter()
{

}

int
Tailsitter::parameters_update()
{
	float v;
	int l;

	/* vtol duration of a front transition */
	param_get(_params_handles_tailsitter.front_trans_dur, &v);
	_params_tailsitter.front_trans_dur = math::constrain(v, 1.0f, 5.0f);

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tailsitter.front_trans_dur_p2, &v);
	_params_tailsitter.front_trans_dur_p2 = v;

	/* vtol duration of a back transition */
	param_get(_params_handles_tailsitter.back_trans_dur, &v);
	_params_tailsitter.back_trans_dur = math::constrain(v, 0.0f, 5.0f);

	/* vtol airspeed at which it is ok to switch to fw mode */
	param_get(_params_handles_tailsitter.airspeed_trans, &v);
	_params_tailsitter.airspeed_trans = v;

	/* vtol airspeed at which we start blending mc/fw controls */
	param_get(_params_handles_tailsitter.airspeed_blend_start, &v);
	_params_tailsitter.airspeed_blend_start = v;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles_tailsitter.elevons_mc_lock, &l);
	_params_tailsitter.elevons_mc_lock = l;

	/* avoid parameters which will lead to zero division in the transition code */
	_params_tailsitter.front_trans_dur = math::max(_params_tailsitter.front_trans_dur, _min_front_trans_dur);

	if (_params_tailsitter.airspeed_trans < _params_tailsitter.airspeed_blend_start + 1.0f) {
		_params_tailsitter.airspeed_trans = _params_tailsitter.airspeed_blend_start + 1.0f;
	}

	return OK;
}

void Tailsitter::update_vtol_state()
{
	parameters_update();

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting in MC control mode, picking up
	 * forward speed. After the vehicle has picked up enough and sufficient pitch angle the uav will go into FW mode.
	 * For the backtransition the pitch is controlled in MC mode again and switches to full MC control reaching the sufficient pitch angle.
	*/

	if (!_attc->is_fixed_wing_requested()) {


		switch (_vtol_schedule.flight_mode) { // user switchig to MC mode
		case MC_MODE:
			break;

		case FW_MODE:
			_vtol_schedule.flight_mode 	= TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_FRONT_P2:
			// NOT USED
			// failsafe into multicopter mode
			//_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:

			// check if we have reached pitch angle to switch to MC mode
			if (_v_att->pitch >= PITCH_TRANSITION_BACK) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}

	} else {  // user switchig to FW mode

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT_P1:

			// check if we have reached airspeed  and pitch angle to switch to TRANSITION P2 mode
			if ((_airspeed->true_airspeed_m_s >= _params_tailsitter.airspeed_trans
			     && _v_att->pitch <= PITCH_TRANSITION_FRONT_P1) || !_armed->armed) {
				_vtol_schedule.flight_mode = FW_MODE;
				//_vtol_schedule.transition_start = hrt_absolute_time();
			}

			break;

		case TRANSITION_FRONT_P2:

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;

			/*  **LATER***  if pitch is closer to mc (-45>)
			*   go to transition P1
			*/
			break;
		}
	}

	// map tailsitter specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		_vtol_vehicle_status->vtol_in_trans_mode = false;
		_flag_was_in_trans_mode = false;
		break;

	case TRANSITION_FRONT_P1:
		_vtol_mode = TRANSITION;
		_vtol_vehicle_status->vtol_in_trans_mode = true;

	case TRANSITION_FRONT_P2:
		_vtol_mode = TRANSITION;
		_vtol_vehicle_status->vtol_in_trans_mode = true;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION;
		_vtol_vehicle_status->vtol_in_trans_mode = true;
		break;
	}
}

void Tailsitter::update_transition_state()
{
	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_yaw_transition = _v_att_sp->yaw_body;
		_pitch_transition_start = _v_att_sp->pitch_body;
		_thrust_transition_start = _v_att_sp->thrust;
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {

		/** create time dependant pitch angle set point + 0.2 rad overlap over the switch value*/
		_v_att_sp->pitch_body = _pitch_transition_start	- (fabsf(PITCH_TRANSITION_FRONT_P1 - _pitch_transition_start) *
					(float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tailsitter.front_trans_dur * 1000000.0f));
		_v_att_sp->pitch_body = math::constrain(_v_att_sp->pitch_body , PITCH_TRANSITION_FRONT_P1 - 0.2f ,
							_pitch_transition_start);

		/** create time dependant throttle signal higher than  in MC and growing untill  P2 switch speed reached */
		if (_airspeed->true_airspeed_m_s <= _params_tailsitter.airspeed_trans) {
			_thrust_transition = _thrust_transition_start + (fabsf(THROTTLE_TRANSITION_MAX * _thrust_transition_start) *
					    (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tailsitter.front_trans_dur * 1000000.0f));
			_thrust_transition = math::constrain(_thrust_transition , _thrust_transition_start ,
							    (1.0f + THROTTLE_TRANSITION_MAX) * _thrust_transition_start);
			_v_att_sp->thrust = _thrust_transition;
		}

		// disable mc yaw control once the plane has picked up speed
		if (_airspeed->true_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;

		} else {
			_mc_yaw_weight = 1.0f;
		}

		_mc_roll_weight = 1.0f;
		_mc_pitch_weight = 1.0f;

	} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, smoothly switch the actuator controls, keep pitching down

		/** no motor  switching */

		if (flag_idle_mc) {
			set_idle_fw();
			flag_idle_mc = false;
		}

		/** create time dependant pitch angle set point  + 0.2 rad overlap over the switch value*/
		if (_v_att_sp->pitch_body >= (PITCH_TRANSITION_FRONT_P2 - 0.2f)) {
			_v_att_sp->pitch_body = PITCH_TRANSITION_FRONT_P1 -
						(fabsf(PITCH_TRANSITION_FRONT_P2 - PITCH_TRANSITION_FRONT_P1) * (float)hrt_elapsed_time(
							 &_vtol_schedule.transition_start) / (_params_tailsitter.front_trans_dur_p2 * 1000000.0f));

			if (_v_att_sp->pitch_body <= (PITCH_TRANSITION_FRONT_P2 - 0.2f)) {
				_v_att_sp->pitch_body = PITCH_TRANSITION_FRONT_P2 - 0.2f;
			}

		}

		_v_att_sp->thrust = _thrust_transition;

		/** start blending MC and FW controls from pitch -45 to pitch -70 for smooth control takeover*/

		//_mc_roll_weight = 1.0f - 1.0f * ((float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tailsitter.front_trans_dur_p2 * 1000000.0f));
		//_mc_pitch_weight = 1.0f - 1.0f * ((float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tailsitter.front_trans_dur_p2 * 1000000.0f));


		_mc_roll_weight = 0.0f;
		_mc_pitch_weight = 0.0f;

		/** keep yaw disabled */
		_mc_yaw_weight = 0.0f;


	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {

		if (!flag_idle_mc) {
			set_idle_mc();
			flag_idle_mc = true;
		}

		/** create time dependant pitch angle set point stating at -pi/2 + 0.2 rad overlap over the switch value*/
		_v_att_sp->pitch_body = M_PI_2_F + _pitch_transition_start + fabsf(PITCH_TRANSITION_BACK + 1.57f) *
					(float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_tailsitter.back_trans_dur * 1000000.0f);
		_v_att_sp->pitch_body = math::constrain(_v_att_sp->pitch_body , -2.0f , PITCH_TRANSITION_BACK + 0.2f);

		//  throttle value is decreesed
		_v_att_sp->thrust = _thrust_transition_start * 0.9f;

		/** keep yaw disabled */
		_mc_yaw_weight = 0.0f;

		/** smoothly move control weight to MC */
		_mc_roll_weight = 1.0f * (float)hrt_elapsed_time(&_vtol_schedule.transition_start) /
				  (_params_tailsitter.back_trans_dur * 1000000.0f);
		_mc_pitch_weight = 1.0f * (float)hrt_elapsed_time(&_vtol_schedule.transition_start) /
				   (_params_tailsitter.back_trans_dur * 1000000.0f);

	}




	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_pitch_weight = math::constrain(_mc_pitch_weight, 0.0f, 1.0f);

	// compute desired attitude and thrust setpoint for the transition

	_v_att_sp->timestamp = hrt_absolute_time();
	_v_att_sp->roll_body = 0.0f;
	_v_att_sp->yaw_body = _yaw_transition;
	_v_att_sp->R_valid = true;

	math::Matrix<3, 3> R_sp;
	R_sp.from_euler(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body);
	memcpy(&_v_att_sp->R_body[0], R_sp.data, sizeof(_v_att_sp->R_body));

	math::Quaternion q_sp;
	q_sp.from_dcm(R_sp);
	memcpy(&_v_att_sp->q_d[0], &q_sp.data[0], sizeof(_v_att_sp->q_d));
}

void Tailsitter::waiting_on_tecs()
{
	// copy the last trust value from the front transition
	_v_att_sp->thrust = _thrust_transition;
}

void Tailsitter::calc_tot_airspeed()
{
	float airspeed = math::max(1.0f, _airspeed->true_airspeed_m_s);	// prevent numerical drama
	// calculate momentary power of one engine
	float P = _batt_status->voltage_filtered_v * _batt_status->current_a / _params->vtol_motor_count;
	P = math::constrain(P, 1.0f, _params->power_max);
	// calculate prop efficiency
	float power_factor = 1.0f - P * _params->prop_eff / _params->power_max;
	float eta = (1.0f / (1 + expf(-0.4f * power_factor * airspeed)) - 0.5f) * 2.0f;
	eta = math::constrain(eta, 0.001f, 1.0f);	// live on the safe side
	// calculate induced airspeed by propeller
	float v_ind = (airspeed / eta - airspeed) * 2.0f;
	// calculate total airspeed
	float airspeed_raw = airspeed + v_ind;
	// apply low-pass filter
	_airspeed_tot = _params->arsp_lp_gain * (_airspeed_tot - airspeed_raw) + airspeed_raw;
}

void Tailsitter::scale_mc_output()
{
	// scale around tuning airspeed
	float airspeed;
	calc_tot_airspeed();	// estimate air velocity seen by elevons

	// if airspeed is not updating, we assume the normal average speed
	if (bool nonfinite = !PX4_ISFINITE(_airspeed->true_airspeed_m_s) ||
			     hrt_elapsed_time(&_airspeed->timestamp) > 1e6) {
		airspeed = _params->mc_airspeed_trim;

		if (nonfinite) {
			perf_count(_nonfinite_input_perf);
		}

	} else {
		airspeed = _airspeed_tot;
		airspeed = math::constrain(airspeed, _params->mc_airspeed_min, _params->mc_airspeed_max);
	}

	_vtol_vehicle_status->airspeed_tot = airspeed;	// save value for logging
	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	float airspeed_scaling = _params->mc_airspeed_trim / ((airspeed < _params->mc_airspeed_min) ? _params->mc_airspeed_min :
				 airspeed);
	_actuators_mc_in->control[1] = math::constrain(_actuators_mc_in->control[1] * airspeed_scaling * airspeed_scaling,
				       -1.0f, 1.0f);
}

void Tailsitter::update_mc_state()
{
	VtolType::update_mc_state();

	// set idle speed for rotary wing mode
	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}
}

void Tailsitter::update_fw_state()
{
	VtolType::update_fw_state();

	if (flag_idle_mc) {
		set_idle_fw();
		flag_idle_mc = false;
	}
}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	switch (_vtol_mode) {
	case ROTARY_WING:
		_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		_actuators_out_1->timestamp = _actuators_mc_in->timestamp;

		if (_params->elevons_mc_lock == 1) {
			_actuators_out_1->control[0] = 0;
			_actuators_out_1->control[1] = 0;

		} else {
			// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_YAW];	//roll elevon
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
				_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];	//pitch elevon
		}

		break;

	case FIXED_WING:
		// in fixed wing mode we use engines only for providing thrust, no moments are generated
		_actuators_out_0->timestamp = _actuators_fw_in->timestamp;
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = 0;
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];	// roll elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim;	// pitch elevon
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];	// yaw
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];	// throttle
		break;

	case TRANSITION:
		// in transition engines are mixed by weight (BACK TRANSITION ONLY)
		_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
		_actuators_out_1->timestamp = _actuators_mc_in->timestamp;
		_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]
				* _mc_roll_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
				_mc_yaw_weight;
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

		// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL]
				* (1 - _mc_roll_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
		// **LATER** + (_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) *(1 - _mc_pitch_weight);
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
		break;

	case EXTERNAL:
		// not yet implemented, we are switching brute force at the moment
		break;
	}
}
