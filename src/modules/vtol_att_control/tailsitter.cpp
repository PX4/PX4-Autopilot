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
 *
 */

 #include "tailsitter.h"
 #include "vtol_att_control_main.h"

Tailsitter::Tailsitter (VtolAttitudeControl *att_controller) :
VtolType(att_controller),
_airspeed_tot(0),
_loop_perf(perf_alloc(PC_ELAPSED, "vtol_att_control-tailsitter")),
_nonfinite_input_perf(perf_alloc(PC_COUNT, "vtol att control-tailsitter nonfinite input"))
{

}

Tailsitter::~Tailsitter()
{

}

void Tailsitter::update_vtol_state()
{
	// simply switch between the two modes
	if (!_attc->is_fixed_wing_requested()) {
		_vtol_mode = ROTARY_WING;
	} else {
		_vtol_mode = FIXED_WING;
	}
}

void Tailsitter::update_mc_state()
{
 	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}
}

void Tailsitter::update_fw_state()
{
	if (flag_idle_mc) {
		set_idle_fw();
		flag_idle_mc = false;
	}
}

void Tailsitter::update_transition_state()
{

}

void Tailsitter::update_external_state()
{

}

 void Tailsitter::calc_tot_airspeed()
 {
	float airspeed = math::max(1.0f, _airspeed->true_airspeed_m_s);	// prevent numerical drama
	// calculate momentary power of one engine
	float P = _batt_status->voltage_filtered_v * _batt_status->current_a / _params->vtol_motor_count;
	P = math::constrain(P,1.0f,_params->power_max);
	// calculate prop efficiency
	float power_factor = 1.0f - P*_params->prop_eff/_params->power_max;
	float eta = (1.0f/(1 + expf(-0.4f * power_factor * airspeed)) - 0.5f)*2.0f;
	eta = math::constrain(eta,0.001f,1.0f);	// live on the safe side
	// calculate induced airspeed by propeller
	float v_ind = (airspeed/eta - airspeed)*2.0f;
	// calculate total airspeed
	float airspeed_raw = airspeed + v_ind;
	// apply low-pass filter
	_airspeed_tot = _params->arsp_lp_gain * (_airspeed_tot - airspeed_raw) + airspeed_raw;
}

void
Tailsitter::scale_mc_output()
{
	// scale around tuning airspeed
	float airspeed;
	calc_tot_airspeed();	// estimate air velocity seen by elevons
	// if airspeed is not updating, we assume the normal average speed
	if (bool nonfinite = !isfinite(_airspeed->true_airspeed_m_s) ||
	    hrt_elapsed_time(&_airspeed->timestamp) > 1e6) {
		airspeed = _params->mc_airspeed_trim;
		if (nonfinite) {
			perf_count(_nonfinite_input_perf);
		}
	} else {
		airspeed = _airspeed_tot;
		airspeed = math::constrain(airspeed,_params->mc_airspeed_min, _params->mc_airspeed_max);
	}

	_vtol_vehicle_status->airspeed_tot = airspeed;	// save value for logging
	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	float airspeed_scaling = _params->mc_airspeed_trim / ((airspeed < _params->mc_airspeed_min) ? _params->mc_airspeed_min : airspeed);
	_actuators_mc_in->control[1] = math::constrain(_actuators_mc_in->control[1]*airspeed_scaling*airspeed_scaling,-1.0f,1.0f);
}

/**
* Write data to actuator output topic.
*/
void Tailsitter::fill_actuator_outputs()
{
	switch(_vtol_mode) {
		case ROTARY_WING:
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL];
			_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];
			_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];
			_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];

			if (_params->elevons_mc_lock == 1) {
				_actuators_out_1->control[0] = 0;
				_actuators_out_1->control[1] = 0;
			} else {
				// NOTE: There is no mistake in the line below, multicopter yaw axis is controlled by elevon roll actuation!
				_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW];	//roll elevon
				_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = _actuators_mc_in->control[actuator_controls_s::INDEX_PITCH];	//pitch elevon
			}
			break;
		case FIXED_WING:
			// in fixed wing mode we use engines only for providing thrust, no moments are generated
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = 0;
			_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] = 0;
			_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = 0;
			_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

			_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];	// roll elevon
			_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = _actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim;	// pitch elevon
			_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = _actuators_fw_in->control[actuator_controls_s::INDEX_YAW];	// yaw
			_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] = _actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];	// throttle
			break;
		case TRANSITION:
		case EXTERNAL:
			// not yet implemented, we are switching brute force at the moment
			break;
	}
}
