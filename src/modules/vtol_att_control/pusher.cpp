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
 * @file pusher.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst 		<bapstroman@gmail.com>
 *
*/

#include "pusher.h"
#include "vtol_att_control_main.h"

#define ARSP_BLEND_START 8.0f	// airspeed at which we start blending mc/fw controls

Pusher::Pusher(VtolAttitudeControl *attc) :
	VtolType(attc),
	_flag_enable_mc_motors(true),
	_pusher_throttle(0.0f),
	_mc_att_ctl_weight(1.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_params_handles_pusher.front_trans_dur = param_find("VT_F_TRANS_DUR");
	_params_handles_pusher.back_trans_dur = param_find("VT_B_TRANS_DUR");
	_params_handles_pusher.pusher_mc = param_find("VT_TILT_MC");
	_params_handles_pusher.pusher_transition = param_find("VT_TILT_TRANS");
	_params_handles_pusher.pusher_fw = param_find("VT_TILT_FW");
	_params_handles_pusher.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_pusher.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
 }

Pusher::~Pusher()
{
}

int
Pusher::parameters_update()
{
	float v;
	int l;

	/* vtol duration of a front transition */
	param_get(_params_handles_pusher.front_trans_dur, &v);
	_params_pusher.front_trans_dur = math::constrain(v, 1.0f, 5.0f);

	/* vtol duration of a back transition */
	param_get(_params_handles_pusher.back_trans_dur, &v);
	_params_pusher.back_trans_dur = math::constrain(v, 0.0f, 5.0f);

	/* vtol pusher mechanism position in mc mode */
	param_get(_params_handles_pusher.pusher_mc, &v);
	_params_pusher.pusher_mc = v;

	/* vtol pusher mechanism position in transition mode */
	param_get(_params_handles_pusher.pusher_transition, &v);
	_params_pusher.pusher_transition = v;

	/* vtol pusher mechanism position in fw mode */
	param_get(_params_handles_pusher.pusher_fw, &v);
	_params_pusher.pusher_fw = v;

	/* vtol airspeed at which it is ok to switch to fw mode */
	param_get(_params_handles_pusher.airspeed_trans, &v);
	_params_pusher.airspeed_trans = v;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles_pusher.elevons_mc_lock, &l);
	_params_pusher.elevons_mc_lock = l;

	return OK;
}

void Pusher::update_vtol_state()
{
 	parameters_update();

 	/* Simple logic using a two way switch to perform transitions.
	 * After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown. 
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
 	 */

	if (_manual_control_sp->aux1 < 0.0f) {
		// the transition to fw mode switch is off
		if (_vtol_schedule.flight_mode == MC_MODE) {
			// in mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			_mc_att_ctl_weight = 1.0f;

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// transition to mc mode
			_vtol_schedule.flight_mode = TRANSITION_TO_MC;
			_flag_enable_mc_motors = true;
			_vtol_schedule.transition_start = hrt_absolute_time();

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_schedule.flight_mode = MC_MODE;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// keep transitioning to mc mode
			_vtol_schedule.flight_mode = MC_MODE;
		}

		// the pusher motor should never be powered when in or transitioning to mc mode
		_pusher_throttle = 0.0f;

	} else {
		// the transition to fw mode switch is on
		if (_vtol_schedule.flight_mode == MC_MODE) {
			// start transition to fw mode
			_vtol_schedule.flight_mode = TRANSITION_TO_FW;
			_pusher_throttle = 0.0f;	// start from zero so we can ramp up from a known min. value
			_vtol_schedule.transition_start = hrt_absolute_time();

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// in fw mode
			_vtol_schedule.flight_mode = FW_MODE;
			// use commanded fw throttle
			_pusher_throttle = _actuators_fw_in->control[3];

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// continue the transition to fw mode while monitoring airspeed for a final switch to fw mode
			if (_airspeed->true_airspeed_m_s >= _params_pusher.airspeed_trans) {
				_vtol_schedule.flight_mode = FW_MODE;
				// we can turn off the multirotor motors now
				_flag_enable_mc_motors = false;
				// don't set pusher throttle here as it's being ramped up elsewhere
			}

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// transitioning to mc mode & transition switch on - failsafe into fw mode
			_vtol_schedule.flight_mode = FW_MODE;
			_pusher_throttle = _actuators_fw_in->control[3];
		}
	}

	update_transition_state();

	// map pusher specific control phases to simple control modes
	switch(_vtol_schedule.flight_mode) {
		case MC_MODE:
			_vtol_mode = ROTARY_WING;
			break;
		case FW_MODE:
			_vtol_mode = FIXED_WING;
			break;
		case TRANSITION_TO_FW:
		case TRANSITION_TO_MC:
			_vtol_mode = TRANSITION;
			break;
	}
}

void Pusher::update_transition_state()
{
	if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
		// Ramp up throttle to the current throttle value to something above cruise throttle
		// NOTE: Probably need to be a bit more clever than this. Should we keep ramping up to full throttle during which time transition airspeed should be reached?
		if (_pusher_throttle <= _params_pusher.pusher_transition) {
			_pusher_throttle = _params_pusher.pusher_mc + fabsf(_params_pusher.pusher_transition - _params_pusher.pusher_mc) * 
						(float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_pusher.front_trans_dur * 1000000.0f);
		}

		// do blending of mc and fw controls
		if (_airspeed->true_airspeed_m_s >= ARSP_BLEND_START) {
			_mc_att_ctl_weight = 1.0f - (_airspeed->true_airspeed_m_s - ARSP_BLEND_START) / (_params_pusher.airspeed_trans - ARSP_BLEND_START);
		} else {
			// at low speeds give full weight to mc
			_mc_att_ctl_weight = 1.0f;
		}

		_mc_att_ctl_weight = math::constrain(_mc_att_ctl_weight, 0.0f, 1.0f);
	} else if (_vtol_schedule.flight_mode == FW_MODE) {
		_mc_att_ctl_weight = 0.0f;

	} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
		// continually increase mc attitude control as we transition back to mc mode
		_mc_att_ctl_weight = (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_pusher.back_trans_dur * 1000000.0f);

		// in fw mode we need the multirotor motors to stop spinning, in backtransition mode we let them spin up again	
		if (_flag_enable_mc_motors) {
			set_max_mc(2000);
			set_idle_mc();
			_flag_enable_mc_motors = false;
		}
	}
}

void Pusher::update_mc_state()
{
	// do nothing
}

void Pusher::process_mc_data()
{
	fill_mc_att_control_output();
}

 void Pusher::update_fw_state()
{
	// in fw mode we need the multirotor motors to stop spinning, in backtransition mode we let them spin up again	
	if (!_flag_enable_mc_motors) {
		set_max_mc(950);
		set_idle_fw();  // force them to stop, not just idle
		_flag_enable_mc_motors = true;
	}
 }

void Pusher::process_fw_data()
{
	fill_fw_att_control_output();
}

void Pusher::update_external_state()
{

}

 /**
* Prepare message to acutators with data from mc attitude controller.
* Note that this function will also be called for all transitions into and out of mc mode.
*/
void Pusher::fill_mc_att_control_output()
{
	_actuators_out_0->control[0] = _actuators_mc_in->control[0] * _mc_att_ctl_weight;	// roll
	_actuators_out_0->control[1] = _actuators_mc_in->control[1] * _mc_att_ctl_weight;	// pitch
	_actuators_out_0->control[2] = _actuators_mc_in->control[2] * _mc_att_ctl_weight;	// yaw
	_actuators_out_0->control[3] = _actuators_mc_in->control[3];	// throttle

	_actuators_out_1->control[0] = -_actuators_fw_in->control[0] * (1.0f - _mc_att_ctl_weight);	//roll elevon
	_actuators_out_1->control[1] = (_actuators_fw_in->control[1] + _params->fw_pitch_trim)* (1.0f -_mc_att_ctl_weight);	//pitch elevon
	
	_actuators_out_1->control[4] = _pusher_throttle;	// for pusher-rotor control
}

/**
* Prepare message to acutators with data from fw attitude controller.
*/
void Pusher::fill_fw_att_control_output()
{
	/* For the first test in fw mode, only use engines for thrust!!! */
	_actuators_out_0->control[0] = 0.0f;	// roll
	_actuators_out_0->control[1] = 0.0f;	// pitch
	_actuators_out_0->control[2] = 0.0f;	// yaw
	_actuators_out_0->control[3] = 0.0f;	// throttle
	
	/* controls for the elevons */
	_actuators_out_1->control[0] = -_actuators_fw_in->control[0];	// roll elevon
	_actuators_out_1->control[1] = _actuators_fw_in->control[1] + _params->fw_pitch_trim;	// pitch elevon

	/* controls for the pusher motor */
	_actuators_out_1->control[4] = _pusher_throttle;	

	/* unused now but still logged */
	_actuators_out_1->control[2] = _actuators_fw_in->control[2];	// yaw
}

/**
* Disable all multirotor motors when in fw mode.
*/
void
Pusher::set_max_mc(unsigned pwm_value)
{
	int ret;
	unsigned servo_count;
	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = open(dev, 0);

	if (fd < 0) {err(1, "can't open %s", dev);}

	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {
		pwm_values.values[i] = pwm_value;
		pwm_values.channel_count = _params->vtol_motor_count;
	}

	ret = ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {errx(ret, "failed setting max values");}

	close(fd);
}
