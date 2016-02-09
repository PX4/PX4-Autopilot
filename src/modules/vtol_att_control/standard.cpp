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
 * @file standard.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst		<bapstroman@gmail.com>
 * @author Andreas Antener	<andreas@uaventure.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
*/

#include "standard.h"
#include "vtol_att_control_main.h"

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc),
	_flag_enable_mc_motors(true),
	_pusher_throttle(0.0f),
	_airspeed_trans_blend_margin(0.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
	_mc_throttle_weight = 1.0f;

	_params_handles_standard.front_trans_dur = param_find("VT_F_TRANS_DUR");
	_params_handles_standard.back_trans_dur = param_find("VT_B_TRANS_DUR");
	_params_handles_standard.pusher_trans = param_find("VT_TRANS_THR");
	_params_handles_standard.airspeed_blend = param_find("VT_ARSP_BLEND");
	_params_handles_standard.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_standard.front_trans_timeout = param_find("VT_TRANS_TIMEOUT");
}

Standard::~Standard()
{
}

int
Standard::parameters_update()
{
	float v;

	/* duration of a forwards transition to fw mode */
	param_get(_params_handles_standard.front_trans_dur, &v);
	_params_standard.front_trans_dur = math::constrain(v, 0.0f, 5.0f);

	/* duration of a back transition to mc mode */
	param_get(_params_handles_standard.back_trans_dur, &v);
	_params_standard.back_trans_dur = math::constrain(v, 0.0f, 5.0f);

	/* target throttle value for pusher motor during the transition to fw mode */
	param_get(_params_handles_standard.pusher_trans, &v);
	_params_standard.pusher_trans = math::constrain(v, 0.0f, 5.0f);

	/* airspeed at which we should switch to fw mode */
	param_get(_params_handles_standard.airspeed_trans, &v);
	_params_standard.airspeed_trans = math::constrain(v, 1.0f, 20.0f);

	/* airspeed at which we start blending mc/fw controls */
	param_get(_params_handles_standard.airspeed_blend, &v);
	_params_standard.airspeed_blend = math::constrain(v, 0.0f, 20.0f);

	_airspeed_trans_blend_margin = _params_standard.airspeed_trans - _params_standard.airspeed_blend;

	/* timeout for transition to fw mode */
	param_get(_params_handles_standard.front_trans_timeout, &_params_standard.front_trans_timeout);

	return OK;
}

void Standard::update_vtol_state()
{
	parameters_update();

	/* After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	 * For the back transition the pusher motor is immediately stopped and rotors reactivated.
	 */

	if (!_attc->is_fixed_wing_requested()) {
		// the transition to fw mode switch is off
		if (_vtol_schedule.flight_mode == MC_MODE) {
			// in mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			_mc_yaw_weight = 1.0f;
			_mc_throttle_weight = 1.0f;

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// transition to mc mode
			_vtol_schedule.flight_mode = TRANSITION_TO_MC;
			_flag_enable_mc_motors = true;
			_vtol_schedule.transition_start = hrt_absolute_time();

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			_mc_yaw_weight = 1.0f;
			_mc_throttle_weight = 1.0f;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// transition to MC mode if transition time has passed
			if (hrt_elapsed_time(&_vtol_schedule.transition_start) >
			    (_params_standard.back_trans_dur * 1000000.0f)) {
				_vtol_schedule.flight_mode = MC_MODE;
			}
		}

		// the pusher motor should never be powered when in or transitioning to mc mode
		_pusher_throttle = 0.0f;

	} else {
		// the transition to fw mode switch is on
		if (_vtol_schedule.flight_mode == MC_MODE) {
			// start transition to fw mode
			_vtol_schedule.flight_mode = TRANSITION_TO_FW;
			_vtol_schedule.transition_start = hrt_absolute_time();

		} else if (_vtol_schedule.flight_mode == FW_MODE) {
			// in fw mode
			_vtol_schedule.flight_mode = FW_MODE;
			_mc_roll_weight = 0.0f;
			_mc_pitch_weight = 0.0f;
			_mc_yaw_weight = 0.0f;
			_mc_throttle_weight = 0.0f;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// continue the transition to fw mode while monitoring airspeed for a final switch to fw mode
			if (_airspeed->true_airspeed_m_s >= _params_standard.airspeed_trans || !_armed->armed) {
				_vtol_schedule.flight_mode = FW_MODE;
				// we can turn off the multirotor motors now
				_flag_enable_mc_motors = false;
				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// transitioning to mc mode & transition switch on - failsafe back into fw mode
			_vtol_schedule.flight_mode = FW_MODE;
		}
	}

	// map specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
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

void Standard::update_transition_state()
{
	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
		if (_params_standard.front_trans_dur <= 0.0f) {
			// just set the final target throttle value
			_pusher_throttle = _params_standard.pusher_trans;

		} else if (_pusher_throttle <= _params_standard.pusher_trans) {
			// ramp up throttle to the target throttle value
			_pusher_throttle = _params_standard.pusher_trans *
					   (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_standard.front_trans_dur * 1000000.0f);
		}

		// do blending of mc and fw controls if a blending airspeed has been provided
		if (_airspeed_trans_blend_margin > 0.0f && _airspeed->true_airspeed_m_s >= _params_standard.airspeed_blend) {
			float weight = 1.0f - fabsf(_airspeed->true_airspeed_m_s - _params_standard.airspeed_blend) /
				       _airspeed_trans_blend_margin;
			_mc_roll_weight = weight;
			_mc_pitch_weight = weight;
			_mc_yaw_weight = weight;
			_mc_throttle_weight = weight;

		} else {
			// at low speeds give full weight to mc
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			_mc_yaw_weight = 1.0f;
			_mc_throttle_weight = 1.0f;
		}

		// check front transition timeout
		if (_params_standard.front_trans_timeout > FLT_EPSILON) {
			if ( (float)hrt_elapsed_time(&_vtol_schedule.transition_start) > (_params_standard.front_trans_timeout * 1000000.0f)) {
				// transition timeout occured, abort transition
				_attc->abort_front_transition();
			}
		}

	} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
		// continually increase mc attitude control as we transition back to mc mode
		if (_params_standard.back_trans_dur > 0.0f) {
			float weight = (float)hrt_elapsed_time(&_vtol_schedule.transition_start) / (_params_standard.back_trans_dur *
					1000000.0f);
			_mc_roll_weight = weight;
			_mc_pitch_weight = weight;
			_mc_yaw_weight = weight;
			_mc_throttle_weight = weight;

		} else {
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			_mc_yaw_weight = 1.0f;
			_mc_throttle_weight = 1.0f;
		}

		// in fw mode we need the multirotor motors to stop spinning, in backtransition mode we let them spin up again
		if (_flag_enable_mc_motors) {
			set_max_mc(2000);
			set_idle_mc();
			_flag_enable_mc_motors = false;
		}
	}

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_pitch_weight = math::constrain(_mc_pitch_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);
}

void Standard::update_fw_state()
{
	VtolType::update_fw_state();

	// in fw mode we need the multirotor motors to stop spinning, in backtransition mode we let them spin up again
	if (!_flag_enable_mc_motors) {
		set_max_mc(950);
		set_idle_fw();  // force them to stop, not just idle
		_flag_enable_mc_motors = true;
	}
}

/**
 * Prepare message to acutators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	/* multirotor controls */
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]
			* _mc_roll_weight;	// roll
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;	// pitch
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
			_mc_yaw_weight;	// yaw
	_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;	// throttle

	/* fixed wing controls */
	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;
	_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL]
			* (1 - _mc_roll_weight);	//roll
	_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
		(_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) * (1 - _mc_pitch_weight);	//pitch
	_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = _actuators_fw_in->control[actuator_controls_s::INDEX_YAW]
			* (1 - _mc_yaw_weight);	// yaw

	// set the fixed wing throttle control
	if (_vtol_schedule.flight_mode == FW_MODE && _armed->armed) {
		// take the throttle value commanded by the fw controller
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

	} else {
		// otherwise we may be ramping up the throttle during the transition to fw mode
		_actuators_out_1->control[actuator_controls_s::INDEX_THROTTLE] = _pusher_throttle;
	}
}

void
Standard::waiting_on_fw_ctl()
{
	_v_att_sp->thrust = _pusher_throttle;
};

/**
* Disable all multirotor motors when in fw mode.
*/
void
Standard::set_max_mc(unsigned pwm_value)
{
	int ret;
	unsigned servo_count;
	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {PX4_WARN("can't open %s", dev);}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {
		pwm_values.values[i] = pwm_value;
		pwm_values.channel_count = _params->vtol_motor_count;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {PX4_WARN("failed setting max values");}

	px4_close(fd);
}
