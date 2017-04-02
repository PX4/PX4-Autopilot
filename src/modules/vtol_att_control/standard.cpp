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

#include <float.h>

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc),
	_flag_enable_mc_motors(true),
	_pusher_throttle(0.0f),
	_airspeed_trans_blend_margin(0.0f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;
	_pusher_active = false;

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
	_params_handles_standard.front_trans_time_min = param_find("VT_TRANS_MIN_TM");
	_params_handles_standard.down_pitch_max = param_find("VT_DWN_PITCH_MAX");
	_params_handles_standard.forward_thrust_scale = param_find("VT_FWD_THRUST_SC");
	_params_handles_standard.airspeed_mode = param_find("FW_ARSP_MODE");
	_params_handles_standard.pitch_setpoint_offset = param_find("FW_PSP_OFF");
}

Standard::~Standard()
{
}

void
Standard::parameters_update()
{
	float v;
	int i;

	/* duration of a forwards transition to fw mode */
	param_get(_params_handles_standard.front_trans_dur, &v);
	_params_standard.front_trans_dur = math::constrain(v, 0.0f, 20.0f);

	/* duration of a back transition to mc mode */
	param_get(_params_handles_standard.back_trans_dur, &v);
	_params_standard.back_trans_dur = math::constrain(v, 0.0f, 20.0f);

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

	/* minimum time for transition to fw mode */
	param_get(_params_handles_standard.front_trans_time_min, &_params_standard.front_trans_time_min);

	/* maximum down pitch allowed */
	param_get(_params_handles_standard.down_pitch_max, &v);
	_params_standard.down_pitch_max = math::radians(v);

	/* scale for fixed wing thrust used for forward acceleration in multirotor mode */
	param_get(_params_handles_standard.forward_thrust_scale, &_params_standard.forward_thrust_scale);

	/* airspeed mode */
	param_get(_params_handles_standard.airspeed_mode, &i);
	_params_standard.airspeed_mode = math::constrain(i, 0, 2);

	/* pitch setpoint offset */
	param_get(_params_handles_standard.pitch_setpoint_offset, &v);
	_params_standard.pitch_setpoint_offset = math::radians(v);


}

void Standard::update_vtol_state()
{
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
			if (_vtol_vehicle_status->vtol_transition_failsafe == true) {
				// Failsafe event, engage mc motors immediately
				_vtol_schedule.flight_mode = MC_MODE;
				_flag_enable_mc_motors = true;

			} else {
				// Regular backtransition
				_vtol_schedule.flight_mode = TRANSITION_TO_MC;
				_flag_enable_mc_motors = true;
				_vtol_schedule.transition_start = hrt_absolute_time();
			}

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_FW) {
			// failsafe back to mc mode
			_vtol_schedule.flight_mode = MC_MODE;
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			_mc_yaw_weight = 1.0f;
			_mc_throttle_weight = 1.0f;

		} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// transition to MC mode if transition time has passed
			// XXX: base this on XY hold velocity of MC
			if (hrt_elapsed_time(&_vtol_schedule.transition_start) >
			    (_params_standard.back_trans_dur * 1000000.0f)) {
				_vtol_schedule.flight_mode = MC_MODE;
			}
		}

		// the pusher motor should never be powered when in or transitioning to mc mode
		_pusher_throttle = 0.0f;

	} else {
		// the transition to fw mode switch is on
		if (_vtol_schedule.flight_mode == MC_MODE || _vtol_schedule.flight_mode == TRANSITION_TO_MC) {
			// start transition to fw mode
			/* NOTE: The failsafe transition to fixed-wing was removed because it can result in an
			 * unsafe flying state. */
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
			if (((_params_standard.airspeed_mode == control_state_s::AIRSPD_MODE_DISABLED ||
			      _airspeed->indicated_airspeed_m_s >= _params_standard.airspeed_trans) &&
			     (float)hrt_elapsed_time(&_vtol_schedule.transition_start)
			     > (_params_standard.front_trans_time_min * 1000000.0f)) ||
			    can_transition_on_ground()) {

				_vtol_schedule.flight_mode = FW_MODE;
				// we can turn off the multirotor motors now
				_flag_enable_mc_motors = false;
				// don't set pusher throttle here as it's being ramped up elsewhere
				_trans_finished_ts = hrt_absolute_time();
			}

		}
	}

	// map specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		break;

	case FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		break;

	case TRANSITION_TO_FW:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case TRANSITION_TO_MC:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}

void Standard::update_transition_state()
{
	VtolType::update_transition_state();

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

		// do blending of mc and fw controls if a blending airspeed has been provided and the minimum transition time has passed
		if (_airspeed_trans_blend_margin > 0.0f &&
		    _airspeed->indicated_airspeed_m_s >= _params_standard.airspeed_blend &&
		    (float)hrt_elapsed_time(&_vtol_schedule.transition_start) > (_params_standard.front_trans_time_min * 1000000.0f)
		   ) {
			float weight = 1.0f - fabsf(_airspeed->indicated_airspeed_m_s - _params_standard.airspeed_blend) /
				       _airspeed_trans_blend_margin;
			_mc_roll_weight = weight;
			_mc_pitch_weight = weight;
			_mc_yaw_weight = weight;
			_mc_throttle_weight = weight;

			// time based blending when no airspeed sensor is set

		} else if (_params_standard.airspeed_mode == control_state_s::AIRSPD_MODE_DISABLED &&
			   (float)hrt_elapsed_time(&_vtol_schedule.transition_start) < (_params_standard.front_trans_time_min * 1000000.0f) &&
			   (float)hrt_elapsed_time(&_vtol_schedule.transition_start) > ((_params_standard.front_trans_time_min / 2.0f) *
					   1000000.0f)
			  ) {
			float weight = 1.0f - ((float)(hrt_elapsed_time(&_vtol_schedule.transition_start) - ((
							       _params_standard.front_trans_time_min / 2.0f) * 1000000.0f)) /
					       ((_params_standard.front_trans_time_min / 2.0f) * 1000000.0f));

			weight = math::constrain(weight, 0.0f, 1.0f);

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

		// ramp up FW_PSP_OFF
		_v_att_sp->pitch_body = _params_standard.pitch_setpoint_offset * (1.0f - _mc_pitch_weight);
		matrix::Quatf q_sp(matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		// check front transition timeout
		if (_params_standard.front_trans_timeout > FLT_EPSILON) {
			if ((float)hrt_elapsed_time(&_vtol_schedule.transition_start) > (_params_standard.front_trans_timeout * 1000000.0f)) {
				// transition timeout occured, abort transition
				_attc->abort_front_transition("Transition timeout");
			}
		}

	} else if (_vtol_schedule.flight_mode == TRANSITION_TO_MC) {

		// maintain FW_PSP_OFF
		_v_att_sp->pitch_body = _params_standard.pitch_setpoint_offset;
		matrix::Quatf q_sp(matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
		q_sp.copyTo(_v_att_sp->q_d);
		_v_att_sp->q_d_valid = true;

		// continually increase mc attitude control as we transition back to mc mode
		if (_params_standard.back_trans_dur > FLT_EPSILON) {
			float weight = (float)hrt_elapsed_time(&_vtol_schedule.transition_start) /
				       ((_params_standard.back_trans_dur / 2) * 1000000.0f);
			weight = math::constrain(weight, 0.0f, 1.0f);
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

void Standard::update_mc_state()
{
	VtolType::update_mc_state();

	// enable MC motors here in case we transitioned directly to MC mode
	if (_flag_enable_mc_motors) {
		set_max_mc(2000);
		set_idle_mc();
		_flag_enable_mc_motors = false;
	}

	// if the thrust scale param is zero then the pusher-for-pitch strategy is disabled and we can return
	if (_params_standard.forward_thrust_scale < FLT_EPSILON) {
		return;
	}

	matrix::Dcmf R(matrix::Quatf(_v_att->q));
	matrix::Dcmf R_sp(matrix::Quatf(_v_att_sp->q_d));
	matrix::Eulerf euler(R);
	matrix::Eulerf euler_sp(R_sp);
	_pusher_throttle = 0.0f;

	// direction of desired body z axis represented in earth frame
	matrix::Vector3f body_z_sp(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	// rotate desired body z axis into new frame which is rotated in z by the current
	// heading of the vehicle. we refer to this as the heading frame.
	matrix::Dcmf R_yaw = matrix::Eulerf(0.0f, 0.0f, -euler(2));
	body_z_sp = R_yaw * body_z_sp;
	body_z_sp.normalize();

	// calculate the desired pitch seen in the heading frame
	// this value corresponds to the amount the vehicle would try to pitch forward
	float pitch_forward = atan2f(body_z_sp(0), body_z_sp(2));

	// only allow pitching forward up to threshold, the rest of the desired
	// forward acceleration will be compensated by the pusher
	if (pitch_forward < -_params_standard.down_pitch_max) {
		// desired roll angle in heading frame stays the same
		float roll_new = -asinf(body_z_sp(1));

		_pusher_throttle = (sinf(-pitch_forward) - sinf(_params_standard.down_pitch_max))
				   * _v_att_sp->thrust * _params_standard.forward_thrust_scale;

		// limit desired pitch
		float pitch_new = -_params_standard.down_pitch_max;

		// create corrected desired body z axis in heading frame
		matrix::Dcmf R_tmp = matrix::Eulerf(roll_new, pitch_new, 0.0f);
		matrix::Vector3f tilt_new(R_tmp(0, 2), R_tmp(1, 2), R_tmp(2, 2));

		// rotate the vector into a new frame which is rotated in z by the desired heading
		// with respect to the earh frame.
		float yaw_error = _wrap_pi(euler_sp(2) - euler(2));
		matrix::Dcmf R_yaw_correction = matrix::Eulerf(0.0f, 0.0f, -yaw_error);
		tilt_new = R_yaw_correction * tilt_new;

		// now extract roll and pitch setpoints
		_v_att_sp->pitch_body = atan2f(tilt_new(0), tilt_new(2));
		_v_att_sp->roll_body = -asinf(tilt_new(1));
		R_sp = matrix::Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, euler_sp(2));
		matrix::Quatf q_sp(R_sp);
		memcpy(&_v_att_sp->q_d[0], &q_sp._data[0], sizeof(_v_att_sp->q_d));
	}

	_pusher_throttle = _pusher_throttle < 0.0f ? 0.0f : _pusher_throttle;

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
	// multirotor controls
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;

	// roll
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
	// pitch
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	// yaw
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight;
	// throttle
	_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;


	// fixed wing controls
	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;


	if (_vtol_schedule.flight_mode != MC_MODE) {

		//roll
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
		//pitch
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim;
		// yaw
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];

	} else {

		// zero outputs when inactive
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = _params->fw_pitch_trim;
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;
	}

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
Standard::waiting_on_tecs()
{
	// keep thrust from transition
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
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {
		pwm_values.values[i] = pwm_value;
		pwm_values.channel_count = _params->vtol_motor_count;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_WARN("failed setting max values");
	}

	px4_close(fd);
}
