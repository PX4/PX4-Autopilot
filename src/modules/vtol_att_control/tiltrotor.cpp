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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc),
	_rear_motors(ENABLED),
	_tilt_control(0.0f),
	_min_front_trans_dur(0.5f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tiltrotor.front_trans_dur = param_find("VT_F_TRANS_DUR");
	_params_handles_tiltrotor.back_trans_dur = param_find("VT_B_TRANS_DUR");
	_params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC");
	_params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS");
	_params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW");
	_params_handles_tiltrotor.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_tiltrotor.airspeed_blend_start = param_find("VT_ARSP_BLEND");
	_params_handles_tiltrotor.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
	_params_handles_tiltrotor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");
	_params_handles_tiltrotor.fw_motors_off = param_find("VT_FW_MOT_OFFID");
}

Tiltrotor::~Tiltrotor()
{

}

int
Tiltrotor::parameters_update()
{
	float v;
	int l;

	/* motors that must be turned off when in fixed wing mode */
	param_get(_params_handles_tiltrotor.fw_motors_off, &l);
	_params_tiltrotor.fw_motors_off = get_motor_off_channels(l);


	/* vtol duration of a front transition */
	param_get(_params_handles_tiltrotor.front_trans_dur, &v);
	_params_tiltrotor.front_trans_dur = math::constrain(v, 1.0f, 5.0f);

	/* vtol duration of a back transition */
	param_get(_params_handles_tiltrotor.back_trans_dur, &v);
	_params_tiltrotor.back_trans_dur = math::constrain(v, 0.0f, 5.0f);

	/* vtol tilt mechanism position in mc mode */
	param_get(_params_handles_tiltrotor.tilt_mc, &v);
	_params_tiltrotor.tilt_mc = v;

	/* vtol tilt mechanism position in transition mode */
	param_get(_params_handles_tiltrotor.tilt_transition, &v);
	_params_tiltrotor.tilt_transition = v;

	/* vtol tilt mechanism position in fw mode */
	param_get(_params_handles_tiltrotor.tilt_fw, &v);
	_params_tiltrotor.tilt_fw = v;

	/* vtol airspeed at which it is ok to switch to fw mode */
	param_get(_params_handles_tiltrotor.airspeed_trans, &v);
	_params_tiltrotor.airspeed_trans = v;

	/* vtol airspeed at which we start blending mc/fw controls */
	param_get(_params_handles_tiltrotor.airspeed_blend_start, &v);
	_params_tiltrotor.airspeed_blend_start = v;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles_tiltrotor.elevons_mc_lock, &l);
	_params_tiltrotor.elevons_mc_lock = l;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tiltrotor.front_trans_dur_p2, &v);
	_params_tiltrotor.front_trans_dur_p2 = v;

	/* avoid parameters which will lead to zero division in the transition code */
	_params_tiltrotor.front_trans_dur = math::max(_params_tiltrotor.front_trans_dur, _min_front_trans_dur);

	if (_params_tiltrotor.airspeed_trans < _params_tiltrotor.airspeed_blend_start + 1.0f) {
		_params_tiltrotor.airspeed_trans = _params_tiltrotor.airspeed_blend_start + 1.0f;
	}

	return OK;
}

int Tiltrotor::get_motor_off_channels(int channels)
{
	int channel_bitmap = 0;

	int channel;

	for (int i = 0; i < _params->vtol_motor_count; ++i) {
		channel = channels % 10;

		if (channel == 0) {
			break;
		}

		channel_bitmap |= 1 << (channel - 1);
		channels = channels / 10;
	}

	return channel_bitmap;
}

void Tiltrotor::update_vtol_state()
{
	parameters_update();

	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_schedule.flight_mode) {
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
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:
			if (_tilt_control <= _params_tiltrotor.tilt_mc) {
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}

	} else {

		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT_P1:

			// check if we have reached airspeed to switch to fw mode
			// also allow switch if we are not armed for the sake of bench testing
			if (_airspeed->true_airspeed_m_s >= _params_tiltrotor.airspeed_trans || !_armed->armed) {
				_vtol_schedule.flight_mode = TRANSITION_FRONT_P2;
				_vtol_schedule.transition_start = hrt_absolute_time();
			}

			break;

		case TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _params_tiltrotor.tilt_fw) {
				_vtol_schedule.flight_mode = FW_MODE;
				_tilt_control = _params_tiltrotor.tilt_fw;
			}

			break;

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		break;

	case TRANSITION_FRONT_P1:
	case TRANSITION_FRONT_P2:
	case TRANSITION_BACK:
		_vtol_mode = TRANSITION;
		break;
	}
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	// make sure motors are not tilted
	_tilt_control = _params_tiltrotor.tilt_mc;

	// enable rear motors
	if (_rear_motors != ENABLED) {
		set_rear_motor_state(ENABLED);
	}

	// set idle speed for rotary wing mode
	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}
}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// make sure motors are tilted forward
	_tilt_control = _params_tiltrotor.tilt_fw;

	// disable rear motors
	if (_rear_motors != DISABLED) {
		set_rear_motor_state(DISABLED);
	}

	// adjust idle for fixed wing flight
	if (flag_idle_mc) {
		set_idle_fw();
		flag_idle_mc = false;
	}
}

void Tiltrotor::update_transition_state()
{
	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) {
		// for the first part of the transition the rear rotors are enabled
		if (_rear_motors != ENABLED) {
			set_rear_motor_state(ENABLED);
		}

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _params_tiltrotor.tilt_transition) {
			_tilt_control = _params_tiltrotor.tilt_mc +
					fabsf(_params_tiltrotor.tilt_transition - _params_tiltrotor.tilt_mc) * (float)hrt_elapsed_time(
						&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur * 1000000.0f);
		}

		// do blending of mc and fw controls
		if (_airspeed->true_airspeed_m_s >= _params_tiltrotor.airspeed_blend_start) {
			_mc_roll_weight = 0.0f;

		} else {
			// at low speeds give full weight to mc
			_mc_roll_weight = 1.0f;
		}

		// disable mc yaw control once the plane has picked up speed
		_mc_yaw_weight = 1.0f;

		if (_airspeed->true_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;
		}

		_thrust_transition = _mc_virtual_att_sp->thrust;

	} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = _params_tiltrotor.tilt_transition +
				fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition) * (float)hrt_elapsed_time(
					&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur_p2 * 1000000.0f);
		_mc_roll_weight = 0.0f;

		_thrust_transition = _mc_virtual_att_sp->thrust;

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {
		if (_rear_motors != IDLE) {
			set_rear_motor_state(IDLE);
		}

		if (!flag_idle_mc) {
			set_idle_mc();
			flag_idle_mc = true;
		}

		// tilt rotors back
		if (_tilt_control > _params_tiltrotor.tilt_mc) {
			_tilt_control = _params_tiltrotor.tilt_fw -
					fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_mc) * (float)hrt_elapsed_time(
						&_vtol_schedule.transition_start) / (_params_tiltrotor.back_trans_dur * 1000000.0f);
		}

		// set zero throttle for backtransition otherwise unwanted moments will be created
		_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		_mc_roll_weight = 0.0f;

	}

	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);

	// copy virtual attitude setpoint to real attitude setpoint (we use multicopter att sp)
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust = _thrust_transition;
}

/**
* Write data to actuator output topic.
*/
void Tiltrotor::fill_actuator_outputs()
{
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] = _actuators_mc_in->control[actuator_controls_s::INDEX_ROLL]
			* _mc_roll_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] = _actuators_mc_in->control[actuator_controls_s::INDEX_YAW] *
			_mc_yaw_weight;

	if (_vtol_schedule.flight_mode == FW_MODE) {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

	} else {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];;
	}

	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;
	_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = -_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL]
			* (1 - _mc_roll_weight);
	_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
		(_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) * (1 - _mc_pitch_weight);
	_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = _actuators_fw_in->control[actuator_controls_s::INDEX_YAW]
			* (1 - _mc_yaw_weight);	// yaw
	_actuators_out_1->control[4] = _tilt_control;
}


/**
* Set state of rear motors.
*/

void Tiltrotor::set_rear_motor_state(rear_motor_state state)
{
	int pwm_value = PWM_DEFAULT_MAX;

	// map desired rear rotor state to max allowed pwm signal
	switch (state) {
	case ENABLED:
		pwm_value = PWM_DEFAULT_MAX;
		_rear_motors = ENABLED;
		break;

	case DISABLED:
		pwm_value = PWM_MOTOR_OFF;
		_rear_motors = DISABLED;
		break;

	case IDLE:
		pwm_value = _params->idle_pwm_mc;
		_rear_motors = IDLE;
		break;
	}

	int ret;
	unsigned servo_count;
	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {PX4_WARN("can't open %s", dev);}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_max_values;
	memset(&pwm_max_values, 0, sizeof(pwm_max_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {
		if (is_motor_off_channel(i)) {
			pwm_max_values.values[i] = pwm_value;

		} else {
			pwm_max_values.values[i] = PWM_DEFAULT_MAX;
		}

		pwm_max_values.channel_count = _params->vtol_motor_count;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);

	if (ret != OK) {PX4_WARN("failed setting max values");}

	px4_close(fd);
}

bool Tiltrotor::is_motor_off_channel(const int channel)
{
	return (_params_tiltrotor.fw_motors_off >> channel) & 1;
}
