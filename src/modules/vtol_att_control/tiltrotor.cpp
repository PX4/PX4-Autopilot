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
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

static constexpr float ARSP_YAW_CTRL_DISABLE =
	7.0f;	// airspeed at which we stop controlling yaw during a front transition

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc),
	_param_tilt_mc(this, "TILT_MC"),
	_param_tilt_transition(this, "TILT_TRANS"),
	_param_tilt_fw(this, "TILT_FW"),
	_param_front_trans_time_openloop(this, "F_TR_OL_TM"),
	_param_front_trans_dur_p2(this, "TRANS_P2_DUR"),
	_param_fw_motors_off(this, "FW_MOT_OFFID"),
	_param_diff_thrust(this, "FW_DIFTHR_EN"),
	_param_diff_thrust_scale(this, "FW_DIFTHR_SC")
{
}

void
Tiltrotor::parameters_update()
{
	updateParams();

	/* motors that must be turned off when in fixed wing mode */
	_fw_motors_off = get_motor_off_channels(_param_fw_motors_off.get());
}

uint32_t Tiltrotor::get_motor_off_channels(int channels)
{
	uint32_t channel_bitmap = 0;

	for (int i = 0; i < _param_vtol_motor_count.get(); ++i) {
		uint32_t channel = channels % 10;

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
			if (_tilt_control <= _param_tilt_mc.get()) {
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

		case TRANSITION_FRONT_P1: {
				// allow switch if we are not armed for the sake of bench testing
				bool transition_to_p2 = can_transition_on_ground();

				const bool airspeed_enabled = (_param_airspeed_mode.get() == 0);

				// check if we have reached airspeed to switch to fw mode
				transition_to_p2 |= airspeed_enabled &&
						    (_airspeed->indicated_airspeed_m_s >= _param_airspeed_trans.get()) &&
						    (hrt_elapsed_time(&_vtol_schedule.transition_start) > (_param_front_trans_time_min.get() * 1e6f));

				// check if airspeed is invalid and transition by time
				transition_to_p2 |= !airspeed_enabled &&
						    (_tilt_control > _param_tilt_transition.get()) &&
						    (hrt_elapsed_time(&_vtol_schedule.transition_start) > (_param_front_trans_time_openloop.get() * 1e6f));

				if (transition_to_p2) {
					_vtol_schedule.flight_mode = TRANSITION_FRONT_P2;
					_vtol_schedule.transition_start = hrt_absolute_time();
				}

				break;
			}

		case TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _param_tilt_fw.get()) {
				_vtol_schedule.flight_mode = FW_MODE;
				_tilt_control = _param_tilt_fw.get();
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
		_vtol_mode = TRANSITION_TO_FW;
		break;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION_TO_MC;
		break;
	}
}

void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	// make sure motors are not tilted
	_tilt_control = _param_tilt_mc.get();

	// enable rear motors
	if (_rear_motors != ENABLED) {
		set_rear_motor_state(ENABLED);
	}

	// set idle speed for rotary wing mode
	if (!flag_idle_mc) {
		flag_idle_mc = enable_mc_motors();
	}
}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// make sure motors are tilted forward
	_tilt_control = _param_tilt_fw.get();

	// disable rear motors
	if (_rear_motors != DISABLED) {
		set_rear_motor_state(DISABLED);
	}

	// adjust idle for fixed wing flight
	if (flag_idle_mc) {
		flag_idle_mc = !disable_mc_motors();
	}
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();

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
		if (_tilt_control <= _param_tilt_transition.get()) {

			_tilt_control = _param_tilt_mc.get() +
					fabsf(_param_tilt_transition.get() - _param_tilt_mc.get())
					* hrt_elapsed_time(&_vtol_schedule.transition_start) / (_param_front_trans_dur.get() * 1e6f);
		}

		bool use_airspeed = (_param_airspeed_mode.get() == 0);

		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		// reduce MC controls once the plane has picked up speed
		if (use_airspeed && _airspeed->indicated_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;
		}

		if (use_airspeed && _airspeed->indicated_airspeed_m_s >= _param_airspeed_blend_start.get()) {
			_mc_roll_weight = 1.0f - (_airspeed->indicated_airspeed_m_s - _param_airspeed_blend_start.get()) /
					  (_param_airspeed_trans.get() - _param_airspeed_blend_start.get());
		}

		// without airspeed do timed weight changes
		if (!use_airspeed
		    && hrt_elapsed_time(&_vtol_schedule.transition_start) > (_param_front_trans_time_min.get() * 1e6f)) {
			_mc_roll_weight = 1.0f - (hrt_elapsed_time(&_vtol_schedule.transition_start) - _param_front_trans_time_min.get() *
						  1e6f) / (_param_front_trans_time_openloop.get() * 1e6f - _param_front_trans_time_min.get() * 1e6f);
			_mc_yaw_weight = _mc_roll_weight;
		}

		_thrust_transition = _mc_virtual_att_sp->thrust;

	} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = _param_tilt_transition.get() + fabsf(_param_tilt_fw.get() - _param_tilt_transition.get()) *
				hrt_elapsed_time(&_vtol_schedule.transition_start) / (_param_front_trans_dur_p2.get() * 1e6f);

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// ramp down rear motors (setting MAX_PWM down scales the given output into the new range)
		int rear_value = (1.0f - hrt_elapsed_time(&_vtol_schedule.transition_start) / (_param_front_trans_dur_p2.get() * 1e6f))
				 * (float)(PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + (float)PWM_DEFAULT_MIN;

		set_rear_motor_state(VALUE, rear_value);

		_thrust_transition = _mc_virtual_att_sp->thrust;

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {
		if (_rear_motors != IDLE) {
			set_rear_motor_state(IDLE);
		}

		if (!flag_idle_mc) {
			flag_idle_mc = enable_mc_motors();
		}

		// tilt rotors back
		if (_tilt_control > _param_tilt_mc.get()) {

			_tilt_control = _param_tilt_fw.get() - fabsf(_param_tilt_fw.get() - _param_tilt_mc.get()) *
					hrt_elapsed_time(&_vtol_schedule.transition_start) / (_param_back_trans_dur.get() * 1e6f);
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

		/* allow differential thrust if enabled */
		if (_param_diff_thrust.get() == 1) {
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * _param_diff_thrust_scale.get();
		}

	} else {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];
	}

	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;
	_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
		-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
	_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
		(_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _param_fw_pitch_trim.get());
	_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
		_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];	// yaw
	_actuators_out_1->control[4] = _tilt_control;
}


/**
* Set state of rear motors.
*/

void Tiltrotor::set_rear_motor_state(rear_motor_state state, int value)
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
		pwm_value = _param_idle_pwm_mc.get();
		_rear_motors = IDLE;
		break;

	case VALUE:
		pwm_value = value;
		_rear_motors = VALUE;
		break;
	}

	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	struct pwm_output_values pwm_max_values = {};

	for (int i = 0; i < _param_vtol_motor_count.get(); i++) {
		if (is_motor_off_channel(i)) {
			pwm_max_values.values[i] = pwm_value;

		} else {
			pwm_max_values.values[i] = PWM_DEFAULT_MAX;
		}

		pwm_max_values.channel_count = _param_vtol_motor_count.get();
	}

	int ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);

	if (ret != OK) {
		PX4_ERR("failed setting max values");
	}

	px4_close(fd);
}

bool Tiltrotor::is_motor_off_channel(const int channel)
{
	return (_fw_motors_off >> channel) & 1;
}
