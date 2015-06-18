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

 Tiltrotor::Tiltrotor ():
 flag_max_mc(false)
 {
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_params_handles_tiltrotor.front_trans_dur = param_find("VT_F_TRANS_DUR");
	_params_handles_tiltrotor.back_trans_dur = param_find("VT_B_TRANS_DUR");
	_params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC");
	_params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS");
	_params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW");
	_params_handles_tiltrotor.airspeed_trans = param_find("VT_ARSP_TRANS");
	_params_handles_tiltrotor.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
 }

 Tiltrotor::~Tiltrotor() {

 }

int
Tiltrotor::parameters_update()
{
	float v;
	int l;

	/* vtol duration of a front transition */
	param_get(_params_handles_tiltrotor.front_trans_dur, &v);
	_params_tiltrotor.front_trans_dur = math::constrain(v,1.0f,5.0f);

	/* vtol duration of a back transition */
	param_get(_params_handles_tiltrotor.back_trans_dur, &v);
	_params_tiltrotor.back_trans_dur = math::constrain(v,0.0f,5.0f);

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

	/* vtol lock elevons in multicopter */
	param_get(_params_handles_tiltrotor.elevons_mc_lock, &l);
	_params_tiltrotor.elevons_mc_lock = l;

	return OK;
}

 void Tiltrotor::update_vtol_state()
 {
	if (_manual_control_sp.aux1 < 0.0f && _vtol_schedule.flight_mode == MC_MODE) {
		// mc mode
		_vtol_schedule.flight_mode 	= MC_MODE;
		_tilt_control 			= _params_tiltrotor.tilt_mc;
	} else if (_manual_control_sp.aux1 < 0.0f && _vtol_schedule.flight_mode == FW_MODE) {
		_vtol_schedule.flight_mode 	= TRANSITION_BACK;
		flag_max_mc = true;
		_vtol_schedule.transition_start = hrt_absolute_time();
	} else if (_manual_control_sp.aux1 >= 0.0f && _vtol_schedule.flight_mode == MC_MODE) {
		// instant of doeing a front-transition
		_vtol_schedule.flight_mode 	= TRANSITION_FRONT;
		_vtol_schedule.transition_start = hrt_absolute_time();
	} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT && _manual_control_sp.aux1 > 0.0f) {
		// check if we have reached airspeed to switch to fw mode
		if (_airspeed.true_airspeed_m_s >= _params_tiltrotor.airspeed_trans) {
			_vtol_schedule.flight_mode = FW_MODE;
			// rotate rotors forward
			_tilt_control = _params_tiltrotor.tilt_fw;
		}
	} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT && _manual_control_sp.aux1 < 0.0f) {
		// failsave into mc mode
		_vtol_schedule.flight_mode = MC_MODE;
		_tilt_control = _params_tiltrotor.tilt_mc;
	}else if (_vtol_schedule.flight_mode == TRANSITION_BACK && _manual_control_sp.aux1 < 0.0f) {
		if (_tilt_control >= _params_tiltrotor.tilt_mc) {
			_vtol_schedule.flight_mode = MC_MODE;
			_tilt_control = _params_tiltrotor.tilt_mc;
		}
	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK && _manual_control_sp.aux1 > 0.0f) {
		// failsave into fw mode
		_vtol_schedule.flight_mode = FW_MODE;
		_tilt_control = _params_tiltrotor.tilt_fw;
		flag_max_mc = true;
	}

	// set correct control state according to schedule
	switch(_vtol_schedule.flight_mode) {
		case MC_MODE:
		case TRANSITION_FRONT:
			_vtol_mode = ROTARY_WING;
			break;
		case FW_MODE:
		case TRANSITION_BACK:
			_vtol_mode = FIXED_WING;
			break;
	}
 }

 void Tiltrotor::update_mc_state()
{
	if (!flag_max_mc) {
		set_max_mc();
		flag_max_mc = true;
	}
}

 void Tiltrotor::process_mc_data()
{
	fill_mc_att_control_output();
	fill_mc_att_rates_sp();
}

 void Tiltrotor::update_fw_state()
{
	if (flag_max_mc) {
		if (_vtol_schedule.flight_mode == TRANSITION_BACK) {
			set_max_fw(1200);
			set_idle_mc();
		} else {
			set_max_fw(950);
			set_idle_fw();
		}
		flag_max_mc = false;
	}
 }

void Tiltrotor::process_fw_data()
{
	fill_fw_att_control_output();
	fill_fw_att_rates_sp();
}

void Tiltrotor::update_transition_state() {
	if (_vtol_schedule.flight_mode == TRANSITION_FRONT) {
		// tilt rotors forward up to certain angle
		if (_tilt_control <= _params_tiltrotor.tilt_transition) {
			_tilt_control = _params_tiltrotor.tilt_mc +  fabsf(_params_tiltrotor.tilt_transition - _params_tiltrotor.tilt_mc)*(float)hrt_elapsed_time(&_vtol_schedule.transition_start)/(_params_tiltrotor.front_trans_dur*1000000.0f);
		}
	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) {
		// tilt rotors forward up to certain angle
		if (_tilt_control > _params_tiltrotor.tilt_transition) {
			_tilt_control = _params_tiltrotor.tilt_fw -  fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition)*(float)hrt_elapsed_time(&_vtol_schedule.transition_start)/(_params_tiltrotor.back_trans_dur*1000000.0f);
		}
	}
}

 void Tiltrotor::update_external_state() {

 }

 /**
* Prepare message to acutators with data from mc attitude controller.
*/
void Tiltrotor::fill_mc_att_control_output()
{
	_actuators_out_0.control[0] = _actuators_mc_in.control[0];
	_actuators_out_0.control[1] = _actuators_mc_in.control[1];
	_actuators_out_0.control[2] = _actuators_mc_in.control[2];
	_actuators_out_0.control[3] = _actuators_mc_in.control[3];

	if(_params_tiltrotor.elevons_mc_lock == 1) {
		_actuators_out_1.control[0] = 0;	//roll elevon locked
		_actuators_out_1.control[1] = 0;	//pitch elevon locked
	} else {
		_actuators_out_1.control[0] = _actuators_mc_in.control[2];	//roll elevon
		_actuators_out_1.control[1] = _actuators_mc_in.control[1];	//pitch elevon
	}

	_actuators_out_1.control[4] = _tilt_control;	// for tilt-rotor control
}

/**
* Prepare message to acutators with data from fw attitude controller.
*/
void Tiltrotor::fill_fw_att_control_output()
{
	/*For the first test in fw mode, only use engines for thrust!!!*/
	_actuators_out_0.control[0] = 0;
	_actuators_out_0.control[1] = 0;
	_actuators_out_0.control[2] = 0;
	_actuators_out_0.control[3] = _actuators_fw_in.control[3];
	/*controls for the elevons */
	_actuators_out_1.control[0] = -_actuators_fw_in.control[0];	// roll elevon
	_actuators_out_1.control[1] = _actuators_fw_in.control[1] + _params.fw_pitch_trim;	// pitch elevon
	// unused now but still logged
	_actuators_out_1.control[2] = _actuators_fw_in.control[2];	// yaw
	_actuators_out_1.control[3] = _actuators_fw_in.control[3];	// throttle
	_actuators_out_1.control[4] = _tilt_control;
}

/**
* Kill rear motors for the FireFLY6 when in fw mode.
*/
void
Tiltrotor::set_max_fw(unsigned pwm_value)
{
	int ret;
	unsigned servo_count;
	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = open(dev, 0);

	if (fd < 0) {err(1, "can't open %s", dev);}

	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	for (unsigned i = 0; i < _params.vtol_motor_count; i++) {
		if (i == 2 || i == 3) {
			pwm_values.values[i] = pwm_value;
		} else {
			pwm_values.values[i] = 2000;
		}
		pwm_values.channel_count = _params.vtol_motor_count;
	}

	ret = ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {errx(ret, "failed setting max values");}

	close(fd);
}

void
Tiltrotor::set_max_mc()
{
	int ret;
	unsigned servo_count;
	char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = open(dev, 0);

	if (fd < 0) {err(1, "can't open %s", dev);}

	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	for (unsigned i = 0; i < _params.vtol_motor_count; i++) {
		pwm_values.values[i] = 2000;
		pwm_values.channel_count = _params.vtol_motor_count;
	}

	ret = ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {errx(ret, "failed setting max values");}

	close(fd);
}
