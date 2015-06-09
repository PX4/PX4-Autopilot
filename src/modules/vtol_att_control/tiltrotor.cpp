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

 Tiltrotor::Tiltrotor ()
 {

 }

 Tiltrotor::~Tiltrotor() {

 }

 void Tiltrotor::update_vtol_state() {
 }

 void Tiltrotor::update_mc_state() {
 	
 }

 void Tiltrotor::process_mc_data() {
 	// scale pitch control with total airspeed
	fill_mc_att_control_output();
	fill_mc_att_rates_sp();
 }

 void Tiltrotor::update_fw_state() {

 }

void Tiltrotor::process_fw_data() {
	fill_fw_att_control_output();
	fill_fw_att_rates_sp();
}

 void Tiltrotor::update_transition_state() {

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

	_actuators_out_1.control[0] = 0;	//roll elevon locked
	_actuators_out_1.control[1] = 0;	//pitch elevon locked

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