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
* @file vtol_type.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/

#include "vtol_type.h"
#include "vtol_att_control_main.h"

#include <cfloat>
#include <px4_defines.h>
#include <matrix/math.hpp>

VtolType::VtolType(VtolAttitudeControl *att_controller) :
	_attc(att_controller),
	_vtol_mode(ROTARY_WING)
{
	_v_att = _attc->get_att();
	_v_att_sp = _attc->get_att_sp();
	_mc_virtual_att_sp = _attc->get_mc_virtual_att_sp();
	_fw_virtual_att_sp = _attc->get_fw_virtual_att_sp();
	_v_rates_sp = _attc->get_rates_sp();
	_mc_virtual_v_rates_sp = _attc->get_mc_virtual_rates_sp();
	_fw_virtual_v_rates_sp = _attc->get_fw_virtual_rates_sp();
	_manual_control_sp = _attc->get_manual_control_sp();
	_v_control_mode = _attc->get_control_mode();
	_vtol_vehicle_status = _attc->get_vtol_vehicle_status();
	_actuators_out_0 = _attc->get_actuators_out0();
	_actuators_out_1 = _attc->get_actuators_out1();
	_actuators_mc_in = _attc->get_actuators_mc_in();
	_actuators_fw_in = _attc->get_actuators_fw_in();
	_armed = _attc->get_armed();
	_local_pos = _attc->get_local_pos();
	_airspeed = _attc->get_airspeed();
	_batt_status = _attc->get_batt_status();
	_tecs_status = _attc->get_tecs_status();
	_land_detected = _attc->get_land_detected();
	_params = _attc->get_params();

	flag_idle_mc = true;
}

VtolType::~VtolType()
{

}

/**
* Adjust idle speed for mc mode.
*/
void VtolType::set_idle_mc()
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	unsigned servo_count;
	int ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	unsigned pwm_value = _params->idle_pwm_mc;
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {
		pwm_values.values[i] = pwm_value;
		pwm_values.channel_count++;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_WARN("failed setting min values");
	}

	px4_close(fd);

	flag_idle_mc = true;
}

/**
* Adjust idle speed for fw mode.
*/
void VtolType::set_idle_fw()
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	struct pwm_output_values pwm_values;

	memset(&pwm_values, 0, sizeof(pwm_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {

		pwm_values.values[i] = PWM_MOTOR_OFF;
		pwm_values.channel_count++;
	}

	int ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_WARN("failed setting min values");
	}

	px4_close(fd);
}

void VtolType::update_mc_state()
{
	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
}

void VtolType::update_fw_state()
{
	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
	_mc_roll_weight = 0.0f;
	_mc_pitch_weight = 0.0f;
	_mc_yaw_weight = 0.0f;

	// tecs didn't publish an update yet after the transition
	if (_tecs_status->timestamp < _trans_finished_ts) {
		_tecs_running = false;

	} else if (!_tecs_running) {
		_tecs_running = true;
		_tecs_running_ts = hrt_absolute_time();
	}

	// TECS didn't publish yet or the position controller didn't publish yet AFTER tecs
	// only wait on TECS we're in a mode where it is actually running
	if ((!_tecs_running || (_tecs_running && _fw_virtual_att_sp->timestamp <= _tecs_running_ts))
	    && _v_control_mode->flag_control_altitude_enabled) {

		waiting_on_tecs();
	}

	check_quadchute_condition();
}

void VtolType::update_transition_state()
{
	check_quadchute_condition();
}

bool VtolType::can_transition_on_ground()
{
	return !_armed->armed || _land_detected->landed;
}

void VtolType::check_quadchute_condition()
{

	if (_armed->armed && !_land_detected->landed) {
		matrix::Eulerf euler = matrix::Quatf(_v_att->q);

		// fixed-wing minimum altitude
		if (_params->fw_min_alt > FLT_EPSILON) {

			if (-(_local_pos->z) < _params->fw_min_alt) {
				_attc->abort_front_transition("Minimum altitude breached");
			}
		}

		// fixed-wing maximum pitch angle
		if (_params->fw_qc_max_pitch > 0) {

			if (fabsf(euler.theta()) > fabsf(math::radians(_params->fw_qc_max_pitch))) {
				_attc->abort_front_transition("Maximum pitch angle exceeded");
			}
		}

		// fixed-wing maximum roll angle
		if (_params->fw_qc_max_roll > 0) {

			if (fabsf(euler.phi()) > fabsf(math::radians(_params->fw_qc_max_roll))) {
				_attc->abort_front_transition("Maximum roll angle exceeded");
			}
		}
	}

}
