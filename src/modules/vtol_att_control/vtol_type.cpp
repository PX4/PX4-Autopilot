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
	SuperBlock(nullptr, "VT"),
	_attc(att_controller),
	_param_idle_pwm_mc(this, "IDLE_PWM_MC"),
	_param_vtol_motor_count(this, "MOT_COUNT"),
	_param_fw_pitch_trim(this, "FW_PITCH_TRIM"),
	_param_elevons_mc_lock(this, "ELEV_MC_LOCK"),
	_param_front_trans_dur(this, "F_TRANS_DUR"),
	_param_front_trans_time_min(this, "TRANS_MIN_TM"),
	_param_airspeed_blend_start(this, "ARSP_BLEND"),
	_param_airspeed_trans(this, "ARSP_TRANS"),
	_param_back_trans_dur(this, "B_TRANS_DUR"),
	_param_qc_fw_min_alt(this, "FW_MIN_ALT"),
	_param_qc_fw_alt_err(this, "FW_ALT_ERR"),
	_param_qc_fw_max_pitch(this, "FW_QC_P"),
	_param_qc_fw_max_roll(this, "FW_QC_R"),
	_param_wv_takeoff(this, "WV_TKO_EN"),
	_param_wv_loiter(this, "WV_LTR_EN"),
	_param_wv_land(this, "WV_LND_EN"),
	// non-vtol params
	_param_airspeed_mode(this, "FW_ARSP_MODE", false)
{
	for (auto &pwm_max : _max_mc_pwm_values.values) {
		pwm_max = PWM_DEFAULT_MAX;
	}

	_v_att = _attc->get_att();
	_v_att_sp = _attc->get_att_sp();
	_mc_virtual_att_sp = _attc->get_mc_virtual_att_sp();
	_fw_virtual_att_sp = _attc->get_fw_virtual_att_sp();
	_v_control_mode = _attc->get_control_mode();
	_actuators_out_0 = _attc->get_actuators_out0();
	_actuators_out_1 = _attc->get_actuators_out1();
	_actuators_mc_in = _attc->get_actuators_mc_in();
	_actuators_fw_in = _attc->get_actuators_fw_in();
	_local_pos = _attc->get_local_pos();
	_local_pos_sp = _attc->get_local_pos_sp();
	_airspeed = _attc->get_airspeed();
	_tecs_status = _attc->get_tecs_status();
	_land_detected = _attc->get_land_detected();
}

void VtolType::update_mc_state()
{
	// copy virtual attitude setpoint to real attitude setpoint
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	// VTOL weathervane
	_v_att_sp->disable_mc_yaw_control = false;

	if (_attc->get_pos_sp_triplet()->current.valid &&
	    !_v_control_mode->flag_control_manual_enabled) {

		if (_param_wv_takeoff.get()
		    && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

			_v_att_sp->disable_mc_yaw_control = true;

		} else if (_param_wv_loiter.get()
			   && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			_v_att_sp->disable_mc_yaw_control = true;

		} else if (_param_wv_land.get()
			   && _attc->get_pos_sp_triplet()->current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			_v_att_sp->disable_mc_yaw_control = true;
		}
	}
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
	return !_v_control_mode->flag_armed || _land_detected->landed;
}

void VtolType::check_quadchute_condition()
{

	if (_v_control_mode->flag_armed && !_land_detected->landed) {
		matrix::Eulerf euler = matrix::Quatf(_v_att->q);

		// fixed-wing minimum altitude
		if (_param_qc_fw_min_alt.get() > FLT_EPSILON) {

			if (-(_local_pos->z) < _param_qc_fw_min_alt.get()) {
				_attc->abort_front_transition("QuadChute: Minimum altitude breached");
			}
		}

		// adaptive quadchute
		if (_param_qc_fw_alt_err.get() > FLT_EPSILON && _v_control_mode->flag_control_altitude_enabled) {

			// We use tecs for tracking in FW and local_pos_sp during transitions
			if (_tecs_running) {
				// 1 second rolling average
				_ra_hrate = (49 * _ra_hrate + _tecs_status->flightPathAngle) / 50;
				_ra_hrate_sp = (49 * _ra_hrate_sp + _tecs_status->flightPathAngleSp) / 50;

				// are we dropping while requesting significant ascend?
				if (((_tecs_status->altitudeSp - _tecs_status->altitude_filtered) > _param_qc_fw_alt_err.get()) &&
				    (_ra_hrate < -1.0f) &&
				    (_ra_hrate_sp > 1.0f)) {

					_attc->abort_front_transition("QuadChute: loss of altitude");
				}

			} else {
				const bool height_error = _local_pos->z_valid && ((-_local_pos_sp->z - -_local_pos->z) > _param_qc_fw_alt_err.get());
				const bool height_rate_error = _local_pos->v_z_valid && (_local_pos->vz > 1.0f) && (_local_pos->z_deriv > 1.0f);

				if (height_error && height_rate_error) {
					_attc->abort_front_transition("QuadChute: large altitude error");
				}
			}
		}

		// fixed-wing maximum pitch angle
		if (_param_qc_fw_max_pitch.get() > 0) {
			if (fabsf(math::degrees(euler.theta())) > _param_qc_fw_max_pitch.get()) {
				_attc->abort_front_transition("Maximum pitch angle exceeded");
			}
		}

		// fixed-wing maximum roll angle
		if (_param_qc_fw_max_roll.get() > 0) {
			if (fabsf(math::degrees(euler.phi())) > _param_qc_fw_max_roll.get()) {
				_attc->abort_front_transition("Maximum roll angle exceeded");
			}
		}
	}
}

bool
VtolType::disable_mc_motors()
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return false;
	}

	// first save the current max values
	struct pwm_output_values max_pwm_values = {};

	int ret = px4_ioctl(fd, PWM_SERVO_GET_MAX_PWM, (long unsigned int)&max_pwm_values);

	if (ret == OK) {
		_max_mc_pwm_values = max_pwm_values;

	} else {
		PX4_ERR("failed getting max values");
		px4_close(fd);
		return false;
	}

	// now get the disarmed PWM values
	pwm_output_values disarmed_pwm_values = {};
	ret = px4_ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (long unsigned int)&disarmed_pwm_values);

	if (ret == OK) {

		// finally disable by setting the MC motors max to the disarmed value
		for (int i = 0; i < _param_vtol_motor_count.get(); i++) {
			max_pwm_values.values[i] = disarmed_pwm_values.values[i];
			max_pwm_values.channel_count = _param_vtol_motor_count.get();
		}

		ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&max_pwm_values);

		if (ret != OK) {
			PX4_ERR("failed setting max values");
		}

	} else {
		PX4_ERR("failed getting max values");
	}

	px4_close(fd);

	return (ret == PX4_OK);
}

bool
VtolType::enable_mc_motors()
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return false;
	}

	struct pwm_output_values pwm_values = {};

	for (int i = 0; i < _param_vtol_motor_count.get(); i++) {
		pwm_values.values[i] = _max_mc_pwm_values.values[i];
		pwm_values.channel_count = _param_vtol_motor_count.get();
	}

	int ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

	if (ret != OK) {
		PX4_ERR("failed setting max values");
	}

	px4_close(fd);

	return (ret == PX4_OK);
}
