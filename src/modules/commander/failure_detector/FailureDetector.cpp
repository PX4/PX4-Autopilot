/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
* @file FailureDetector.cpp
*
* @author Mathieu Bresciani	<brescianimathieu@gmail.com>
*
*/

#include "FailureDetector.hpp"

using namespace time_literals;

FailureDetector::FailureDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FailureDetector::update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode)
{
	failure_detector_status_u status_prev = _status;

	if (vehicle_control_mode.flag_control_attitude_enabled) {
		updateAttitudeStatus();

		if (_param_fd_ext_ats_en.get()) {
			updateExternalAtsStatus();
		}

	} else {
		_status.flags.roll = false;
		_status.flags.pitch = false;
		_status.flags.alt = false;
		_status.flags.ext = false;
	}

	if (_param_escs_en.get()) {
		updateEscsStatus(vehicle_status);
	}

	if (_param_fd_imb_prop_thr.get() > 0) {
		updateImbalancedPropStatus();
	}

	return _status.value != status_prev.value;
}

void FailureDetector::updateAttitudeStatus()
{
	vehicle_attitude_s attitude;

	if (_vehicule_attitude_sub.update(&attitude)) {

		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		const float roll(euler.phi());
		const float pitch(euler.theta());

		const float max_roll_deg = _param_fd_fail_r.get();
		const float max_pitch_deg = _param_fd_fail_p.get();
		const float max_roll(fabsf(math::radians(max_roll_deg)));
		const float max_pitch(fabsf(math::radians(max_pitch_deg)));

		const bool roll_status = (max_roll > 0.0f) && (fabsf(roll) > max_roll);
		const bool pitch_status = (max_pitch > 0.0f) && (fabsf(pitch) > max_pitch);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_roll_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_r_ttri.get()));
		_pitch_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_p_ttri.get()));
		_roll_failure_hysteresis.set_state_and_update(roll_status, time_now);
		_pitch_failure_hysteresis.set_state_and_update(pitch_status, time_now);

		// Update status
		_status.flags.roll = _roll_failure_hysteresis.get_state();
		_status.flags.pitch = _pitch_failure_hysteresis.get_state();
	}
}

void FailureDetector::updateExternalAtsStatus()
{
	pwm_input_s pwm_input;

	if (_pwm_input_sub.update(&pwm_input)) {

		uint32_t pulse_width = pwm_input.pulse_width;
		bool ats_trigger_status = (pulse_width >= (uint32_t)_param_fd_ext_ats_trig.get()) && (pulse_width < 3_ms);

		hrt_abstime time_now = hrt_absolute_time();

		// Update hysteresis
		_ext_ats_failure_hysteresis.set_hysteresis_time_from(false, 100_ms); // 5 consecutive pulses at 50hz
		_ext_ats_failure_hysteresis.set_state_and_update(ats_trigger_status, time_now);

		_status.flags.ext = _ext_ats_failure_hysteresis.get_state();
	}
}

void FailureDetector::updateEscsStatus(const vehicle_status_s &vehicle_status)
{
	hrt_abstime time_now = hrt_absolute_time();

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		esc_status_s esc_status;

		if (_esc_status_sub.update(&esc_status)) {
			int all_escs_armed = (1 << esc_status.esc_count) - 1;

			_esc_failure_hysteresis.set_hysteresis_time_from(false, 300_ms);
			_esc_failure_hysteresis.set_state_and_update(all_escs_armed != esc_status.esc_armed_flags, time_now);

			if (_esc_failure_hysteresis.get_state()) {
				_status.flags.arm_escs = true;
			}
		}

	} else {
		// reset ESC bitfield
		_esc_failure_hysteresis.set_state_and_update(false, time_now);
		_status.flags.arm_escs = false;
	}
}

void FailureDetector::updateImbalancedPropStatus()
{

	if (_sensor_selection_sub.updated()) {
		sensor_selection_s selection;

		if (_sensor_selection_sub.copy(&selection)) {
			_selected_accel_device_id = selection.accel_device_id;
		}
	}

	const bool updated = _vehicle_imu_status_sub.updated(); // save before doing a copy

	// Find the imu_status instance corresponding to the selected accelerometer
	vehicle_imu_status_s imu_status{};
	_vehicle_imu_status_sub.copy(&imu_status);

	if (imu_status.accel_device_id != _selected_accel_device_id) {

		for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (!_vehicle_imu_status_sub.ChangeInstance(i)) {
				continue;
			}

			if (_vehicle_imu_status_sub.copy(&imu_status)
			    && (imu_status.accel_device_id == _selected_accel_device_id)) {
				// instance found
				break;
			}
		}
	}

	if (updated) {

		if (_vehicle_imu_status_sub.copy(&imu_status)) {

			if ((imu_status.accel_device_id != 0)
			    && (imu_status.accel_device_id == _selected_accel_device_id)) {
				const float dt = math::constrain((float)(imu_status.timestamp - _imu_status_timestamp_prev), 0.01f, 1.f);
				_imbalanced_prop_lpf.setParameters(dt, _imbalanced_prop_lpf_time_constant);

				const float var_x = imu_status.var_accel[0];
				const float var_y = imu_status.var_accel[1];
				const float var_z = imu_status.var_accel[2];

				const float metric = var_x + var_y - var_z;
				const float metric_lpf = _imbalanced_prop_lpf.update(metric);

				const bool is_imbalanced = metric_lpf > _param_fd_imb_prop_thr.get();
				_status.flags.imbalanced_prop = is_imbalanced;
			}
		}
	}
}
