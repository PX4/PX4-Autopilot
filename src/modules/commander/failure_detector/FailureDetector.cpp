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
#include "../HealthAndArmingChecks/HealthAndArmingChecks.hpp"

using namespace time_literals;

FailureDetector::FailureDetector(ModuleParams *parent, HealthAndArmingChecks &health_and_arming_checks) :
	ModuleParams(parent),
	_health_and_arming_checks(health_and_arming_checks)
{
}

bool FailureDetector::update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode)
{
	_failure_injector.update();

	failure_detector_status_u status_prev = _failure_detector_status;

	if (vehicle_control_mode.flag_control_attitude_enabled) {
		updateAttitudeStatus(vehicle_status);

		if (_param_fd_ext_ats_en.get()) {
			updateExternalAtsStatus();
		}

	} else {
		_failure_detector_status.flags.roll = false;
		_failure_detector_status.flags.pitch = false;
		_failure_detector_status.flags.alt = false;
		_failure_detector_status.flags.ext = false;
	}

	if (_param_fd_imb_prop_thr.get() > 0) {
		updateImbalancedPropStatus();
	}

	return _failure_detector_status.value != status_prev.value;
}

void FailureDetector::publishStatus()
{
	failure_detector_status_s failure_detector_status{};
	failure_detector_status.fd_roll = _failure_detector_status.flags.roll;
	failure_detector_status.fd_pitch = _failure_detector_status.flags.pitch;
	failure_detector_status.fd_alt = _failure_detector_status.flags.alt;
	failure_detector_status.fd_ext = _failure_detector_status.flags.ext;
	failure_detector_status.fd_arm_escs = _health_and_arming_checks.getEscArmStatus();
	failure_detector_status.fd_battery = _failure_detector_status.flags.battery;
	failure_detector_status.fd_imbalanced_prop = _failure_detector_status.flags.imbalanced_prop;
	failure_detector_status.fd_motor = (_health_and_arming_checks.getMotorFailureMask() != 0) || (_failure_injector.getMotorStopMask() != 0);
	failure_detector_status.imbalanced_prop_metric = _imbalanced_prop_lpf.getState();
	failure_detector_status.motor_failure_mask = _health_and_arming_checks.getMotorFailureMask();
	failure_detector_status.motor_stop_mask = _failure_injector.getMotorStopMask();
	failure_detector_status.timestamp = hrt_absolute_time();
	_failure_detector_status_pub.publish(failure_detector_status);
}

void FailureDetector::updateAttitudeStatus(const vehicle_status_s &vehicle_status)
{
	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.update(&attitude)) {

		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		float roll(euler.phi());
		float pitch(euler.theta());

		// special handling for tailsitter
		if (vehicle_status.is_vtol_tailsitter) {
			if (vehicle_status.in_transition_mode) {
				// disable attitude check during tailsitter transition
				roll = 0.f;
				pitch = 0.f;

			} else if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				// in FW flight rotate the attitude by 90° around pitch (level FW flight = 0° pitch)
				const matrix::Eulerf euler_rotated = matrix::Eulerf(matrix::Quatf(attitude.q) * matrix::Quatf(matrix::Eulerf(0.f,
								     M_PI_2_F, 0.f)));
				roll = euler_rotated.phi();
				pitch = euler_rotated.theta();
			}
		}

		const float max_roll_deg = _param_fd_fail_r.get();
		const float max_pitch_deg = _param_fd_fail_p.get();
		const float max_roll(fabsf(math::radians(max_roll_deg)));
		const float max_pitch(fabsf(math::radians(max_pitch_deg)));

		const bool roll_status = (max_roll > FLT_EPSILON) && (fabsf(roll) > max_roll);
		const bool pitch_status = (max_pitch > FLT_EPSILON) && (fabsf(pitch) > max_pitch);

		hrt_abstime now = hrt_absolute_time();

		// Update hysteresis
		_roll_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_r_ttri.get()));
		_pitch_failure_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_fail_p_ttri.get()));
		_roll_failure_hysteresis.set_state_and_update(roll_status, now);
		_pitch_failure_hysteresis.set_state_and_update(pitch_status, now);

		// Update status
		_failure_detector_status.flags.roll = _roll_failure_hysteresis.get_state();
		_failure_detector_status.flags.pitch = _pitch_failure_hysteresis.get_state();
	}
}

void FailureDetector::updateExternalAtsStatus()
{
	pwm_input_s pwm_input;

	if (_pwm_input_sub.update(&pwm_input)) {

		uint32_t pulse_width = pwm_input.pulse_width;
		bool ats_trigger_status = (pulse_width >= (uint32_t)_param_fd_ext_ats_trig.get()) && (pulse_width < 3_ms);

		// Update hysteresis
		_ext_ats_failure_hysteresis.set_hysteresis_time_from(false, 100_ms); // 5 consecutive pulses at 50hz
		_ext_ats_failure_hysteresis.set_state_and_update(ats_trigger_status, hrt_absolute_time());

		_failure_detector_status.flags.ext = _ext_ats_failure_hysteresis.get_state();
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
				const float dt = math::constrain((imu_status.timestamp - _imu_status_timestamp_prev) * 1e-6f, 0.01f, 1.f);
				_imu_status_timestamp_prev = imu_status.timestamp;

				_imbalanced_prop_lpf.setParameters(dt, _imbalanced_prop_lpf_time_constant);

				const float std_x = sqrtf(math::max(imu_status.var_accel[0], 0.f));
				const float std_y = sqrtf(math::max(imu_status.var_accel[1], 0.f));
				const float std_z = sqrtf(math::max(imu_status.var_accel[2], 0.f));

				// Note: the metric is done using standard deviations instead of variances to be linear
				const float metric = (std_x + std_y) / 2.f - std_z;
				const float metric_lpf = _imbalanced_prop_lpf.update(metric);

				const bool is_imbalanced = metric_lpf > _param_fd_imb_prop_thr.get();
				_failure_detector_status.flags.imbalanced_prop = is_imbalanced;
			}
		}
	}
}

