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

FailureDetector::FailureDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FailureDetector::update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode)
{
	if (_failure_injection_config.update()) {
		_injected_motor_masks = failure_injection::process_motor(_failure_injection_config);
	}

	failure_detector_status_u status_prev = _failure_detector_status;

	if (vehicle_control_mode.flag_control_attitude_enabled) {
		updateAttitudeStatus(vehicle_status);
		updateAltitudeStatus(vehicle_status, vehicle_control_mode);

		if (_param_fd_ext_ats_en.get()) {
			updateExternalAtsStatus();
		}

	} else {
		_failure_detector_status.flags.roll = false;
		_failure_detector_status.flags.pitch = false;
		_failure_detector_status.flags.alt = false;
		_failure_detector_status.flags.ext = false;
		// Reset altitude loss state so it reinitialises cleanly when altitude control re-engages.
		_alt_loss_ref_z = NAN;
		_alt_loss_hysteresis.set_state_and_update(false, hrt_absolute_time());
	}

	// Note: keep the imbalanced propeller check before the impact check, it relies on vehicle_imu_status.updated()
	if (_param_fd_imb_prop_thr.get() > 0) {
		updateImbalancedPropStatus();
	}

	if (_param_fd_impact_thr.get() > FLT_EPSILON) {
		updateImpactStatus(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	} else {
		_failure_detector_status.flags.impact = false;
		_failure_detector_status.flags.crash = false;
		_crash_no_movement_hysteresis.set_state_and_update(false, hrt_absolute_time());
	}

	return _failure_detector_status.value != status_prev.value;
}

void FailureDetector::publishStatus(bool esc_arm_status, uint16_t motor_failure_mask)
{
	const uint16_t injected_motor_failure_mask = _injected_motor_masks.failure_mask;

	failure_detector_status_s failure_detector_status{};
	failure_detector_status.fd_roll = _failure_detector_status.flags.roll;
	failure_detector_status.fd_pitch = _failure_detector_status.flags.pitch;
	failure_detector_status.fd_alt = _failure_detector_status.flags.alt;
	failure_detector_status.fd_ext = _failure_detector_status.flags.ext;
	failure_detector_status.fd_arm_escs = esc_arm_status || (motor_failure_mask != 0);
	failure_detector_status.fd_battery = _failure_detector_status.flags.battery;
	failure_detector_status.fd_imbalanced_prop = _failure_detector_status.flags.imbalanced_prop;
	failure_detector_status.fd_motor = (motor_failure_mask != 0) || (injected_motor_failure_mask != 0);
	failure_detector_status.fd_impact = _failure_detector_status.flags.impact;
	failure_detector_status.fd_crash = _failure_detector_status.flags.crash;
	failure_detector_status.imbalanced_prop_metric = _imbalanced_prop_lpf.getState();
	failure_detector_status.impact_metric = _impact_metric_peak;
	_impact_metric_peak = 0.f;
	failure_detector_status.motor_failure_mask = motor_failure_mask | injected_motor_failure_mask;
	failure_detector_status.motor_stop_mask = _injected_motor_masks.stop_mask;
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

void FailureDetector::updateAltitudeStatus(const vehicle_status_s &vehicle_status,
		const vehicle_control_mode_s &vehicle_control_mode)
{
	const float threshold = _param_fd_alt_loss.get();

	if (threshold < FLT_EPSILON
	    || !vehicle_control_mode.flag_control_altitude_enabled
	    || vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_failure_detector_status.flags.alt = false;
		_alt_loss_ref_z = NAN;
		return;
	}

	vehicle_local_position_s lpos{};
	vehicle_local_position_setpoint_s lpos_sp{};
	_vehicle_local_position_sub.copy(&lpos);
	_vehicle_local_position_setpoint_sub.copy(&lpos_sp);

	// Adjust reference on EKF z reset to avoid false triggers
	if (lpos.z_reset_counter != _alt_loss_z_reset_counter) {
		if (PX4_ISFINITE(_alt_loss_ref_z)) {
			_alt_loss_ref_z += lpos.delta_z;
		}

		_alt_loss_z_reset_counter = lpos.z_reset_counter;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (lpos.z > lpos_sp.z) {
		// Ratcheting NED-z reference: tracks the highest altitude reached while below setpoint.
		if (!PX4_ISFINITE(_alt_loss_ref_z)) {
			_alt_loss_ref_z = lpos.z;
		}

		_alt_loss_ref_z = math::constrain(_alt_loss_ref_z, lpos_sp.z, lpos.z);

		const bool is_below_threshold = (lpos.z - _alt_loss_ref_z) > threshold;
		_alt_loss_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_alt_loss_ttri.get()));
		_alt_loss_hysteresis.set_state_and_update(is_below_threshold, now);

	} else {
		_alt_loss_ref_z = NAN;
		_alt_loss_hysteresis.set_state_and_update(false, now);
	}

	_failure_detector_status.flags.alt = _alt_loss_hysteresis.get_state();
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


bool FailureDetector::copySelectedImuStatus(vehicle_imu_status_s &imu_status)
{
	if (_sensor_selection_sub.updated()) {
		sensor_selection_s selection;

		if (_sensor_selection_sub.copy(&selection)) {
			_selected_accel_device_id = selection.accel_device_id;
		}
	}

	// Find the imu_status instance corresponding to the selected accelerometer
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

	return (imu_status.accel_device_id != 0) && (imu_status.accel_device_id == _selected_accel_device_id);
}

void FailureDetector::updateImbalancedPropStatus()
{
	const bool updated = _vehicle_imu_status_sub.updated(); // save before doing a copy

	vehicle_imu_status_s imu_status{};

	if (updated && copySelectedImuStatus(imu_status)) {
		const hrt_abstime dt_us = math::constrain(imu_status.timestamp - _imu_status_timestamp_prev, 10_ms, 1_s);
		_imu_status_timestamp_prev = imu_status.timestamp;

		_imbalanced_prop_lpf.setParameters(dt_us, _imbalanced_prop_lpf_time_constant);

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

void FailureDetector::updateImpactStatus(bool armed)
{
	const hrt_abstime now = hrt_absolute_time();

	if (!armed) {
		// Both flags are latched while armed and reset on disarm
		_failure_detector_status.flags.impact = false;
		_failure_detector_status.flags.crash = false;
		_crash_no_movement_hysteresis.set_state_and_update(false, now);
		return;
	}

	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	// Impact: the short-window averaged specific force (computed in vehicle_imu for the selected accel) exceeds
	// what the rotors can produce, i.e. a large external force such as hitting the ground or an obstacle.
	vehicle_imu_status_s imu_status{};

	if (copySelectedImuStatus(imu_status)) {
		_impact_metric_peak = math::max(_impact_metric_peak, imu_status.accel_impact_metric);

		if (!land_detected.landed && (imu_status.accel_impact_metric > _param_fd_impact_thr.get())) {
			_failure_detector_status.flags.impact = true;
		}
	}

	// Crash: after an impact the vehicle does not move for FD_IMPACT_T seconds while not detected as landed.
	// The movement flags are the land detector's own checks (LNDMC_Z_VEL_MAX, LNDMC_XY_VEL_MAX, LNDMC_ROT_MAX),
	// each true if any movement was seen since the previous vehicle_land_detected publication.
	const bool no_movement = !land_detected.vertical_movement
				 && !land_detected.horizontal_movement
				 && !land_detected.rotational_movement;

	_crash_no_movement_hysteresis.set_hysteresis_time_from(false, (hrt_abstime)(1_s * _param_fd_impact_ttri.get()));
	_crash_no_movement_hysteresis.set_state_and_update(_failure_detector_status.flags.impact && !land_detected.landed
			&& no_movement, now);

	if (_crash_no_movement_hysteresis.get_state()) {
		_failure_detector_status.flags.crash = true;
	}
}
