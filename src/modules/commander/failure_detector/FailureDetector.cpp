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

	// esc_status subscriber is shared between subroutines
	esc_status_s esc_status;

	if (_esc_status_sub.update(&esc_status)) {
		_failure_injector.manipulateEscStatus(esc_status);

		if (_param_escs_en.get()) {
			updateEscsStatus(vehicle_status, esc_status);
		}

		if (_param_fd_act_en.get()) {
			updateMotorStatus(vehicle_status, esc_status);
		}
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
	failure_detector_status.fd_arm_escs = _failure_detector_status.flags.arm_escs;
	failure_detector_status.fd_battery = _failure_detector_status.flags.battery;
	failure_detector_status.fd_imbalanced_prop = _failure_detector_status.flags.imbalanced_prop;
	failure_detector_status.fd_motor = _failure_detector_status.flags.motor;
	failure_detector_status.imbalanced_prop_metric = _imbalanced_prop_lpf.getState();
	failure_detector_status.motor_failure_mask = _motor_failure_mask;
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

void FailureDetector::updateEscsStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status)
{
	hrt_abstime now = hrt_absolute_time();

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);
		const int all_escs_armed_mask = (1 << limited_esc_count) - 1;
		const bool is_all_escs_armed = (all_escs_armed_mask == esc_status.esc_armed_flags);

		bool is_esc_failure = !is_all_escs_armed;

		for (int i = 0; i < limited_esc_count; i++) {
			is_esc_failure = is_esc_failure;
		}

		_esc_failure_hysteresis.set_hysteresis_time_from(false, 300_ms);
		_esc_failure_hysteresis.set_state_and_update(is_esc_failure, now);

		if (_esc_failure_hysteresis.get_state()) {
			_failure_detector_status.flags.arm_escs = true;
		}

	} else {
		// reset ESC bitfield
		_esc_failure_hysteresis.set_state_and_update(false, now);
		_failure_detector_status.flags.arm_escs = false;
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

void FailureDetector::updateMotorStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status)
{
	// 1. Telemetry times out -> communication or power lost on that ESC
	// 2. Too low current draw compared to commanded thrust
	// Overvoltage, overcurrent do not have checks yet esc_report.failures are handled separately

	const hrt_abstime now = hrt_absolute_time();

	// Only check while armed
	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		actuator_motors_s actuator_motors{};
		_actuator_motors_sub.copy(&actuator_motors);

		// Check individual ESC reports
		for (uint8_t i = 0; i < esc_status_s::CONNECTED_ESC_MAX; ++i) {
			// Map the esc status index to the actuator function index
			const uint8_t actuator_function_index =
				esc_status.esc[i].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

			if (actuator_function_index >= actuator_motors_s::NUM_CONTROLS) {
				continue; // Invalid mapping
			}

			const bool timeout = now > esc_status.esc[i].timestamp + 300_ms;
			const float current = esc_status.esc[i].esc_current;

			// First wait for ESC telemetry reporting non-zero current. Before that happens, don't check it.
			if (current > FLT_EPSILON) {
				_esc_has_reported_current[i] = true;
			}

			if (!_esc_has_reported_current[i]) {
				continue;
			}

			_motor_failure_mask &= ~(1u << actuator_function_index); // Reset bit in mask to accumulate failures again
			_motor_failure_mask |= (static_cast<uint16_t>(timeout) << actuator_function_index); // Telemetry timeout

			// Current limits
			float thrust = 0.f;

			if (PX4_ISFINITE(actuator_motors.control[actuator_function_index])) {
				// Normalized motor thrust commands before thrust model factor is applied, NAN means motor is turned off -> 0 thrust
				thrust = fabsf(actuator_motors.control[actuator_function_index]);
			}

			bool thrust_above_threshold = thrust > _param_fd_act_mot_thr.get();
			bool current_too_low = current < (thrust * _param_fd_act_mot_c2t.get()) - _param_fd_act_low_off.get();
			bool current_too_high = current > (thrust * _param_fd_act_mot_c2t.get()) + _param_fd_act_high_off.get();

			_esc_undercurrent_hysteresis[i].set_hysteresis_time_from(false, _param_fd_act_mot_tout.get() * 1_ms);
			_esc_overcurrent_hysteresis[i].set_hysteresis_time_from(false, _param_fd_act_mot_tout.get() * 1_ms);

			if (!_esc_undercurrent_hysteresis[i].get_state()) {
				// Do not clear mid operation because a reaction could be to stop the motor and that would be conidered healthy again
				_esc_undercurrent_hysteresis[i].set_state_and_update(thrust_above_threshold && current_too_low && !timeout, now);
			}

			if (!_esc_overcurrent_hysteresis[i].get_state()) {
				// Do not clear mid operation because a reaction could be to stop the motor and that would be conidered healthy again
				_esc_overcurrent_hysteresis[i].set_state_and_update(current_too_high && !timeout, now);
			}

			_motor_failure_mask |= (static_cast<uint16_t>(_esc_undercurrent_hysteresis[i].get_state()) << actuator_function_index);
			_motor_failure_mask |= (static_cast<uint16_t>(_esc_overcurrent_hysteresis[i].get_state()) << actuator_function_index);
		}

		_failure_detector_status.flags.motor = (_motor_failure_mask != 0u);

	} else { // Disarmed
		for (uint8_t i = 0; i < esc_status_s::CONNECTED_ESC_MAX; ++i) {
			_esc_undercurrent_hysteresis[i].set_state_and_update(false, now);
			_esc_overcurrent_hysteresis[i].set_state_and_update(false, now);
		}

		_failure_detector_status.flags.motor = false;
	}
}
