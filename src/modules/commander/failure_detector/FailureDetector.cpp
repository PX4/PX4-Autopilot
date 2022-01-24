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

void FailureInjector::update()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);
		const int instance = static_cast<int>(vehicle_command.param3 + 0.5f);

		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SYSTEM_MOTOR) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, motors ok");
				supported = false;

				// 0 to signal all
				if (instance == 0) {
					supported = true;

					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d ok", i);
						_esc_blocked &= ~(1 << i);
						_esc_wrong &= ~(1 << i);
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					supported = true;

					PX4_INFO("CMD_INJECT_FAILURE, motor %d ok", instance - 1);
					_esc_blocked &= ~(1 << (instance - 1));
					_esc_wrong &= ~(1 << (instance - 1));
				}
			}

			else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, motors off");
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d off", i);
						_esc_blocked |= 1 << i;
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, motor %d off", instance - 1);
					_esc_blocked |= 1 << (instance - 1);
				}
			}

			else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				PX4_INFO("CMD_INJECT_FAILURE, motors wrong");
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d wrong", i);
						_esc_wrong |= 1 << i;
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, motor %d wrong", instance - 1);
					_esc_wrong |= 1 << (instance - 1);
				}
			}
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				     vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED :
				     vehicle_command_ack_s::VEHICLE_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}
	}

}

void FailureInjector::manipulateEscStatus(esc_status_s &status)
{
	if (_esc_blocked != 0 || _esc_wrong != 0) {
		unsigned offline = 0;

		for (int i = 0; i < status.esc_count; i++) {
			const unsigned i_esc = status.esc[i].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

			if (_esc_blocked & (1 << i_esc)) {
				unsigned function = status.esc[i].actuator_function;
				memset(&status.esc[i], 0, sizeof(status.esc[i]));
				status.esc[i].actuator_function = function;
				offline |= 1 << i;

			} else if (_esc_wrong & (1 << i_esc)) {
				// Create wrong rerport for this motor by scaling key values up and down
				status.esc[i].esc_voltage *= 0.1f;
				status.esc[i].esc_current *= 0.1f;
				status.esc[i].esc_rpm *= 10.0f;
			}
		}

		status.esc_online_flags &= ~offline;
	}
}

FailureDetector::FailureDetector(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool FailureDetector::update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode)
{
	_failure_injector.update();

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

	// esc_status subscriber is shared between subroutines
	esc_status_s esc_status;

	if (_esc_status_sub.update(&esc_status)) {
		_failure_injector.manipulateEscStatus(esc_status);

		if (_param_escs_en.get()) {
			updateEscsStatus(vehicle_status, esc_status);
		}

		if (_param_fd_actuator_en.get()) {
			updateMotorStatus(vehicle_status, esc_status);
		}
	}

	if (_param_fd_imb_prop_thr.get() > 0) {
		updateImbalancedPropStatus();
	}

	return _status.value != status_prev.value;
}

void FailureDetector::updateAttitudeStatus()
{
	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.update(&attitude)) {

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

void FailureDetector::updateEscsStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status)
{
	hrt_abstime time_now = hrt_absolute_time();

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);
		const int all_escs_armed_mask = (1 << limited_esc_count) - 1;
		const bool is_all_escs_armed = (all_escs_armed_mask == esc_status.esc_armed_flags);

		bool is_esc_failure = !is_all_escs_armed;

		for (int i = 0; i < limited_esc_count; i++) {
			is_esc_failure = is_esc_failure | (esc_status.esc[i].failures > 0);
		}

		_esc_failure_hysteresis.set_hysteresis_time_from(false, 300_ms);
		_esc_failure_hysteresis.set_state_and_update(is_esc_failure, time_now);

		if (_esc_failure_hysteresis.get_state()) {
			_status.flags.arm_escs = true;
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
				_imu_status_timestamp_prev = imu_status.timestamp;

				_imbalanced_prop_lpf.setParameters(dt, _imbalanced_prop_lpf_time_constant);

				const float std_x = sqrtf(imu_status.var_accel[0]);
				const float std_y = sqrtf(imu_status.var_accel[1]);
				const float std_z = sqrtf(imu_status.var_accel[2]);

				// Note: the metric is done using standard deviations instead of variances to be linear
				const float metric = (std_x + std_y) / 2.f - std_z;
				const float metric_lpf = _imbalanced_prop_lpf.update(metric);

				const bool is_imbalanced = metric_lpf > _param_fd_imb_prop_thr.get();
				_status.flags.imbalanced_prop = is_imbalanced;
			}
		}
	}
}

void FailureDetector::updateMotorStatus(const vehicle_status_s &vehicle_status, const esc_status_s &esc_status)
{
	// What need to be checked:
	//
	// 1. ESC telemetry disappears completely -> dead ESC or power loss on that ESC
	// 2. ESC failures like overvoltage, overcurrent etc. But DShot driver for example is not populating the field 'esc_report.failures'
	// 3. Motor current too low. Compare drawn motor current to expected value from a parameter
	// -- ESC voltage does not really make sense and is highly dependent on the setup

	// First wait for some ESC telemetry that has the required fields. Before that happens, don't check this ESC
	// Then check

	const hrt_abstime time_now = hrt_absolute_time();

	// Only check while armed
	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);

		actuator_motors_s actuator_motors{};
		_actuator_motors_sub.copy(&actuator_motors);

		// Check individual ESC reports
		for (int esc_status_idx = 0; esc_status_idx < limited_esc_count; esc_status_idx++) {

			const esc_report_s &cur_esc_report = esc_status.esc[esc_status_idx];

			// Map the esc status index to the actuator function index
			const unsigned i_esc = cur_esc_report.actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

			if (i_esc >= actuator_motors_s::NUM_CONTROLS) {
				continue;
			}

			// Check if ESC telemetry was available and valid at some point. This is a prerequisite for the failure detection.
			if (!(_motor_failure_esc_valid_current_mask & (1 << i_esc)) && cur_esc_report.esc_current > 0.0f) {
				_motor_failure_esc_valid_current_mask |= (1 << i_esc);
			}

			// Check for telemetry timeout
			const hrt_abstime telemetry_age = time_now - cur_esc_report.timestamp;
			const bool esc_timed_out = telemetry_age > 100_ms;  // TODO: magic number

			const bool esc_was_valid = _motor_failure_esc_valid_current_mask & (1 << i_esc);
			const bool esc_timeout_currently_flagged = _motor_failure_esc_timed_out_mask & (1 << i_esc);

			if (esc_was_valid && esc_timed_out && !esc_timeout_currently_flagged) {
				// Set flag
				_motor_failure_esc_timed_out_mask |= (1 << i_esc);

			} else if (!esc_timed_out && esc_timeout_currently_flagged) {
				// Reset flag
				_motor_failure_esc_timed_out_mask &= ~(1 << i_esc);
			}

			// Check if ESC current is too low
			float esc_throttle = 0.f;

			if (PX4_ISFINITE(actuator_motors.control[i_esc])) {
				esc_throttle = fabsf(actuator_motors.control[i_esc]);
			}

			const bool throttle_above_threshold = esc_throttle > _param_fd_motor_throttle_thres.get();
			const bool current_too_low = cur_esc_report.esc_current < esc_throttle *
						     _param_fd_motor_current2throttle_thres.get();

			if (throttle_above_threshold && current_too_low && !esc_timed_out) {
				if (_motor_failure_undercurrent_start_time[i_esc] == 0) {
					_motor_failure_undercurrent_start_time[i_esc] = time_now;
				}

			} else {
				if (_motor_failure_undercurrent_start_time[i_esc] != 0) {
					_motor_failure_undercurrent_start_time[i_esc] = 0;
				}
			}

			if (_motor_failure_undercurrent_start_time[i_esc] != 0
			    && (time_now - _motor_failure_undercurrent_start_time[i_esc]) > _param_fd_motor_time_thres.get() * 1_ms
			    && (_motor_failure_esc_under_current_mask & (1 << i_esc)) == 0) {
				// Set flag
				_motor_failure_esc_under_current_mask |= (1 << i_esc);

			} // else: this flag is never cleared, as the motor is stopped, so throttle < threshold

		}

		bool critical_esc_failure = (_motor_failure_esc_timed_out_mask != 0 || _motor_failure_esc_under_current_mask != 0);

		if (critical_esc_failure && !(_status.flags.motor)) {
			// Add motor failure flag to bitfield
			_status.flags.motor = true;

		} else if (!critical_esc_failure && _status.flags.motor) {
			// Reset motor failure flag
			_status.flags.motor = false;
		}

	} else { // Disarmed
		// reset ESC bitfield
		for (int i_esc = 0; i_esc < actuator_motors_s::NUM_CONTROLS; i_esc++) {
			_motor_failure_undercurrent_start_time[i_esc] = 0;
		}

		_motor_failure_esc_under_current_mask = 0;
		_status.flags.motor = false;
	}
}
