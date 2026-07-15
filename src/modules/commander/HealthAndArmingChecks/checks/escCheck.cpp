/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "escCheck.hpp"
#include <px4_platform_common/events.h>
#include <uORB/topics/actuator_motors.h>

using namespace time_literals;

using esc_fault_reason_t = events::px4::enums::esc_fault_reason_t;
static_assert(esc_report_s::ESC_FAILURE_COUNT == (static_cast<uint8_t>(esc_fault_reason_t::_max) + 1)
	      , "ESC fault flags mismatch!");
static constexpr const char *esc_fault_reason_str(esc_fault_reason_t esc_fault_reason)
{
	switch (esc_fault_reason) {
	case esc_fault_reason_t::over_current: return "over current";

	case esc_fault_reason_t::over_voltage: return "over voltage";

	case esc_fault_reason_t::motor_over_temp: return "motor critical temperature";

	case esc_fault_reason_t::over_rpm: return "over RPM";

	case esc_fault_reason_t::inconsistent_cmd: return "control failure";

	case esc_fault_reason_t::motor_stuck: return "motor stall";

	case esc_fault_reason_t::failure_generic: return "hardware failure";

	case esc_fault_reason_t::motor_warn_temp: return "motor over temperature";

	case esc_fault_reason_t::esc_warn_temp: return "over temperature";

	case esc_fault_reason_t::esc_over_temp: return "critical temperature";
	}

	return "";
};


void EscChecks::checkAndReport(const Context &context, Report &reporter)
{
	const hrt_abstime now = hrt_absolute_time();
	esc_status_s esc_status;

	// Run motor status checks even when no new ESC data arrives (to detect timeouts)
	if (_esc_status_sub.copy(&esc_status)) {
		if (esc_status.esc_count <= 0) {
			return;
		}

		uint16_t mask = 0;

		if (_param_com_arm_chk_escs.get() > 0) {
			mask |= checkEscOnline(context, reporter, esc_status, now);
			mask |= checkEscStatus(context, reporter, esc_status);
		}

		if (_param_fd_act_en.get() > 0) {
			updateEscsStatus(context, reporter, esc_status, now);
			mask |= checkMotorStatus(context, reporter, esc_status);
		}

		if (_param_esc_temp_warn_th.get() >= FLT_EPSILON) {
			checkEscTemperature(reporter, esc_status);
		}

		_motor_failure_mask = mask;
		reporter.setIsPresent(health_component_t::motors_escs);
		reporter.failsafeFlags().fd_motor_failure = (mask != 0);

	} else if ((_param_com_arm_chk_escs.get() > 0) && now > _start_time + 3_s) { // Allow ESCs to init
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_CHK_ESCS</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::motors_escs, events::ID("check_escs_telem_missing"),
				       events::Log::Critical, "ESC telemetry missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: ESC telemetry missing");
		}
	}
}

uint16_t EscChecks::checkEscOnline(const Context &context, Report &reporter, const esc_status_s &esc_status, hrt_abstime now)
{
	// Check if one or more the ESCs are offline
	uint16_t mask = 0;
	char esc_fail_msg[esc_status_s::CONNECTED_ESC_MAX * 6 + 1] = "";

	for (int esc_index = 0; esc_index < esc_status_s::CONNECTED_ESC_MAX; esc_index++) {
		if (!math::isInRange(esc_status.esc[esc_index].actuator_function,
				     esc_report_s::ACTUATOR_FUNCTION_MOTOR1, esc_report_s::ACTUATOR_FUNCTION_MOTOR_MAX)) {
			continue; // Skip unmapped ESC status entries
		}

		const bool esc_telemetry_timeout = now > esc_status.esc[esc_index].timestamp + ESC_TIMEOUT_US;
		const bool is_offline = (esc_status.esc_online_flags & (1 << esc_index)) == 0;

		// Set failure bits for this motor
		if (esc_telemetry_timeout || is_offline) {
			mask |= (1u << esc_index);

			uint8_t esc_nr = esc_index + 1;
			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>COM_ARM_CHK_ESCS</param> parameter.
			 * </profile>
			 */
			reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::motors_escs, events::ID("check_escs_offline"),
							events::Log::Critical, "ESC {1} offline", esc_nr);
			snprintf(esc_fail_msg + strlen(esc_fail_msg), sizeof(esc_fail_msg) - strlen(esc_fail_msg), "ESC%d ", esc_nr);
			esc_fail_msg[sizeof(esc_fail_msg) - 1] = '\0';
		}
	}

	if (reporter.mavlink_log_pub() && esc_fail_msg[0] != '\0') {
		mavlink_log_critical(reporter.mavlink_log_pub(), "%soffline. %s\t", esc_fail_msg, context.isArmed() ? "Land now!" : "");
	}

	return mask;
}

uint16_t EscChecks::checkEscStatus(const Context &context, Report &reporter, const esc_status_s &esc_status)
{
	uint16_t mask = 0;

	for (int esc_index = 0; esc_index < esc_status_s::CONNECTED_ESC_MAX; esc_index++) {
		if (!math::isInRange(esc_status.esc[esc_index].actuator_function,
				     esc_report_s::ACTUATOR_FUNCTION_MOTOR1, esc_report_s::ACTUATOR_FUNCTION_MOTOR_MAX)) {
			continue; // Skip unmapped ESC status entries
		}

		const uint8_t actuator_function_index = esc_status.esc[esc_index].actuator_function - esc_report_s::ACTUATOR_FUNCTION_MOTOR1;

		if (esc_status.esc[esc_index].failures == 0) {
			continue;

		} else {
			mask |= (1u << actuator_function_index); // Set bit in mask
		}

		for (uint8_t fault_index = 0; fault_index <= static_cast<uint8_t>(esc_fault_reason_t::_max); fault_index++) {
			if (!(esc_status.esc[esc_index].failures & (1 << fault_index))) {
				continue;
			}

			esc_fault_reason_t fault_reason_index = static_cast<esc_fault_reason_t>(fault_index);
			const char *user_action = "";
			events::px4::enums::suggested_action_t action = events::px4::enums::suggested_action_t::none;

			if (context.isArmed()) {
				if (fault_reason_index == esc_fault_reason_t::motor_warn_temp
				    || fault_reason_index == esc_fault_reason_t::esc_warn_temp
				    || fault_reason_index == esc_fault_reason_t::over_rpm) {
					user_action = "Reduce throttle";
					action = events::px4::enums::suggested_action_t::reduce_throttle;

				} else {
					user_action = "Land now!";
					action = events::px4::enums::suggested_action_t::land;
				}
			}

			/* EVENT
			 * @description
			 * {3}
			 *
			 * <profile name="dev">
			 * This check can be configured via <param>COM_ARM_CHK_ESCS</param> parameter.
			 * </profile>
			 */
			reporter.healthFailure<uint8_t, events::px4::enums::esc_fault_reason_t, events::px4::enums::suggested_action_t>(
				NavModes::All, health_component_t::motors_escs, events::ID("check_failure_detector_arm_esc"),
				events::Log::Critical, "ESC {1}: {2}", esc_index + 1, fault_reason_index, action);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_emergency(reporter.mavlink_log_pub(), "ESC%d: %s. %s \t", esc_index + 1,
						      esc_fault_reason_str(fault_reason_index), user_action);
			}
		}
	}

	return mask;
}

void EscChecks::checkEscTemperature(Report &reporter, const esc_status_s &esc_status)
{
	const float warn_temp = _param_esc_temp_warn_th.get();

	uint8_t hottest_esc_index = UINT8_MAX;
	float max_temperature = -FLT_MAX;

	for (uint8_t esc_index = 0; esc_index < esc_status_s::CONNECTED_ESC_MAX; esc_index++) {
		if (!math::isInRange(esc_status.esc[esc_index].actuator_function,
				     esc_report_s::ACTUATOR_FUNCTION_MOTOR1, esc_report_s::ACTUATOR_FUNCTION_MOTOR_MAX)) {
			continue;
		}

		const float temperature = esc_status.esc[esc_index].esc_temperature;

		if (!PX4_ISFINITE(temperature)) {
			continue;
		}

		if (temperature > max_temperature) {
			max_temperature = temperature;
			hottest_esc_index = esc_index;
		}

		if (temperature > warn_temp) {
			/* EVENT
			* @description
			* <profile name="dev">
			* Configured by <param>ESC_TEMP_WARN_TH</param>
			* </profile>
			*/
			reporter.healthFailure<uint8_t, uint8_t>(NavModes::None, health_component_t::motors_escs,
					events::ID("check_esc_over_temperature"),
					events::Log::Warning,
					"ESC {1} temperature warning, {2:C}",
					static_cast<uint8_t>(esc_index + 1), static_cast<uint8_t>(temperature));
		}
	}

	if (hottest_esc_index == UINT8_MAX) {
		return;
	}

	if (max_temperature >= warn_temp) {
		if (!_esc_over_temp_warned && reporter.mavlink_log_pub()) {
			mavlink_log_warning(reporter.mavlink_log_pub(), "High ESC temperature. Reduce throttle!");
			_esc_over_temp_warned = true;
		}

	} else if (max_temperature < warn_temp - 5.f) {
		_esc_over_temp_warned = false;
	}
}

uint16_t EscChecks::checkMotorStatus(const Context &context, Report &reporter, const esc_status_s &esc_status)
{
	uint16_t mask = 0;

	if (context.status().arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		// If no disarmed cycle configured us first (FD_ACT_EN toggled while armed, or ESCs that only
		// report while spinning), configure once here -- else the detector stays disabled and never trips.
		if (!_motor_failure_configured) { configureMotorFailureDetector(); }

		actuator_motors_s actuator_motors{};
		_actuator_motors_sub.copy(&actuator_motors);

		// The arrays below are indexed by (actuator_function - MOTOR1); keep that within bounds so a
		// future change to the ESC function range can't silently overflow them in the arming path.
		static_assert(esc_report_s::ACTUATOR_FUNCTION_MOTOR_MAX - esc_report_s::ACTUATOR_FUNCTION_MOTOR1
			      < MotorFailureDetector::kMaxMotors, "ESC motor-function range exceeds kMaxMotors");

		// command defaults to NAN (= no ESC data this tick -> excluded); current/reversible to 0/false.
		float command[MotorFailureDetector::kMaxMotors];
		float current[MotorFailureDetector::kMaxMotors] {};
		bool reversible[MotorFailureDetector::kMaxMotors] {};

		for (float &u : command) { u = NAN; }

		for (uint8_t i = 0; i < esc_status_s::CONNECTED_ESC_MAX; ++i) {
			if (!math::isInRange(esc_status.esc[i].actuator_function,
					     esc_report_s::ACTUATOR_FUNCTION_MOTOR1, esc_report_s::ACTUATOR_FUNCTION_MOTOR_MAX)) {
				continue;
			}

			const uint8_t motor = esc_status.esc[i].actuator_function - esc_report_s::ACTUATOR_FUNCTION_MOTOR1;
			const float measured_current = esc_status.esc[i].esc_current;

			// Wait for the ESC to start reporting current before judging it.
			if (measured_current > FLT_EPSILON) {
				_esc_has_reported_current[i] = true;
			}

			if (!_esc_has_reported_current[i]) {
				continue;
			}

			current[motor] = measured_current;
			command[motor] = actuator_motors.control[motor];
			reversible[motor] = (actuator_motors.reversible_flags >> motor) & 1;
		}

		// Drive the detector off the ESC sample time so a telemetry stall freezes the filter.
		_motor_failure_detector.update(MotorFailureDetector::kMaxMotors, esc_status.timestamp,
					       command, current, reversible);

		for (uint8_t motor = 0; motor < MotorFailureDetector::kMaxMotors; ++motor) {
			if (_motor_failure_detector.status(motor).excluded || !_motor_failure_detector.status(motor).failed) {
				continue;
			}

			mask |= (1u << motor);

			// Latch under/over direction at first trip so the message stays consistent even if
			// the (sticky) residual later drifts across zero.
			if (_motor_failure_direction[motor] == 0) {
				_motor_failure_direction[motor] = (_motor_failure_detector.status(motor).residual_lpf < 0.f) ? -1 : 1;
			}

			if (_motor_failure_direction[motor] < 0) {
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>FD_ACT_EN</param> parameter.
				 * </profile>
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::motors_escs,
								events::ID("check_motor_undercurrent"),
								events::Log::Critical, "Motor {1} undercurrent detected", motor + 1);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Motor failure: Motor %d undercurrent detected", motor + 1);
				}

			} else {
				/* EVENT
				 * @description
				 * <profile name="dev">
				 * This check can be configured via <param>FD_ACT_EN</param> parameter.
				 * </profile>
				 */
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::motors_escs,
								events::ID("check_motor_overcurrent"),
								events::Log::Critical, "Motor {1} overcurrent detected", motor + 1);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Motor failure: Motor %d overcurrent detected", motor + 1);
				}
			}
		}

	} else {
		// Disarmed: re-apply parameters (also resets detector state) for the next arm.
		configureMotorFailureDetector();
	}

	return mask;
}

void EscChecks::configureMotorFailureDetector()
{
	MotorFailureDetector::Config cfg{};
	cfg.model_b = _param_motfail_c2t.get();
	cfg.model_c = _param_motfail_idle.get();
	cfg.residual_lpf_tau_s = MOTOR_FAILURE_LPF_TAU_S;
	cfg.threshold_a = _param_motfail_off.get();
	cfg.persistence_s = _param_motfail_time.get();

	for (int motor = 0; motor < MotorFailureDetector::kMaxMotors; ++motor) { _motor_failure_direction[motor] = 0; }

	_motor_failure_detector.configure(cfg);
	_motor_failure_configured = true;
}

void EscChecks::updateEscsStatus(const Context &context, Report &reporter, const esc_status_s &esc_status, hrt_abstime now)
{
	if (context.status().arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);
		const int all_escs_armed_mask = (1 << limited_esc_count) - 1;
		const bool is_all_escs_armed = (all_escs_armed_mask == esc_status.esc_armed_flags);

		_esc_arm_hysteresis.set_hysteresis_time_from(false, ESC_TIMEOUT_US);
		_esc_arm_hysteresis.set_state_and_update(!is_all_escs_armed, now);

		if (_esc_arm_hysteresis.get_state()) {
			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>COM_ARM_CHK_ESCS</param> parameter.
			 * </profile>
			 */
			reporter.healthFailure(NavModes::All, health_component_t::motors_escs,
					       events::ID("check_escs_not_all_armed"),
					       events::Log::Critical, "Not all ESCs are armed");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "ESC failure: Not all ESCs are armed. Land now!");
			}

			reporter.failsafeFlags().fd_esc_arming_failure = true;
		}

	} else {
		// reset ESC bitfield
		_esc_arm_hysteresis.set_state_and_update(false, now);
		reporter.failsafeFlags().fd_esc_arming_failure = false;
	}
}
