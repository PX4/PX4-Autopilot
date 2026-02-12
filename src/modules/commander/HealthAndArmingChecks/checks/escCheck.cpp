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
	const hrt_abstime esc_telemetry_timeout =
		2_s; // Some DShot ESCs are unresponsive for ~550ms during their initialization, so we use a timeout higher than that

	esc_status_s esc_status;


	// Run motor status checks even when no new ESC data arrives (to detect timeouts)
	if (_esc_status_sub.copy(&esc_status)) {
		if (esc_status.esc_count <= 0) {
			return;
		}
		uint16_t mask = 0;

		mask |= checkEscOnline(context, reporter, esc_status);
		mask |= checkEscStatus(context, reporter, esc_status);
		updateEscsStatus(context, reporter, esc_status);

		if (!_param_fd_act_en.get()) {
			mask |= checkMotorStatus(context, reporter, esc_status);
		}
		_motor_failure_mask = mask;
		reporter.setIsPresent(health_component_t::motors_escs);

	} else if (_param_com_arm_chk_escs.get()
		   && now - _start_time > esc_telemetry_timeout) { // Wait a bit after startup to allow esc's to init

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

uint16_t EscChecks::checkEscOnline(const Context &context, Report &reporter, const esc_status_s &esc_status)
{
	// Check if one or more the ESCs are offline
	if (!_param_com_arm_chk_escs.get()){
		return 0;
	}
	uint16_t mask = 0;
	char esc_fail_msg[50];
	esc_fail_msg[0] = '\0';

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		// check if mapped
		if (!math::isInRange(esc_status.esc[index].actuator_function, actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1,
							    uint8_t(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + actuator_motors_s::NUM_CONTROLS - 1))) {
			continue; // Skip unmapped ESC status entries
		}
		const uint8_t actuator_function_index =
		esc_status.esc[index].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

		const bool timeout = hrt_absolute_time() > esc_status.esc[index].timestamp + 300_ms;
		const bool is_offline = (esc_status.esc_online_flags & (1 << index)) == 0;



		// Set failure bits for this motor
		if (timeout || is_offline) {
			mask |= (1u << actuator_function_index);

			uint8_t motor_index = actuator_function_index + 1;
			/* EVENT
				* @description
				* <profile name="dev">
				* This check can be configured via <param>COM_ARM_CHK_ESCS</param> parameter.
				* </profile>
				*/
			reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::motors_escs, events::ID("check_escs_offline"),
							events::Log::Critical, "ESC {1} offline", motor_index);
			snprintf(esc_fail_msg + strlen(esc_fail_msg), sizeof(esc_fail_msg) - strlen(esc_fail_msg), "ESC%d ", motor_index);
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
	if (!_param_com_arm_chk_escs.get()){
		return 0;
	}

	uint16_t mask = 0;

	for (int index = 0; index < esc_status_s::CONNECTED_ESC_MAX; index++) {

		if (!math::isInRange(esc_status.esc[index].actuator_function, actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1,
							    uint8_t(actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + actuator_motors_s::NUM_CONTROLS - 1))) {
			continue; // Skip unmapped ESC status entries
		}

		const uint8_t act_function_index =
			esc_status.esc[index].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

		if (esc_status.esc[index].failures == 0) {
			continue;
		}
		else
		{
			mask |= (1u << act_function_index); // Set bit in mask
		}


		for (uint8_t fault_index = 0; fault_index <= static_cast<uint8_t>(esc_fault_reason_t::_max); fault_index++) {
			if (!(esc_status.esc[index].failures & (1 << fault_index))) {
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
				events::Log::Critical, "ESC {1}: {2}", act_function_index + 1, fault_reason_index, action);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_emergency(reporter.mavlink_log_pub(), "ESC%d: %s. %s \t", act_function_index + 1,
						      esc_fault_reason_str(fault_reason_index), user_action);
			}
		}
	}
	return mask;
}

uint16_t EscChecks::checkMotorStatus(const Context &context, Report &reporter, const esc_status_s &esc_status)
{
	if (!_param_fd_act_en.get()){
		return 0;
	}
	const hrt_abstime now = hrt_absolute_time();
	uint16_t mask = 0;

	// Only check while armed
	if (context.status().arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
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

			const float current = esc_status.esc[i].esc_current;

			// First wait for ESC telemetry reporting non-zero current. Before that happens, don't check it.
			if (current > FLT_EPSILON) {
				_esc_has_reported_current[i] = true;
			}

			if (!_esc_has_reported_current[i]) {
				continue;
			}

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
				_esc_undercurrent_hysteresis[i].set_state_and_update(thrust_above_threshold && current_too_low, now);
			}

			if (!_esc_overcurrent_hysteresis[i].get_state()) {
				// Do not clear mid operation because a reaction could be to stop the motor and that would be conidered healthy again
				_esc_overcurrent_hysteresis[i].set_state_and_update(current_too_high, now);
			}

			mask |= (static_cast<uint16_t>(_esc_undercurrent_hysteresis[i].get_state()) << actuator_function_index);
			mask |= (static_cast<uint16_t>(_esc_overcurrent_hysteresis[i].get_state()) << actuator_function_index);

			if (_esc_undercurrent_hysteresis[i].get_state()) {
				/* EVENT
					* @description
					* <profile name="dev">
					* This check can be configured via <param>FD_ACT_EN</param> parameter.
					* </profile>
					*/
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::motors_escs,
								events::ID("check_failure_detector_motor_uc"),
								events::Log::Critical, "Motor {1} undercurrent detected", actuator_function_index + 1);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Motor failure: Motor %d undercurrent detected",
							     actuator_function_index + 1);
				}
			}

			if (_esc_overcurrent_hysteresis[i].get_state()) {
				/* EVENT
					* @description
					* <profile name="dev">
					* This check can be configured via <param>FD_ACT_EN</param> parameter.
					* </profile>
					*/
				reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::motors_escs,
								events::ID("check_failure_detector_motor_oc"),
								events::Log::Critical, "Motor {1} overcurrent detected", actuator_function_index + 1);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Motor failure: Motor %d overcurrent detected",
							     actuator_function_index + 1);
				}
			}
		}

	} else { // Disarmed
		for (uint8_t i = 0; i < esc_status_s::CONNECTED_ESC_MAX; ++i) {
			_esc_undercurrent_hysteresis[i].set_state_and_update(false, now);
			_esc_overcurrent_hysteresis[i].set_state_and_update(false, now);
		}

	}
	return mask;
}


void EscChecks::updateEscsStatus(const Context &context, Report &reporter, const esc_status_s &esc_status)
{
	if (!_param_com_arm_chk_escs.get()){
		return;
	}
	hrt_abstime now = hrt_absolute_time();

	if (context.status().arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		const int limited_esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);
		const int all_escs_armed_mask = (1 << limited_esc_count) - 1;
		const bool is_all_escs_armed = (all_escs_armed_mask == esc_status.esc_armed_flags);

		_esc_arm_hysteresis.set_hysteresis_time_from(false, 300_ms);
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
		}

	} else {
		// reset ESC bitfield
		_esc_arm_hysteresis.set_state_and_update(false, now);
	}
}
