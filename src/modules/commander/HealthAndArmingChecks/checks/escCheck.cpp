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
		700_ms; // Some DShot ESCs are unresponsive for ~550ms during their initialization, so we use a timeout higher than that

	esc_status_s esc_status;

	if (_esc_status_sub.copy(&esc_status) && now - esc_status.timestamp < esc_telemetry_timeout) {

		checkEscStatus(context, reporter, esc_status);
		reporter.setIsPresent(health_component_t::motors_escs);

	} else if (_param_escs_checks_required.get()
		   && now - _start_time > 5_s) { // Wait a bit after startup to allow esc's to init

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

void EscChecks::checkEscStatus(const Context &context, Report &reporter, const esc_status_s &esc_status)
{
	const NavModes required_modes = _param_escs_checks_required.get() ? NavModes::All : NavModes::None;

	if (esc_status.esc_count > 0) {

		char esc_fail_msg[50];
		esc_fail_msg[0] = '\0';

		int online_bitmask = (1 << esc_status.esc_count) - 1;

		// Check if one or more the ESCs are offline
		if (online_bitmask != esc_status.esc_online_flags) {

			for (int index = 0; index < esc_status.esc_count; index++) {
				if ((esc_status.esc_online_flags & (1 << index)) == 0) {
					uint8_t motor_index = esc_status.esc[index].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + 1;
					/* EVENT
					 * @description
					 * <profile name="dev">
					 * This check can be configured via <param>COM_ARM_CHK_ESCS</param> parameter.
					 * </profile>
					 */
					reporter.healthFailure<uint8_t>(required_modes, health_component_t::motors_escs, events::ID("check_escs_offline"),
									events::Log::Critical, "ESC {1} offline", motor_index);
					snprintf(esc_fail_msg + strlen(esc_fail_msg), sizeof(esc_fail_msg) - strlen(esc_fail_msg), "ESC%d ", motor_index);
					esc_fail_msg[sizeof(esc_fail_msg) - 1] = '\0';
				}
			}

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "%soffline. %s\t", esc_fail_msg, context.isArmed() ? "Land now!" : "");
			}
		}

		for (int index = 0; index < esc_status.esc_count; index++) {

			if (esc_status.esc[index].failures != 0) {

				for (uint8_t fault_index = 0; fault_index <= static_cast<uint8_t>(esc_fault_reason_t::_max); fault_index++) {
					if (esc_status.esc[index].failures & (1 << fault_index)) {

						esc_fault_reason_t fault_reason_index = static_cast<esc_fault_reason_t>(fault_index);

						const char *user_action = "";
						events::px4::enums::suggested_action_t action = events::px4::enums::suggested_action_t::none;

						if (context.isArmed()) {
							if (fault_reason_index == esc_fault_reason_t::motor_warn_temp
							    || fault_reason_index == esc_fault_reason_t::esc_warn_temp) {
								user_action = "Reduce throttle";
								action = events::px4::enums::suggested_action_t::reduce_throttle;

							} else {
								user_action = "Land now!";
								action = events::px4::enums::suggested_action_t::land;
							}
						}

						uint8_t motor_index = esc_status.esc[index].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + 1;

						/* EVENT
						 * @description
						 * {3}
						 *
						 * <profile name="dev">
						 * This check can be configured via <param>COM_ARM_CHK_ESCS</param> parameter.
						 * </profile>
						 */
						reporter.healthFailure<uint8_t, events::px4::enums::esc_fault_reason_t, events::px4::enums::suggested_action_t>(
							required_modes, health_component_t::motors_escs, events::ID("check_escs_fault"),
							events::Log::Critical, "ESC {1}: {2}", motor_index, fault_reason_index, action);

						if (reporter.mavlink_log_pub()) {
							mavlink_log_emergency(reporter.mavlink_log_pub(), "ESC%d: %s. %s \t", motor_index,
									      esc_fault_reason_str(fault_reason_index), user_action);
						}
					}
				}
			}
		}
	}
}

