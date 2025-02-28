/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "batteryCheck.hpp"
#include <lib/circuit_breaker/circuit_breaker.h>

#include <px4_platform_common/events.h>

using namespace time_literals;

using battery_fault_reason_t = events::px4::enums::battery_fault_reason_t;
static_assert(battery_status_s::FAULT_COUNT == (static_cast<uint8_t>(battery_fault_reason_t::_max) + 1)
	      , "Battery fault flags mismatch!");

static constexpr const char *battery_fault_reason_str(battery_fault_reason_t battery_fault_reason)
{
	switch (battery_fault_reason) {
	case battery_fault_reason_t::deep_discharge: return "under voltage";

	case battery_fault_reason_t::voltage_spikes: return "over voltage";

	case battery_fault_reason_t::cell_fail: return "cell fault";

	case battery_fault_reason_t::over_current: return "over current";

	case battery_fault_reason_t::over_temperature: return "over temperature";

	case battery_fault_reason_t::under_temperature: return "under temperature";

	case battery_fault_reason_t::incompatible_voltage: return "voltage mismatch";

	case battery_fault_reason_t::incompatible_firmware: return "incompatible firmware";

	case battery_fault_reason_t::incompatible_model: return "incompatible model";

	case battery_fault_reason_t::hardware_fault: return "hardware fault";

	case battery_fault_reason_t::failed_to_arm: return "failed to arm";

	}

	return "";
};

void BatteryChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (circuit_breaker_enabled_by_val(_param_cbrk_supply_chk.get(), CBRK_SUPPLY_CHK_KEY)) {
		// Reset related failsafe flags otherwise failures from before disabling the check cause failsafes without reported reason
		reporter.failsafeFlags().battery_unhealthy = false;
		reporter.failsafeFlags().battery_low_remaining_time = false;
		reporter.failsafeFlags().battery_warning = battery_status_s::WARNING_NONE;
		return;
	}

	bool battery_has_fault = false;
	// There are possibly multiple batteries, and we can't know which ones serve which purpose. So the safest
	// option is to check if ANY of them have a warning, and specifically find which one has the most
	// urgent warning.
	uint8_t worst_warning = battery_status_s::WARNING_NONE;
	float worst_battery_remaining = 1.f;
	// To make sure that all connected batteries are being regularly reported, we check which one has the
	// oldest timestamp.
	hrt_abstime oldest_update = hrt_absolute_time();
	float worst_battery_time_s{NAN};
	int num_connected_batteries{0};
	bool is_required_battery_missing{false};

	for (auto &battery_sub : _battery_status_subs) {
		int index = battery_sub.get_instance();
		battery_status_s battery;

		if (!battery_sub.copy(&battery)) {
			continue;
		}

		if (battery.is_required && !battery.connected) {
			is_required_battery_missing = true;
			/* EVENT
			 * @description
			 * Make sure all required batteries are connected.
			 */
			reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::battery, events::ID("check_battery_missing"),
							events::Log::Error, "Battery {1} missing", index + 1);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Battery %i missing\t", index + 1);
			}
		}

		if (!_last_armed && context.isArmed()) {
			_battery_connected_at_arming[index] = battery.connected;
		}

		if (context.isArmed() && !battery.connected && _battery_connected_at_arming[index]) { // If disconnected after arming
			/* EVENT
			 */
			reporter.healthFailure<uint8_t>(NavModes::All, health_component_t::battery, events::ID("check_battery_disconnected"),
							events::Log::Emergency, "Battery {1} disconnected", index + 1);

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Battery %i disconnected\t", index + 1);
			}

			// trigger a battery failsafe action if a battery disconnects in flight
			worst_warning = battery_status_s::WARNING_CRITICAL;
		}

		if (battery.connected) {
			++num_connected_batteries;

			if (battery.warning > worst_warning) {
				worst_warning = battery.warning;
			}

			if (battery.remaining < worst_battery_remaining) {
				worst_battery_remaining = battery.remaining;
			}

			if (battery.timestamp < oldest_update) {
				oldest_update = battery.timestamp;
			}

			if (battery.faults > 0) {
				battery_has_fault = true;

				for (uint8_t fault_index = 0; fault_index <= static_cast<uint8_t>(battery_fault_reason_t::_max); fault_index++) {
					if (battery.faults & (1 << fault_index)) {
						events::px4::enums::suggested_action_t action = context.isArmed() ?
								events::px4::enums::suggested_action_t::land :
								events::px4::enums::suggested_action_t::none;

						/* EVENT
						 * @description
						 * The battery reported a failure which might be dangerous to fly with.
						 */
						reporter.healthFailure<uint8_t, battery_fault_reason_t, events::px4::enums::suggested_action_t>
						(NavModes::All, health_component_t::battery, events::ID("check_battery_fault"), {events::Log::Emergency, events::LogInternal::Warning},
						 "Battery {1}: {2}. {3}", index + 1, static_cast<battery_fault_reason_t>(fault_index), action);

						if (reporter.mavlink_log_pub()) {
							mavlink_log_emergency(reporter.mavlink_log_pub(), "Battery %d: %s. %s \t", index + 1,
									      battery_fault_reason_str(
										      static_cast<battery_fault_reason_t>(fault_index)),
									      context.isArmed() ? "Land now!" : "");
						}
					}
				}
			}

			if (PX4_ISFINITE(battery.time_remaining_s)
			    && (!PX4_ISFINITE(worst_battery_time_s)
				|| (PX4_ISFINITE(worst_battery_time_s) && (battery.time_remaining_s < worst_battery_time_s)))) {
				worst_battery_time_s = battery.time_remaining_s;
			}
		}
	}

	if (context.isArmed()) {
		// if armed, only allow increase of battery warning severity
		if (worst_warning > reporter.failsafeFlags().battery_warning) {
			reporter.failsafeFlags().battery_warning = worst_warning;
		}

	} else {
		reporter.failsafeFlags().battery_warning = worst_warning;
	}

	const bool battery_warning = reporter.failsafeFlags().battery_warning > battery_status_s::WARNING_NONE
				     && reporter.failsafeFlags().battery_warning < battery_status_s::WARNING_FAILED;
	const bool configured_arm_threshold_in_use = !context.isArmed() && (_param_com_arm_bat_min.get() >= -FLT_EPSILON);
	const bool below_configured_arm_threshold = (worst_battery_remaining < _param_com_arm_bat_min.get());

	if (battery_warning || (configured_arm_threshold_in_use && below_configured_arm_threshold)) {
		const bool critical_or_higher = reporter.failsafeFlags().battery_warning >= battery_status_s::WARNING_CRITICAL;
		NavModes affected_modes = (!configured_arm_threshold_in_use && critical_or_higher)
					  || (configured_arm_threshold_in_use && below_configured_arm_threshold) ? NavModes::All : NavModes::None;
		events::LogLevel log_level = critical_or_higher || below_configured_arm_threshold
					     ? events::Log::Critical : events::Log::Warning;

		switch (reporter.failsafeFlags().battery_warning) {
		default:
		case battery_status_s::WARNING_LOW:
			/* EVENT
			* @description
			* The lowest battery state of charge is below the low threshold.
			*
			* <profile name="dev">
			* Can be configured with <param>BAT_LOW_THR</param>.
			* </profile>
			*/
			reporter.armingCheckFailure(affected_modes, health_component_t::battery, events::ID("check_battery_low"),
						    log_level, "Low battery");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_emergency(reporter.mavlink_log_pub(), "Low battery\t");
			}

			break;

		case battery_status_s::WARNING_CRITICAL:
			/* EVENT
			* @description
			* The lowest battery state of charge is below the critical threshold.
			*
			* <profile name="dev">
			* Can be configured with <param>BAT_CRIT_THR</param> and from when to disalow arming with <param>COM_ARM_BAT_MIN</param>.
			* </profile>
			*/
			reporter.armingCheckFailure(affected_modes, health_component_t::battery, events::ID("check_battery_critical"),
						    log_level, "Critical battery");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_emergency(reporter.mavlink_log_pub(), "Critical battery\t");
			}

			break;

		case battery_status_s::WARNING_EMERGENCY:
			/* EVENT
			* @description
			* The lowest battery state of charge is below the emergency threshold.
			*
			* <profile name="dev">
			* Can be configured with <param>BAT_EMERGEN_THR</param>.
			* </profile>
			*/
			reporter.armingCheckFailure(affected_modes, health_component_t::battery, events::ID("check_battery_emergency"),
						    log_level, "Emergency battery level");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_emergency(reporter.mavlink_log_pub(), "Emergency battery level\t");
			}

			break;
		}

	}

	rtlEstimateCheck(context, reporter, worst_battery_time_s);

	reporter.failsafeFlags().battery_unhealthy =
		// All connected batteries are regularly being published
		hrt_elapsed_time(&oldest_update) > 5_s
		// There is a required battery that's missing
		|| is_required_battery_missing
		// No currently-connected batteries have any fault
		|| battery_has_fault
		|| reporter.failsafeFlags().battery_warning == battery_status_s::WARNING_FAILED;

	if (reporter.failsafeFlags().battery_unhealthy
	    && !is_required_battery_missing && !battery_has_fault) { // missing batteries and faults are reported above already
		/* EVENT
		 * @description
		 * Make sure all batteries are operational.
		 */
		reporter.healthFailure(NavModes::All, health_component_t::battery, events::ID("check_battery_unhealthy"),
				       events::Log::Error, "Battery unhealthy");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Battery unhealthy");
		}
	}

	if (num_connected_batteries > 0) {
		reporter.setIsPresent(health_component_t::battery);
	}

	_last_armed = context.isArmed();
}

void BatteryChecks::rtlEstimateCheck(const Context &context, Report &reporter, float worst_battery_time_s)
{
	rtl_time_estimate_s rtl_time_estimate;

	// Compare estimate of RTL time to estimate of remaining flight time
	// add hysteresis: if already in the condition, only get out of it if the remaining flight time is significantly higher again
	const float hysteresis_factor = reporter.failsafeFlags().battery_low_remaining_time ? 1.1f : 1.0f;

	reporter.failsafeFlags().battery_low_remaining_time = _rtl_time_estimate_sub.copy(&rtl_time_estimate)
			&& (hrt_absolute_time() - rtl_time_estimate.timestamp) < 3_s
			&& rtl_time_estimate.valid
			&& context.isArmed()
			&& PX4_ISFINITE(worst_battery_time_s)
			&& rtl_time_estimate.safe_time_estimate * hysteresis_factor >= worst_battery_time_s;


	if (reporter.failsafeFlags().battery_low_remaining_time) {
		/* EVENT
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::battery, events::ID("check_battery_rem_flight_time_low"),
					    events::Log::Error, "Remaining flight time low");
	}

}
