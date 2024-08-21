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

#include "powerCheck.hpp"
#include <lib/circuit_breaker/circuit_breaker.h>

using namespace time_literals;

void PowerChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (circuit_breaker_enabled_by_val(_param_cbrk_supply_chk.get(), CBRK_SUPPLY_CHK_KEY)) {
		return;
	}

	if (context.status().hil_state == vehicle_status_s::HIL_STATE_ON) {
		// Ignore power check in HITL.
		return;
	}

	if (!context.status().power_input_valid) {

		/* EVENT
		 * @description
		 * Note that USB must be disconnected as well.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>CBRK_SUPPLY_CHK</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::system, events::ID("check_avionics_power_missing"),
				       events::Log::Error, "Power module not connected");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Power module not connected");
		}

		return;
	}

	system_power_s system_power;

	if (_system_power_sub.copy(&system_power)) {
		// Check avionics rail voltages (if USB isn't connected)
		if (!system_power.usb_connected) {
			float avionics_power_rail_voltage = system_power.voltage5v_v;

			const float low_error_threshold = 4.7f;
			const float high_error_threshold = 5.4f;

			if (avionics_power_rail_voltage < low_error_threshold) {

				/* EVENT
				 * @description
				 * Check the voltage supply to the FMU, it must be above {2:.2} Volt.
				 *
				 * <profile name="dev">
				 * This check can be configured via <param>CBRK_SUPPLY_CHK</param> parameter.
				 * </profile>
				 */
				reporter.healthFailure<float, float>(NavModes::All, health_component_t::system,
								     events::ID("check_avionics_power_low"),
								     events::Log::Error, "Avionics Power low: {1:.2} Volt", avionics_power_rail_voltage, low_error_threshold);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Avionics Power low: %6.2f Volt",
							     (double)avionics_power_rail_voltage);
				}

			} else if (avionics_power_rail_voltage > high_error_threshold) {
				/* EVENT
				 * @description
				 * Check the voltage supply to the FMU, it must be below {2:.2} Volt.
				 *
				 * <profile name="dev">
				 * This check can be configured via <param>CBRK_SUPPLY_CHK</param> parameter.
				 * </profile>
				 */
				reporter.healthFailure<float, float>(NavModes::All, health_component_t::system,
								     events::ID("check_avionics_power_high"),
								     events::Log::Error, "Avionics Power high: {1:.2} Volt", avionics_power_rail_voltage, high_error_threshold);

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Avionics Power high: %6.2f Volt",
							     (double)avionics_power_rail_voltage);
				}
			}

			const int power_module_count = math::countSetBits(system_power.brick_valid);

			if (power_module_count < _param_com_power_count.get()) {
				/* EVENT
				 * @description
				 * Available power modules: {1}.
				 * Required power modules: {2}.
				 *
				 * <profile name="dev">
				 * This check can be configured via <param>COM_POWER_COUNT</param> parameter.
				 * </profile>
				 */
				reporter.armingCheckFailure<uint8_t, uint8_t>(NavModes::All, health_component_t::system,
						events::ID("check_avionics_power_redundancy"),
						events::Log::Error, "Power redundancy not met", power_module_count, _param_com_power_count.get());

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Power redundancy not met: %d instead of %" PRId32 "",
							     power_module_count, _param_com_power_count.get());
				}
			}

			// Overcurrent detection
			if (system_power.hipower_5v_oc) {
				/* EVENT
				 * @description
				 * Check the power supply
				 */
				reporter.healthFailure(NavModes::All, health_component_t::system,
						       events::ID("check_power_oc_hipower"),
						       events::Log::Error, "Overcurrent detected for the hipower 5V supply");
			}

			if (system_power.periph_5v_oc) {
				/* EVENT
				 * @description
				 * Check the power supply
				 */
				reporter.healthFailure(NavModes::All, health_component_t::system,
						       events::ID("check_power_oc_periph"),
						       events::Log::Error, "Overcurrent detected for the peripheral 5V supply");
			}

			if (system_power.hipower_5v_oc || system_power.periph_5v_oc) {
				if (context.isArmed() && !_overcurrent_warning_sent) {
					_overcurrent_warning_sent = true;
					events::send(events::ID("check_power_oc_report"),
						     events::Log::Error,
						     "5V overcurrent detected, landing advised");
				}
			}
		}

	} else {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * Ensure the ADC is working and the system_power topic is published.
		 *
		 * This check can be configured via <param>CBRK_SUPPLY_CHK</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::system, events::ID("check_missing_system_power"),
				       events::Log::Error, "System power unavailable");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: system power unavailable");
		}
	}
}
