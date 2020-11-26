/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "PowerChecks.hpp"


static unsigned int countSetBits(unsigned int n)
{
	unsigned int count = 0;

	while (n) {
		count += n & 1;
		n >>= 1;
	}

	return count;
}

void PowerChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (context.isArmed()) {
		// Ignore power check after arming.
		return;
	}

	if (context.status_flags().circuit_breaker_engaged_power_check) {
		return;
	}

	system_power_s system_power;

	if (!_system_power_sub.copy(&system_power) || system_power.timestamp == 0) {
		/* EVENT
		 * @description
		 *  <profile name="dev">
		 *  This check can be disabled with the parameter <param>CBRK_SUPPLY_CHK</param>.
		 *  </profile>
		 */
		reporter.healthFailure(ModeCategory::All, HealthComponentIndex::battery, events::ID("health_power_unavail"),
				       "System power unavailable", events::Log::Critical);
		return;
	}

	// Check avionics rail voltages (if USB isn't connected)
	if (!system_power.usb_connected) {
		float avionics_power_rail_voltage = system_power.voltage5v_v;

		if (avionics_power_rail_voltage < 4.8f) {
			events::Log log_level = events::Log::Error;
			ModeCategory mode_category = ModeCategory::None;

			if (avionics_power_rail_voltage < 4.5f) { // only deny arming if below 4.5 V
				log_level = events::Log::Critical;
				mode_category = ModeCategory::All;
			}

			/* EVENT
			 * @description
			 *  Check your cabling and the 5V power source.
			 *
			 *  <profile name="dev">
			 *  This check can be disabled with the parameter <param>CBRK_SUPPLY_CHK</param>.
			 *  </profile>
			 *  @arg1: avionics_power_rail_voltage
			 */
			reporter.healthFailure<float>(mode_category, HealthComponentIndex::battery,
						      events::ID("health_power_avionics_power_low"),
						      "Avionics Power low: {1:.2} Volt", log_level, avionics_power_rail_voltage);

		} else if (avionics_power_rail_voltage > 5.4f) {
			/* EVENT
			 * @description
			 *  Check your cabling and the 5V power source.
			 *
			 *  <profile name="dev">
			 *  This check can be disabled with the parameter <param>CBRK_SUPPLY_CHK</param>.
			 *  </profile>
			 *  @arg1: avionics_power_rail_voltage
			 */
			reporter.healthFailure<float>(ModeCategory::None, HealthComponentIndex::battery,
						      events::ID("health_power_avionics_power_high"),
						      "Avionics Power high: {1:.2} Volt", events::Log::Critical, avionics_power_rail_voltage);
		}

		const int power_module_count = countSetBits(system_power.brick_valid);

		if (power_module_count < _param_com_power_count.get()) {
			/* EVENT
			 * @description
			 *  Ensure to insert all batteries.
			 *
			 *  <profile name="dev">
			 *  This can be configured with <param>COM_POWER_COUNT</param>.
			 *  </profile>
			 *  @arg1: power_module_count
			 *  @arg2: required_power_module_count
			 */
			reporter.healthFailure<uint8_t, uint8_t>(ModeCategory::All, HealthComponentIndex::battery,
					events::ID("health_power_redudancy"),
					"Power redundancy not met: {1} instead of {2}", events::Log::Warning, power_module_count, _param_com_power_count.get());
		}
	}

}

