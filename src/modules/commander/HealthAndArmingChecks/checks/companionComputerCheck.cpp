/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "companionComputerCheck.hpp"

using namespace time_literals;

void CompanionComputerChecks::checkAndReport(const Context &context, Report &reporter)
{
	const float temperature_warn_threshold = _param_com_cc_temp_warn.get();

	if (temperature_warn_threshold < FLT_EPSILON) {
		return;
	}

	onboard_computer_status_s onboard_computer_status;

	// Skip if no companion computer is connected or reporting.
	if (!_onboard_computer_status_sub.copy(&onboard_computer_status)
	    || hrt_elapsed_time(&onboard_computer_status.timestamp) > 5_s
	    || onboard_computer_status.temperature_board == INT8_MAX) {
		return;
	}

	const float temperature_board = (float)onboard_computer_status.temperature_board;

	if (temperature_board >= temperature_warn_threshold) {
		/* EVENT
		 * @description
		 * The companion computer temperature is above the warning threshold.
		 *
		 * <profile name="dev">
		 * The threshold can be adjusted via <param>COM_CC_TEMP_WARN</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure<int8_t>(NavModes::None, health_component_t::system,
					       events::ID("check_companion_computer_temperature_high"),
					       events::Log::Warning, "Companion computer temperature warning, {1} C",
					       onboard_computer_status.temperature_board);
	}
}
