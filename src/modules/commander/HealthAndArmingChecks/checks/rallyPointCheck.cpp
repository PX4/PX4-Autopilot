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
 *    the distribution.
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

#include "rallyPointCheck.hpp"

RallyPointChecks::RallyPointChecks()
	: _param_rtl_type_handle(param_find("RTL_TYPE"))
{
}

void RallyPointChecks::checkAndReport(const Context &context, Report &reporter)
{
	int32_t rtl_type = 0;

	if (param_get(_param_rtl_type_handle, &rtl_type) != 0 || rtl_type != 5) {
		// Only enforce rally point requirement when RTL_TYPE == 5 (safe points only)
		return;
	}

	rtl_status_s rtl_status;

	if (!_rtl_status_sub.copy(&rtl_status) || rtl_status.safe_point_index == UINT8_MAX) {
		/* EVENT
		 * @description
		 * Upload at least one rally point before arming, or change <param>RTL_TYPE</param>.
		 *
		 * <profile name="dev">
		 * This check is active when RTL_TYPE is set to 5 (safe points only).
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system,
					    events::ID("check_rally_point_missing"),
					    events::Log::Error, "No rally point available");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: No rally point available\t");
		}
	}
}
