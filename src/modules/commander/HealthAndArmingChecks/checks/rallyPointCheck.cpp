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

	if (param_get(_param_rtl_type_handle, &rtl_type) != 0) {
		return;
	}

	static constexpr int32_t RTL_TYPE_ROUTE_SAFE_POINT = 7; // RTL_TYPE value, see navigator rtl_params.yaml

	if (rtl_type == RTL_TYPE_ROUTE_SAFE_POINT) {
#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
		checkRouteReturn(context, reporter);
#else
		/* EVENT
		 * @description
		 * This firmware is built without mission route planning, so Return uses the destination selection of
		 * <param>RTL_TYPE</param> 3 instead of following the mission route.
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::system,
					    events::ID("check_rtl_type_route_unsupported"),
					    events::Log::Warning, "RTL_TYPE 7 unavailable, using type 3");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_warning(reporter.mavlink_log_pub(), "RTL_TYPE 7 unavailable, using type 3\t");
		}

#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
		return;
	}

	if (rtl_type != 5) {
		// Only enforce rally point requirement when RTL_TYPE == 5 (safe points only)
		return;
	}

	if (!_rtl_status_sub.advertised()) {
		return;
	}

	rtl_status_s rtl_status;

	if (!_rtl_status_sub.copy(&rtl_status) || rtl_status.safe_point_index == UINT8_MAX) {
		/* EVENT
		 * @description
		 * No rally point is configured. Return mode will fall back to the current position when triggered.
		 * Upload at least one rally point, or change <param>RTL_TYPE</param> to silence this warning.
		 *
		 * <profile name="dev">
		 * This warning is active when RTL_TYPE is set to 5 (safe points only).
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::None, health_component_t::system,
					    events::ID("check_rally_point_missing"),
					    events::Log::Warning, "No rally point configured");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_warning(reporter.mavlink_log_pub(), "No rally point configured\t");
		}
	}
}

#if CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE > 0
void RallyPointChecks::checkRouteReturn(const Context &context, Report &reporter)
{
	// Navigator plans the route every 2 s, also while its inputs load after an upload.
	// An active Return handles the loss of its route itself.
	if (context.status().nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		return;
	}

	rtl_status_s rtl_status{};
	// Allow two planning intervals plus scheduling margin; absence is not evidence of a usable route.
	static constexpr hrt_abstime ROUTE_STATUS_TIMEOUT = 5_s;

	if (_rtl_status_sub.copy(&rtl_status) && rtl_status.timestamp != 0
	    && hrt_elapsed_time(&rtl_status.timestamp) < ROUTE_STATUS_TIMEOUT
	    && rtl_status.rtl_type == rtl_status_s::RTL_STATUS_TYPE_FOLLOW_MISSION_SAFE_POINT) {
		return;
	}

	const NavModes return_mode = static_cast<NavModes>(1u << vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	/* EVENT
	 * @description
	 * Return cannot follow the mission route from the current position, so Return mode is unavailable:
	 * a mode switch is denied and a failsafe that would return lands instead.
	 * This also applies for a few seconds after an upload, while the mission, rally points and fence load.
	 * A missing or stale route-status report also keeps Return unavailable.
	 * Upload a mission the vehicle can join and a rally point or landing it can reach through it,
	 * or change <param>RTL_TYPE</param>.
	 */
	reporter.armingCheckFailure(return_mode, health_component_t::system,
				    events::ID("check_rtl_route_unavailable"),
				    events::Log::Warning, "Return unavailable: no mission route");

	// A selected Return that a failsafe interrupts also handles a missing route itself: only warn.
	if (context.status().nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
		return;
	}

	// Deny mode switches and let failsafes fall back, as for an unmet external mode requirement.
	reporter.clearCanRunBits(return_mode);
	reporter.failsafeFlags().mode_req_other |= static_cast<uint32_t>(return_mode);
}
#endif // CONFIG_NAVIGATOR_FULL_MISSION_CACHE_SIZE
