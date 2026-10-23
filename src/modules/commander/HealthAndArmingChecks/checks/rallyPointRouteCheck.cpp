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

// RTL_TYPE 7 route check. Built only with the mission cache, so its event stays out of the metadata of boards
// without it.

#include "rallyPointCheck.hpp"

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
	// Report in every mode, as ground stations list only the checks of the selected mode. Only Return fails.
	/* EVENT
	 * @description
	 * Return cannot follow the mission route from the current position, or the uploaded mission, rally points
	 * or fence are still loading.
	 * Unless Return is the selected mode, a switch to Return is denied and a failsafe that would return
	 * lands instead.
	 * Upload a mission the vehicle can join and a rally point or landing it can reach through it,
	 * or change <param>RTL_TYPE</param>.
	 */
	reporter.armingCheckFailure(NavModesMessageFail{NavModes::All, return_mode}, health_component_t::system,
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
