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
/**
 * @file daaCheck.cpp
 *
 * Prevent arming while an active traffic conflict requires an automated DAA action.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "daaCheck.hpp"

void DaaChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_daa_en.get() <= 0) {
		return;
	}

	if (context.isArmed()) {
		return;
	}

	detect_and_avoid_most_urgent_s daa_status{};
	const hrt_abstime stale_timeout = static_cast<hrt_abstime>(_param_daa_traff_tout.get()) * 1_s;

	if (!_detect_and_avoid_most_urgent_sub.copy(&daa_status)
	    || daa_status.timestamp == 0
	    || hrt_elapsed_time(&daa_status.timestamp) > stale_timeout
	    || !daa_status.has_action) {
		return;
	}

	/* EVENT
	 * @description
	 * Resolve the reported detect-and-avoid conflict before arming.
	 */
	reporter.armingCheckFailure(NavModes::All, health_component_t::traffic_avoidance,
				    events::ID("check_daa_conflict"),
				    events::Log::Error, "Air conflict detected");

	if (reporter.mavlink_log_pub()) {
		mavlink_log_critical(reporter.mavlink_log_pub(), "In conflict with automated DAA action\t");
	}
}
