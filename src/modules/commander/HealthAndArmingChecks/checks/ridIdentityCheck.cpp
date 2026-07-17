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

#include "ridIdentityCheck.hpp"

#if defined(CONFIG_BOARD_ODID_RID_IDENTITY)
#include <lib/rid_identity_record/rid_identity_record.h>
#endif

void RidIdentityChecks::checkAndReport(const Context &context, Report &reporter)
{
#if defined(CONFIG_BOARD_ODID_RID_IDENTITY)
	const bool is_hil = context.status().hil_state == vehicle_status_s::HIL_STATE_ON;
	const bool unhealthy = !rid_identity_is_valid();

	bool log_transition = false;

	if (!_state_initialized) {
		_state_initialized = true;
		log_transition = unhealthy;

	} else {
		log_transition = unhealthy && (_last_unhealthy != unhealthy || _last_hil != is_hil);
	}

	_last_unhealthy = unhealthy;
	_last_hil = is_hil;

	if (!unhealthy) {
		return;
	}

	// TODO: Promote this check to arming-blocking once staged integration validation is complete.
	const NavModes affected_modes = NavModes::None;
	const events::Log log_level = events::Log::Warning;

	/* EVENT
	 * @description
	 * Factory Remote ID identity verification failed. The aircraft serial
	 * does not match this MCU or the stored identity data is invalid.
	 */
	reporter.healthFailure(affected_modes, health_component_t::open_drone_id,
			       events::ID("check_rid_identity_invalid"),
			       log_level, "Remote ID identity verification failed");

	if (log_transition && reporter.mavlink_log_pub()) {
		mavlink_log_warning(reporter.mavlink_log_pub(),
				    "Preflight Warn: Remote ID identity verification failed");
	}

#else
	(void)context;
	(void)reporter;
#endif
}
