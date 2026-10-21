/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "openDroneIDCheck.hpp"
#include <lib/remote_id/RidState.hpp>


void OpenDroneIDChecks::checkAndReport(const Context &context, Report &reporter)
{
	reporter.failsafeFlags().remote_id_unhealthy =
		!context.status().open_drone_id_system_present ||
		!context.status().open_drone_id_system_healthy;

	checkProduct(context, reporter);

	if (!_param_com_arm_odid.get()) {
		return;
	}

	const bool is_error = _param_com_arm_odid.get() >= 2;
	const NavModes affected_modes = is_error ? NavModes::All : NavModes::None;
	const events::Log log_level = is_error ? events::Log::Error : events::Log::Warning;

	if (!context.status().open_drone_id_system_present) {
		/* EVENT
		 * @description
		 * Open Drone ID system failed to report. Make sure it is setup and installed properly.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_ODID</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(affected_modes, health_component_t::open_drone_id,
				       events::ID("check_open_drone_id_missing"),
				       log_level, "Open Drone ID system missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Open Drone ID system missing");
		}

	} else if (!context.status().open_drone_id_system_healthy) {
		/* EVENT
		 * @description
		 * Open Drone ID system reported being unhealthy.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_ODID</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(affected_modes, health_component_t::open_drone_id,
				       events::ID("check_open_drone_id_unhealthy"),
				       log_level, "Open Drone ID system not ready");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Open Drone ID system not ready");
		}

	}
}

void OpenDroneIDChecks::checkProduct(const Context &context, Report &reporter)
{
	reporter.failsafeFlags().uom_unhealthy = false;

	if (!_param_com_rid_uom.get()) { return; }

	rid_identity_s identity{};
	rid_uom_status_s uom{};
	open_drone_id_arm_status_s arm{};
	_identity_sub.copy(&identity);
	_uom_sub.copy(&uom);
	_arm_sub.copy(&arm);
	const uint64_t now = hrt_absolute_time();
	const bool identity_ready = rid::identityReady(identity, now);
	const bool rid_ready = identity_ready && arm.node_id == identity.rid_node_id && arm.status == 0
			       && rid::recent(arm.timestamp, now, rid::HEARTBEAT_TIMEOUT_US);
#if defined(CONFIG_RID_UOM_TEST_MOCK)
	static constexpr bool allow_mock = true;
#else
	static constexpr bool allow_mock = false;
#endif
	const bool uom_ready = rid::uomReady(identity, uom, now, allow_mock);
	reporter.failsafeFlags().uom_unhealthy = !uom_ready;
	const NavModes modes = context.isArmed() ? NavModes::None : NavModes::All;
	const events::Log severity = context.isArmed() ? events::Log::Warning : events::Log::Error;

	if (!identity_ready) {
		/* EVENT
		 * @description
		 * Product RID/UOM readiness is required before arming. UOM faults in flight only warn.
		 */
		reporter.armingCheckFailure(modes, health_component_t::open_drone_id,
					    events::ID("check_rid_identity_activation"), severity, "RID identity or activation not ready");

	} else if (!rid_ready) {
		/* EVENT
		 * @description
		 * Product RID/UOM readiness is required before arming. UOM faults in flight only warn.
		 */
		reporter.armingCheckFailure(modes, health_component_t::open_drone_id,
					    events::ID("check_rid_bound_health"), severity, "Bound RID health is missing or stale");
	}

	if (!uom_ready) {
		/* EVENT
		 * @description
		 * Product RID/UOM readiness is required before arming. UOM faults in flight only warn.
		 */
		reporter.armingCheckFailure(modes, health_component_t::system,
					    events::ID("check_uom_not_ready"), severity, "UOM reporting service not ready");
	}

	// UOM is deliberately not part of remote_id_unhealthy or any mode-changing failsafe.
	if (context.isArmed() && uom_ready != _last_uom_ready) {
		if (uom_ready) {
			events::send(events::ID("uom_reporting_restored"), events::Log::Info, "UOM reporting restored");

		} else {
			events::send(events::ID("uom_reporting_lost"), events::Log::Warning, "UOM reporting unavailable; flight mode unchanged");
		}
	}

	_last_uom_ready = uom_ready;
}
