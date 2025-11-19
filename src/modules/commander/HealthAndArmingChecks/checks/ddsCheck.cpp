/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "ddsCheck.hpp"

using namespace time_literals;

void DdsChecks::checkAndReport(const Context &context, Report &reporter)
{
	// Check if DDS connection is required for arming
	// 0: Disabled (DDS connection not required)
	// 1: Warning only (allow arming but warn if DDS not connected)
	// 2: Required (prevent arming if DDS not connected)
	if (_param_com_arm_dds.get() == 0) {
		return; // DDS check disabled
	}

	dds_flag_s dds_flag;
	bool dds_connected = false;

	// Check DDS connection status - always check latest state, not just on updates
	// This ensures errors are cleared immediately when DDS becomes connected
	if (_dds_flag_sub.copy(&dds_flag)) {
		dds_connected = dds_flag.dds_connected;
	}

	if (!dds_connected) {
		NavModes affected_modes = (_param_com_arm_dds.get() == 2) ? NavModes::All : NavModes::None;
		events::LogLevel log_level = (_param_com_arm_dds.get() == 2) ? events::Log::Error : events::Log::Warning;

		/* EVENT
		 * @description
		 * DDS (Data Distribution Service) connection is not established or lost.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>COM_ARM_DDS</param> parameter.
		 * 0: Disabled, 1: Warning only, 2: Required for arming.
		 * </profile>
		 */
		reporter.armingCheckFailure(affected_modes, health_component_t::communication_links,
					    events::ID("check_dds_not_connected"),
					    log_level, "DDS connection not established");

		if (_param_com_arm_dds.get() == 2 && reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: DDS connection not established");
		} else if (_param_com_arm_dds.get() == 1 && reporter.mavlink_log_pub()) {
			mavlink_log_warning(reporter.mavlink_log_pub(), "Preflight Warning: DDS connection not established");
		}

	} else {
		reporter.setIsPresent(health_component_t::communication_links);
	}
}

