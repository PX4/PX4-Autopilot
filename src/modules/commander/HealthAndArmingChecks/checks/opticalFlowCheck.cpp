/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "opticalFlowCheck.hpp"

void OpticalFlowCheck::checkAndReport(const Context &context, Report &reporter)
{
	if (!_param_sys_has_num_of.get()) {
		return;
	}

	const bool exists = _vehicle_optical_flow_sub.advertised();

	if (exists) {
		vehicle_optical_flow_s flow_sens;
		const bool valid = _vehicle_optical_flow_sub.copy(&flow_sens) && (hrt_elapsed_time(&flow_sens.timestamp) < 1_s);
		reporter.setIsPresent(health_component_t::optical_flow);

		if (!valid) {
			/* EVENT
			 */
			reporter.healthFailure(NavModes::All, health_component_t::optical_flow,
					       events::ID("check_optical_flow_sensor_invalid"),
					       events::Log::Error, "No valid data from optical flow sensor");
		}

	} else {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>SYS_HAS_NUM_OF</param> parameter.
		 * </profile>
		 */
		reporter.healthFailure(NavModes::All, health_component_t::optical_flow,
				       events::ID("check_optical_sensor_missing"),
				       events::Log::Error, "Optical flow sensor missing");
	}
}
