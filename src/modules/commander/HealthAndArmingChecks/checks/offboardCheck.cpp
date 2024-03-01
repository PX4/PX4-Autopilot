/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "offboardCheck.hpp"

using namespace time_literals;

void OffboardChecks::checkAndReport(const Context &context, Report &reporter)
{
	reporter.failsafeFlags().offboard_control_signal_lost = true;

	offboard_control_mode_s offboard_control_mode;

	if (_offboard_control_mode_sub.copy(&offboard_control_mode)) {

		bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
				      + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);

		bool offboard_available = (offboard_control_mode.position || offboard_control_mode.velocity
					   || offboard_control_mode.acceleration || offboard_control_mode.attitude || offboard_control_mode.body_rate
					   || offboard_control_mode.thrust_and_torque || offboard_control_mode.direct_actuator) && data_is_recent;

		if (offboard_control_mode.position && reporter.failsafeFlags().local_position_invalid) {
			offboard_available = false;

		} else if (offboard_control_mode.velocity && reporter.failsafeFlags().local_velocity_invalid) {
			offboard_available = false;

		} else if (offboard_control_mode.acceleration && reporter.failsafeFlags().local_velocity_invalid) {
			// OFFBOARD acceleration handled by position controller
			offboard_available = false;
		}

		// This is a mode requirement, no need to report
		reporter.failsafeFlags().offboard_control_signal_lost = !offboard_available;
	}
}
