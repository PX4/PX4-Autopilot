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

#include "windCheck.hpp"

void WindChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_com_wind_warn.get() < FLT_EPSILON && _param_com_wind_max.get() < FLT_EPSILON) {
		reporter.failsafeFlags().wind_limit_exceeded = false;
		return;
	}

	wind_s wind_estimate;
	const hrt_abstime now = hrt_absolute_time();

	if (_wind_sub.copy(&wind_estimate)) {
		const matrix::Vector2f wind(wind_estimate.windspeed_north, wind_estimate.windspeed_east);

		// publish a warning if it's the first since in air or 60s have passed since the last warning
		const bool warning_timeout_passed = _last_wind_warning == 0 || now - _last_wind_warning > 60_s;
		const bool wind_limit_exceeded = _param_com_wind_max.get() > FLT_EPSILON && wind.longerThan(_param_com_wind_max.get());
		reporter.failsafeFlags().wind_limit_exceeded = false; // reset, will be set below if needed

		if (_param_com_wind_max_act.get() > 1 && wind_limit_exceeded) {

			// only set failsafe flag if the high wind failsafe action is higher than warning
			reporter.failsafeFlags().wind_limit_exceeded = true;

			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>COM_WIND_MAX</param> and <param>COM_WIND_MAX_ACT</param> parameters.
			 * </profile>
			 */
			reporter.armingCheckFailure<float>(NavModes::All, health_component_t::system,
							   events::ID("check_wind_too_high"),
							   events::Log::Warning, "Wind speed is above limit ({1:.1m/s})", wind.norm());

		} else if (_param_com_wind_max_act.get() == 1 // warning only
			   && wind_limit_exceeded
			   && warning_timeout_passed
			   && context.status().nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
			   && context.status().nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {

			events::send<float>(events::ID("check_above_wind_limits_warning"),
			{events::Log::Warning, events::LogInternal::Warning},
			"Wind speed above limit ({1:.1m/s}), landing advised", wind.norm());
			_last_wind_warning = now;

		} else if (_param_com_wind_warn.get() > FLT_EPSILON
			   && wind.longerThan(_param_com_wind_warn.get())
			   && warning_timeout_passed
			   && context.status().nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
			   && context.status().nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {

			events::send<float>(events::ID("check_high_wind_warning"),
			{events::Log::Warning, events::LogInternal::Info},
			"High wind speed detected ({1:.1m/s}), landing advised", wind.norm());
			_last_wind_warning = now;
		}
	}
}
