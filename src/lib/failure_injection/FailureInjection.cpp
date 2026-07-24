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

#include "FailureInjection.hpp"

#if defined(CONFIG_MODULES_FAILURE_INJECTION_MANAGER)

#include <parameters/param.h>
#include <uORB/topics/battery_status.h>

namespace failure_injection
{

bool Config::update()
{
	failure_injection_s cfg;

	if (_sub.update(&cfg)) {
		set(cfg);
		return true;
	}

	return false;
}

void Config::set(const failure_injection_s &cfg)
{
	_count = (cfg.count <= failure_injection_s::MAX_FAILURES) ? cfg.count : failure_injection_s::MAX_FAILURES;

	for (uint8_t i = 0; i < _count; i++) {
		_unit[i] = cfg.unit[i];
		_instance_mask[i] = cfg.instance_mask[i];
		_failure_type[i] = static_cast<Mode>(cfg.failure_type[i]);
	}
}

Mode Config::mode(uint8_t unit, uint8_t instance) const
{
	for (uint8_t i = 0; i < _count; i++) {
		if (_unit[i] != unit) {
			continue;
		}

		// instance == 0 matches any instance of the unit; otherwise match the
		// 1-based instance against the bitmask (0xFFFF covers all instances).
		if (instance == 0 || (_instance_mask[i] & (1u << (instance - 1)))) {
			return _failure_type[i];
		}
	}

	return Mode::Ok;
}

bool process_battery(const Config &config, uint8_t instance, battery_status_s &battery_status)
{
	const Mode mode = config.mode(failure_injection_s::FAILURE_UNIT_SYSTEM_BATTERY, instance);

	if (mode == Mode::Off) {
		// Suppress the publication so the pack reads disconnected.
		return false;
	}

	if (mode == Mode::Wrong) {
		static const param_t level_handle = param_find("SYS_FAIL_BAT_LVL");
		static const param_t low_thr_handle = param_find("BAT_LOW_THR");
		static const param_t crit_thr_handle = param_find("BAT_CRIT_THR");
		static const param_t emergen_thr_handle = param_find("BAT_EMERGEN_THR");

		int32_t level = battery_status_s::WARNING_CRITICAL;
		param_get(level_handle, &level);

		param_t threshold_handle = crit_thr_handle;

		switch (level) {
		case battery_status_s::WARNING_LOW:
			battery_status.warning = battery_status_s::WARNING_LOW;
			threshold_handle = low_thr_handle;
			break;

		case battery_status_s::WARNING_EMERGENCY:
			battery_status.warning = battery_status_s::WARNING_EMERGENCY;
			threshold_handle = emergen_thr_handle;
			break;

		case battery_status_s::WARNING_CRITICAL:
		default:
			battery_status.warning = battery_status_s::WARNING_CRITICAL;
			break;
		}

		// Report the remaining charge just below the selected threshold so the
		// matching stage of the low-battery failsafe triggers.
		float threshold = 0.f;
		param_get(threshold_handle, &threshold);
		battery_status.remaining = (threshold > 0.01f) ? (threshold - 0.01f) : 0.f;
	}

	return true;
}

} // namespace failure_injection

#endif // CONFIG_MODULES_FAILURE_INJECTION_MANAGER
