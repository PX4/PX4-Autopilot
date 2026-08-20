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
#include <uORB/topics/sensor_gps.h>

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

void process_battery(const Config &config, uint8_t instance, battery_status_s &battery_status)
{
	if (config.mode(failure_injection_s::FAILURE_UNIT_SYSTEM_BATTERY, instance) != Mode::Off) {
		return;
	}

	// Report a depleted pack so the low-battery failsafe triggers.
	battery_status.remaining = 0.f;
	battery_status.warning = battery_status_s::WARNING_EMERGENCY;
}

bool process_gnss(const Config &config, uint8_t uorb_instance, sensor_gps_s &sensor_gps,
		  Stuck<sensor_gps_s> &stuck)
{
	const Mode mode = config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_GPS, uorb_instance + 1);

	// Off and Stuck are message-agnostic; run them first so the Stuck cache keeps the
	// uncorrupted sample and a later Stuck replays a healthy fix.
	if (!process(mode, sensor_gps, stuck)) {
		return false;
	}

	if (mode == Mode::Wrong) {
		static const param_t fix_type_handle = param_find("SYS_FAIL_GPS_WRG");

		int32_t fix_type = sensor_gps_s::FIX_TYPE_2D;
		param_get(fix_type_handle, &fix_type);
		sensor_gps.fix_type = (uint8_t)fix_type;

		static const param_t jamming_state_handle = param_find("SYS_FAIL_GPS_JAM");

		int32_t jamming_state = sensor_gps_s::JAMMING_STATE_UNKNOWN;
		param_get(jamming_state_handle, &jamming_state);

		if (jamming_state != sensor_gps_s::JAMMING_STATE_UNKNOWN) {
			sensor_gps.jamming_state = (uint8_t)jamming_state;
		}
	}

	return true;
}

esc_status_s process_esc(const Config &config, const esc_status_s &status)
{
	esc_status_s result = status;

	for (int i = 0; i < result.esc_count && i < esc_status_s::CONNECTED_ESC_MAX; i++) {
		const uint8_t function = result.esc[i].actuator_function;

		if (function < esc_report_s::ACTUATOR_FUNCTION_MOTOR1 || function > esc_report_s::ACTUATOR_FUNCTION_MOTOR_MAX) {
			continue; // not a motor output
		}

		const uint8_t instance = function - esc_report_s::ACTUATOR_FUNCTION_MOTOR1 + 1; // 1-based ESC instance

		switch (config.mode(failure_injection_s::FAILURE_UNIT_SYSTEM_ESC, instance)) {
		case Mode::Off:
			result.esc_online_flags &= ~(1u << i);
			result.esc_armed_flags &= ~(1u << i);
			result.esc[i] = esc_report_s{};
			result.esc[i].actuator_function = function;
			break;

		case Mode::Wrong:
			result.esc[i].esc_voltage *= 0.1f;
			result.esc[i].esc_current *= 0.1f;
			result.esc[i].esc_rpm *= 10;
			break;

		default:
			break;
		}
	}

	return result;
}

} // namespace failure_injection

#endif // CONFIG_MODULES_FAILURE_INJECTION_MANAGER
