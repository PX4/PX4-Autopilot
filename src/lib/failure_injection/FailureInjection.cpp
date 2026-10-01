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

#include <lib/mathlib/math/Limits.hpp>
#include <parameters/param.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gnss.h>

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
		if (instance == 0 || (instance <= 16 && (_instance_mask[i] & (1u << (instance - 1))))) {
			return _failure_type[i];
		}
	}

	return Mode::Ok;
}

bool process_battery(const Config &config, uint8_t instance, battery_status_s &battery_status)
{
	const Mode mode = config.mode(failure_injection_s::FAILURE_UNIT_SYSTEM_BATTERY, instance);
	static constexpr float trigger_margin = 0.01f; // report remaining charge just below the threshold

	if (mode == Mode::Off) {
		// Suppress the publication so the pack reads disconnected.
		return false;
	}

	if (mode != Mode::Wrong) {
		return true;
	}

	static const param_t level_handle = param_find("SYS_FAIL_BAT_LVL");
	static const param_t low_thr_handle = param_find("BAT_LOW_THR");
	static const param_t crit_thr_handle = param_find("BAT_CRIT_THR");
	static const param_t emergen_thr_handle = param_find("BAT_EMERGEN_THR");

	int32_t level = battery_status_s::WARNING_CRITICAL;
	param_get(level_handle, &level);

	param_t threshold_handle = crit_thr_handle;

	switch (level) {
	case battery_status_s::WARNING_LOW:
		threshold_handle = low_thr_handle;
		break;

	case battery_status_s::WARNING_EMERGENCY:
		threshold_handle = emergen_thr_handle;
		break;

	case battery_status_s::WARNING_CRITICAL:
	default:
		level = battery_status_s::WARNING_CRITICAL;
		threshold_handle = crit_thr_handle;
		break;
	}

	battery_status.warning = level;


	// Report the remaining charge just below the selected threshold so the
	// matching stage of the low-battery failsafe triggers.
	float threshold = 0.f;
	param_get(threshold_handle, &threshold);
	battery_status.remaining = (threshold > trigger_margin) ? (threshold - trigger_margin) : 0.f;

	return true;
}

namespace
{

int32_t param_int(param_t handle, int32_t fallback)
{
	int32_t value = fallback;
	param_get(handle, &value);
	return value;
}

float param_float(param_t handle)
{
	float value = 0.f;
	param_get(handle, &value);
	return value;
}

void apply_gnss_wrong(sensor_gnss_s &sensor_gnss)
{
	static const param_t fix_type_handle = param_find("SYS_FAIL_GPS_WRG");
	static const param_t eph_handle = param_find("SYS_FAIL_GPS_EPH");
	static const param_t epv_handle = param_find("SYS_FAIL_GPS_EPV");
	static const param_t speed_accuracy_handle = param_find("SYS_FAIL_GPS_SAC");
	static const param_t satellites_handle = param_find("SYS_FAIL_GPS_SAT");
	static const param_t jamming_state_handle = param_find("SYS_FAIL_GPS_JAM");
	static const param_t spoofing_state_handle = param_find("SYS_FAIL_GPS_SPF");

	const int32_t fix_type = param_int(fix_type_handle, sensor_gnss_s::FIX_TYPE_2D);

	if (fix_type > 0) {
		sensor_gnss.fix_type = static_cast<uint8_t>(fix_type);
	}

	const float eph = param_float(eph_handle);

	if (eph > 0.f) {
		sensor_gnss.eph = eph;
	}

	const float epv = param_float(epv_handle);

	if (epv > 0.f) {
		sensor_gnss.epv = epv;
	}

	const float speed_accuracy = param_float(speed_accuracy_handle);

	if (speed_accuracy > 0.f) {
		sensor_gnss.speed_accuracy = speed_accuracy;
	}

	const int32_t satellites = param_int(satellites_handle, 0);

	if (satellites > 0) {
		sensor_gnss.satellites_used = static_cast<uint8_t>(math::min(satellites, static_cast<int32_t>(UINT8_MAX)));
	}

	const int32_t jamming_state = param_int(jamming_state_handle, sensor_gnss_s::JAMMING_STATE_UNKNOWN);

	if (jamming_state != sensor_gnss_s::JAMMING_STATE_UNKNOWN) {
		sensor_gnss.jamming_state = static_cast<uint8_t>(jamming_state);
	}

	const int32_t spoofing_state = param_int(spoofing_state_handle, sensor_gnss_s::SPOOFING_STATE_UNKNOWN);

	if (spoofing_state != sensor_gnss_s::SPOOFING_STATE_UNKNOWN) {
		sensor_gnss.spoofing_state = static_cast<uint8_t>(spoofing_state);
	}
}

} // namespace

bool process_gnss(const Config &config, uint8_t uorb_instance, sensor_gnss_s &sensor_gnss,
		  Stuck<sensor_gnss_s> &stuck)
{
	const Mode mode = config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_GPS, uorb_instance + 1);

	// Off and Stuck are message-agnostic; run them first so the Stuck cache keeps the
	// uncorrupted sample and a later Stuck replays a healthy fix.
	if (!process(mode, sensor_gnss, stuck)) {
		return false;
	}

	if (mode != Mode::Slow) {
		stuck.slow_count = 0;
	}

	switch (mode) {
	case Mode::Wrong:
		apply_gnss_wrong(sensor_gnss);
		break;

	case Mode::Slow: {
			static const param_t divider_handle = param_find("SYS_FAIL_GPS_DIV");
			const int32_t divider = math::max(param_int(divider_handle, 1), static_cast<int32_t>(1));
			const bool pass = (stuck.slow_count == 0);
			stuck.slow_count = (stuck.slow_count + 1) % divider;
			return pass;
		}

	default:
		break;
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

MotorFailureMasks process_motor(const Config &config)
{
	MotorFailureMasks masks{};

	for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
		const uint16_t bit = 1u << i;

		switch (config.mode(failure_injection_s::FAILURE_UNIT_SYSTEM_MOTOR, i + 1)) {
		case Mode::Off:
			masks.failure_mask |= bit;
			break;

		case Mode::Wrong:
			masks.stop_mask |= bit;
			break;

		default:
			break;
		}
	}

	return masks;
}

} // namespace failure_injection

#endif // CONFIG_MODULES_FAILURE_INJECTION_MANAGER
