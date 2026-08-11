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
 * @file FailureStrings.hpp
 *
 * Human-readable names for the failure injection announcement: failure unit
 * and type names (matching the keys of the `failure` console command) and the
 * instance-mask-to-phrase formatter. Header-only and free of the uORB runtime
 * so it can be unit tested alongside FailureTable.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <uORB/topics/failure_injection.h>

namespace failure_injection
{

/** Name of a failure_injection_s::FAILURE_UNIT_* value, "unknown" if unmapped. */
inline const char *unitName(uint8_t unit)
{
	// Same names as the `failure` console command keys, so the announcement
	// matches what the operator typed.
	switch (unit) {
	case failure_injection_s::FAILURE_UNIT_SENSOR_GYRO: return "gyro";

	case failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL: return "accel";

	case failure_injection_s::FAILURE_UNIT_SENSOR_MAG: return "mag";

	case failure_injection_s::FAILURE_UNIT_SENSOR_BARO: return "baro";

	case failure_injection_s::FAILURE_UNIT_SENSOR_GPS: return "gps";

	case failure_injection_s::FAILURE_UNIT_SENSOR_OPTICAL_FLOW: return "optical_flow";

	case failure_injection_s::FAILURE_UNIT_SENSOR_VIO: return "vio";

	case failure_injection_s::FAILURE_UNIT_SENSOR_DISTANCE_SENSOR: return "distance_sensor";

	case failure_injection_s::FAILURE_UNIT_SENSOR_AIRSPEED: return "airspeed";

	case failure_injection_s::FAILURE_UNIT_SYSTEM_BATTERY: return "battery";

	case failure_injection_s::FAILURE_UNIT_SYSTEM_MOTOR: return "motor";

	case failure_injection_s::FAILURE_UNIT_SYSTEM_SERVO: return "servo";

	case failure_injection_s::FAILURE_UNIT_SYSTEM_AVOIDANCE: return "avoidance";

	case failure_injection_s::FAILURE_UNIT_SYSTEM_RC_SIGNAL: return "rc_signal";

	case failure_injection_s::FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL: return "mavlink_signal";

	case failure_injection_s::FAILURE_UNIT_SYSTEM_ESC: return "esc";

	default: return "unknown";
	}
}

/** Name of a failure_injection_s::FAILURE_TYPE_* value, "unknown" if unmapped. */
inline const char *typeName(uint8_t type)
{
	switch (type) {
	case failure_injection_s::FAILURE_TYPE_OK: return "ok";

	case failure_injection_s::FAILURE_TYPE_OFF: return "off";

	case failure_injection_s::FAILURE_TYPE_STUCK: return "stuck";

	case failure_injection_s::FAILURE_TYPE_GARBAGE: return "garbage";

	case failure_injection_s::FAILURE_TYPE_WRONG: return "wrong";

	case failure_injection_s::FAILURE_TYPE_SLOW: return "slow";

	case failure_injection_s::FAILURE_TYPE_DELAYED: return "delayed";

	case failure_injection_s::FAILURE_TYPE_INTERMITTENT: return "intermittent";

	default: return "unknown";
	}
}

/**
 * Format an instance mask (bit i = instance i+1) as a phrase:
 * 0xFFFF -> "all instances", any other mask -> "instances 1,2,4",
 * 0 -> "no instances".
 */
inline void instancePhrase(uint16_t mask, char *buf, size_t len)
{
	if (buf == nullptr || len == 0) {
		return;
	}

	if (mask == 0) {
		snprintf(buf, len, "no instances");
		return;
	}

	if (mask == 0xFFFF) {
		snprintf(buf, len, "all instances");
		return;
	}

	size_t offset = snprintf(buf, len, "instances ");
	bool first = true;

	for (int bit = 0; bit < 16 && offset < len; bit++) {
		if (mask & (1u << bit)) {
			offset += snprintf(buf + offset, len - offset, first ? "%d" : ",%d", bit + 1);
			first = false;
		}
	}
}

} // namespace failure_injection
