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

#include "OsdTelemetry.hpp"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/modes/ui.hpp>
#include <matrix/math.hpp>

#include <stdio.h>

using namespace time_literals;

namespace osd
{

void Telemetry::update()
{
	_actuator_armed_sub.update(&_data.actuator_armed);
	_battery_sub.update(&_data.battery);
	_failsafe_flags_sub.update(&_data.failsafe_flags);
	_home_sub.update(&_data.home);
	_input_rc_sub.update(&_data.input_rc);
	_log_message_sub.update(&_data.log_message);
	_manual_control_sub.update(&_data.manual_control);
	_mission_result_sub.update(&_data.mission_result);
	_radio_status_sub.update(&_data.radio_status);
	_global_position_sub.update(&_data.global_position);
	_gps_sub.update(&_data.gps);
	_local_position_sub.update(&_data.local_position);
#if defined(CONFIG_DRIVERS_VTX)
	_vtx_sub.update(&_data.vtx);
#endif

	vehicle_attitude_s attitude{};

	if (_attitude_sub.update(&attitude)) {
		_data.attitude = attitude;
		const matrix::Eulerf euler{matrix::Quatf{attitude.q}};
		_data.roll_rad = euler.phi();
		_data.pitch_rad = euler.theta();
		_data.yaw_rad = matrix::wrap_2pi(euler.psi());
	}

	vehicle_status_s status{};

	if (_status_sub.update(&status)) {
		if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
		    _data.status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			_data.armed_timestamp = hrt_absolute_time();
		}

		_data.status = status;
	}

	const uint64_t now = hrt_absolute_time();
	_data.battery_valid = _data.battery.connected &&
			      _data.battery.timestamp != 0 &&
			      now - _data.battery.timestamp < 2_s &&
			      PX4_ISFINITE(_data.battery.voltage_v);
	_data.attitude_valid = _data.attitude.timestamp != 0 &&
			       now - _data.attitude.timestamp < 1_s &&
			       PX4_ISFINITE(_data.roll_rad) && PX4_ISFINITE(_data.pitch_rad) && PX4_ISFINITE(_data.yaw_rad);
	_data.home_valid = _data.home.valid_hpos &&
			   _data.global_position.timestamp != 0 &&
			   now - _data.global_position.timestamp < 1_s &&
			   PX4_ISFINITE(_data.global_position.lat) && PX4_ISFINITE(_data.global_position.lon) &&
			   PX4_ISFINITE(_data.home.lat) && PX4_ISFINITE(_data.home.lon);

	if (_data.home_valid) {
		_data.home_distance_m = get_distance_to_next_waypoint(
						_data.global_position.lat, _data.global_position.lon, _data.home.lat,
						_data.home.lon);
		_data.home_bearing_rad = get_bearing_to_next_waypoint(
						 _data.global_position.lat, _data.global_position.lon, _data.home.lat,
						 _data.home.lon);
	}
}

void Telemetry::update_message_display(int log_level, MessageDisplay &display)
{
	const uint64_t now = hrt_absolute_time();

	if (_data.log_message.timestamp > _last_log_message_timestamp) {
		_last_log_message_timestamp = _data.log_message.timestamp;

		if (_data.log_message.severity <= log_level) {
			static constexpr const char *severity_names[] {
				"EMERGENCY", "ALERT", "CRITICAL", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG"
			};
			const char *severity = _data.log_message.severity < 8 ? severity_names[_data.log_message.severity] : "STATUS";
			char message[MSG_BUFFER_SIZE] {};
			snprintf(message, sizeof(message), "%s: %s", severity, _data.log_message.text);
			display.set(message);
			_warning_display_timestamp = now;
		}

	} else if (_warning_display_timestamp != 0 && now - _warning_display_timestamp > 30_s) {
		display.set("");
		_warning_display_timestamp = 0;
	}
}

const char *Telemetry::flight_mode() const
{
	return _data.status.nav_state < vehicle_status_s::NAVIGATION_STATE_MAX
	       ? mode_util::nav_state_names[_data.status.nav_state]
	       : "Unknown";
}

float Telemetry::flight_time_s() const
{
	if (_data.status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
	    _data.armed_timestamp == 0) {
		return 0.f;
	}

	return static_cast<float>(hrt_elapsed_time(&_data.armed_timestamp)) * 1e-6f;
}

} // namespace osd
