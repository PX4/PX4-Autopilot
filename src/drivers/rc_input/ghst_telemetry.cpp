/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ghst_telemetry.cpp
 *
 * IRC Ghost (Immersion RC Ghost) telemetry.
 *
 * @author Igor Misic <igy1000mb@gmail.com>
 * @author Juraj Ciberlin <jciberlin1@gmail.com>
 */

#include "ghst_telemetry.hpp"
#include <lib/rc/ghst.hpp>

using time_literals::operator ""_s;

GHSTTelemetry::GHSTTelemetry(int uart_fd) :
	_uart_fd(uart_fd)
{
}

bool GHSTTelemetry::update(const hrt_abstime &now)
{
	bool success = false;

	if ((now - _last_update) > (1_s / (UPDATE_RATE_HZ * NUM_DATA_TYPES))) {

		switch (_next_type) {
		case 0U:
			success = send_battery_status();
			break;

		case 1U:
			success = send_gps1_status();
			break;

		case 2U:
			success = send_gps2_status();
			break;

		default:
			success = false;
			break;
		}

		_last_update = now;
		_next_type = (_next_type + 1U) % NUM_DATA_TYPES;
	}

	return success;
}

bool GHSTTelemetry::send_battery_status()
{
	bool success = false;
	float voltage_in_10mV;
	float current_in_10mA;
	float fuel_in_10mAh;
	battery_status_s battery_status;

	if (_battery_status_sub.update(&battery_status)) {
		voltage_in_10mV = battery_status.voltage_filtered_v * FACTOR_VOLTS_TO_10MV;
		current_in_10mA = battery_status.current_filtered_a * FACTOR_AMPS_TO_10MA;
		fuel_in_10mAh = battery_status.discharged_mah * FACTOR_MAH_TO_10MAH;
		success = ghst_send_telemetry_battery_status(_uart_fd,
				static_cast<uint16_t>(voltage_in_10mV),
				static_cast<uint16_t>(current_in_10mA),
				static_cast<uint16_t>(fuel_in_10mAh));
	}

	return success;
}

bool GHSTTelemetry::send_gps1_status()
{
	sensor_gps_s vehicle_gps_position;

	if (!_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		return false;
	}

	int32_t latitude = static_cast<int32_t>(round(vehicle_gps_position.latitude_deg * 1e7));        // 1e-7 degrees
	int32_t longitude = static_cast<int32_t>(round(vehicle_gps_position.longitude_deg * 1e7));      // 1e-7 degrees
	uint16_t altitude = static_cast<int16_t>(round(vehicle_gps_position.altitude_msl_m));           // meters

	return ghst_send_telemetry_gps1_status(_uart_fd, latitude, longitude, altitude);
}

bool GHSTTelemetry::send_gps2_status()
{
	sensor_gps_s vehicle_gps_position;

	if (!_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		return false;
	}

	uint16_t ground_speed = (uint16_t)(vehicle_gps_position.vel_d_m_s / 3.6f * 10.f);
	uint16_t ground_course = (uint16_t)(math::degrees(vehicle_gps_position.cog_rad) * 100.f);
	uint8_t num_sats = vehicle_gps_position.satellites_used;

	// TBD: Can these be computed in a RC telemetry driver?
	uint16_t home_dist = 0;
	uint16_t home_dir = 0;
	uint8_t flags = 0;

	return ghst_send_telemetry_gps2_status(_uart_fd, ground_speed, ground_course, num_sats, home_dist, home_dir, flags);
}

