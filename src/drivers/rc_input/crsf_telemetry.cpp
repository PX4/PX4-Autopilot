/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "crsf_telemetry.h"
#include <lib/rc/crsf.h>

CRSFTelemetry::CRSFTelemetry(int uart_fd) :
	_uart_fd(uart_fd)
{
}

bool CRSFTelemetry::update(const hrt_abstime &now)
{
	const int update_rate_hz = 10;

	if (now - _last_update <= 1_s / (update_rate_hz * num_data_types)) {
		return false;
	}

	bool sent = false;

	switch (_next_type) {
	case 0:
		sent = send_battery();
		break;

	case 1:
		sent = send_gps();
		break;

	case 2:
		sent = send_attitude();
		break;

	case 3:
		sent = send_flight_mode();
		break;

	case 4:
		sent = send_baro_altitude();
		break;
	}

	_last_update = now;
	_next_type = (_next_type + 1) % num_data_types;

	return sent;
}

bool CRSFTelemetry::send_battery()
{
	battery_status_s battery_status;

	if (!_battery_status_sub.update(&battery_status)) {
		return false;
	}

	uint16_t voltage = battery_status.voltage_v * 10;
	uint16_t current = battery_status.current_a * 10;
	int fuel = battery_status.discharged_mah;
	uint8_t remaining = battery_status.remaining * 100;
	return crsf_send_telemetry_battery(_uart_fd, voltage, current, fuel, remaining);
}

bool CRSFTelemetry::send_gps()
{
	sensor_gps_s vehicle_gps_position;

	if (!_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		return false;
	}

	int32_t latitude = static_cast<int32_t>(round(vehicle_gps_position.latitude_deg * 1e7));
	int32_t longitude = static_cast<int32_t>(round(vehicle_gps_position.longitude_deg * 1e7));
	uint16_t groundspeed = vehicle_gps_position.vel_m_s * 3.6f * 10.f;   // 0.1 km/h
	uint16_t gps_heading = math::degrees(matrix::wrap_2pi(vehicle_gps_position.cog_rad)) * 100.f;
	uint16_t altitude = static_cast<uint16_t>(round(vehicle_gps_position.altitude_msl_m) + 1000);   // meters + 1000 offset
	uint8_t num_satellites = vehicle_gps_position.satellites_used;

	return crsf_send_telemetry_gps(_uart_fd, latitude, longitude, groundspeed,
				       gps_heading, altitude, num_satellites);
}

/**
 * pack an altitude for the barometric altitude frame: decimeters + 10000 offset,
 * or meters with the MSB set when above the decimeter range
 */
static uint16_t pack_baro_altitude(const float altitude_m)
{
	int32_t altitude_dm = lroundf(altitude_m * 10.f) + 10000;

	if (altitude_dm < 0) {
		return 0;
	}

	if (altitude_dm < 0x8000) {
		return (uint16_t)altitude_dm;
	}

	int32_t altitude_full_m = lroundf(altitude_m);

	if (altitude_full_m > 0x7FFF) {
		altitude_full_m = 0x7FFF;
	}

	return (uint16_t)altitude_full_m | 0x8000;
}

bool CRSFTelemetry::send_baro_altitude()
{
	// Reuse the CRSF baro-altitude frame to carry the fused local altitude
	// (above the EKF origin), not a raw barometer reading. EdgeTX shows it as "Alt".
	vehicle_local_position_s local_position;

	if (!_vehicle_local_position_sub.update(&local_position) || !local_position.z_valid) {
		return false;
	}

	uint16_t altitude = pack_baro_altitude(-local_position.z);
	int16_t vertical_speed = 0;

	if (local_position.v_z_valid) {
		vertical_speed = lroundf(math::constrain(-local_position.vz * 100.f, (float)INT16_MIN, (float)INT16_MAX));
	}

	return crsf_send_telemetry_baro_altitude(_uart_fd, altitude, vertical_speed);
}

bool CRSFTelemetry::send_attitude()
{
	vehicle_attitude_s vehicle_attitude;

	if (!_vehicle_attitude_sub.update(&vehicle_attitude)) {
		return false;
	}

	matrix::Eulerf attitude = matrix::Quatf(vehicle_attitude.q);
	int16_t pitch = attitude(1) * 1e4f;
	int16_t roll = attitude(0) * 1e4f;
	int16_t yaw = attitude(2) * 1e4f;
	return crsf_send_telemetry_attitude(_uart_fd, pitch, roll, yaw);
}

bool CRSFTelemetry::send_flight_mode()
{
	vehicle_status_s vehicle_status;

	if (!_vehicle_status_sub.update(&vehicle_status)) {
		return false;
	}

	const char *flight_mode = "(unknown)";

	switch (vehicle_status.nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		flight_mode = "Manual";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		flight_mode = "Altitude";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE:
		flight_mode = "Altitude Cruise";
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		flight_mode = "Position";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		flight_mode = "Return";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		flight_mode = "Mission";
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		flight_mode = "Auto";
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		flight_mode = "Acro";
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		flight_mode = "Terminate";
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		flight_mode = "Offboard";
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		flight_mode = "Stabilized";
		break;
	}

	return crsf_send_telemetry_flight_mode(_uart_fd, flight_mode);
}
