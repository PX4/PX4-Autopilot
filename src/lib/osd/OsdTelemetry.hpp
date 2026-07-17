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

#pragma once

#include "MessageDisplay.hpp"

#include <px4_platform_common/px4_config.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/sensor_gps.h>
#if defined(CONFIG_DRIVERS_VTX)
#include <uORB/topics/vtx.h>
#endif
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

namespace osd
{

enum class Symbol : uint8_t {
	CraftName = 0,
	SystemId = 0,
	Disarmed = 1,
	GpsLatitude = 2,
	GpsLongitude = 3,
	GpsSatellites = 4,
	GpsSpeed = 5,
	HomeDistance = 6,
	HomeDirection = 7,
	MissionState = 7,
	MainBatteryVoltage = 8,
	CurrentDraw = 9,
	MahDrawn = 10,
	Rssi = 11,
	Altitude = 12,
	NumericalVario = 13,
	FlightMode = 14,
	LinkQuality = 15,
	PitchAngle = 16,
	RollAngle = 17,
	Crosshairs = 18,
	AverageCellVoltage = 19,
	HorizonSidebars = 20,
	MavState = 20,
	Power = 21,
	FlightTime = 22,
	StatusMessage = 23,
	ArtificialHorizon = 24,
	Heading = 25,
	VtxInfo = 26,
	VtxFrequency = 27,
	VtxPower = 28,
	Throttle = 29,
	GpsInfo = 30,
};

struct TelemetryData {
	actuator_armed_s actuator_armed{};
	battery_status_s battery{};
	failsafe_flags_s failsafe_flags{};
	home_position_s home{};
	input_rc_s input_rc{};
	log_message_s log_message{};
	manual_control_setpoint_s manual_control{};
	mission_result_s mission_result{};
	radio_status_s radio_status{};
	sensor_gps_s gps{};
#if defined(CONFIG_DRIVERS_VTX)
	vtx_s vtx {};
#endif
	vehicle_attitude_s attitude {};
	vehicle_global_position_s global_position{};
	vehicle_local_position_s local_position{};
	vehicle_status_s status{};
	uint64_t armed_timestamp{0};
	float roll_rad{0.f};
	float pitch_rad{0.f};
	float yaw_rad{0.f};
	float home_distance_m{0.f};
	float home_bearing_rad{0.f};
	bool battery_valid{false};
	bool attitude_valid{false};
	bool home_valid{false};
};

class Telemetry
{
public:
	void update();
	void update_message_display(int log_level, MessageDisplay &display);

	const TelemetryData &data() const { return _data; }
	const char *flight_mode() const;
	float flight_time_s() const;

private:
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _log_message_sub{ORB_ID(log_message)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _radio_status_sub{ORB_ID(radio_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
#if defined(CONFIG_DRIVERS_VTX)
	uORB::Subscription _vtx_sub {ORB_ID(vtx)};
#endif

	TelemetryData _data{};
	uint64_t _last_log_message_timestamp{0};
	uint64_t _warning_display_timestamp{0};
};

} // namespace osd
