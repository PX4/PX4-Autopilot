/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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
 * @file failure_detector_HITL.cpp
 *
 * @author Ilia Loginov <ilia.loginov@tii.ae>
 */

#include "failure_detector_HITL.hpp"

#include <px4_platform_common/log.h>

FailureDetectorHITL::FailureDetectorHITL(bool hil_enabled)
{

	if (!hil_enabled) {
		_vehicle_command_sub.unsubscribe();

	} else {
		PX4_INFO("RUN FailureDetectorHITL");
	}
}
bool FailureDetectorHITL::update()
{
	vehicle_command_s vehicle_command{};
	bool update = false;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);

		PX4_INFO("Failure detector caught new injection");

		switch (failure_unit) {
		case vehicle_command_s::FAILURE_UNIT_SENSOR_GPS:
#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, gps ok");
				_gps = FailureStatus::ok;
				supported = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, gps off");
				supported = true;
				_gps = FailureStatus::off;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, gps stuck");
				_gps = FailureStatus::stuck;
				supported = true;
			}

#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION
			break;

		case vehicle_command_s::FAILURE_UNIT_SENSOR_BARO:
#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, baro ok");
				_baro = FailureStatus::ok;
				supported = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, baro off");
				_baro = FailureStatus::off;
				supported = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_STUCK) {
				PX4_WARN("CMD_INJECT_FAILURE, baro stuck");
				_baro = FailureStatus::stuck;
				supported = true;
			}

#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA
			break;

		case vehicle_command_s::FAILURE_UNIT_SENSOR_MAG:
#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, mag ok");
				_mag = FailureStatus::ok;
				supported = true;

			} else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, mag off");
				_mag = FailureStatus::off;
				supported = true;
			}

#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER
			break;

		default:
			break;
		}

		if (handled) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = supported ?
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
				     vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
		}

		update |= supported;
	}

	return update;
}
#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
bool FailureDetectorHITL::isGpsOk() const
{
	return FailureStatus::ok == _gps;
}

bool FailureDetectorHITL::isGpsOff() const
{
	return FailureStatus::off == _gps;
}

bool FailureDetectorHITL::isGpsStuck() const
{
	return FailureStatus::stuck == _gps;
}
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
bool FailureDetectorHITL::isBaroOk() const
{
	return FailureStatus::ok == _baro;
}

bool FailureDetectorHITL::isBaroOff() const
{
	return FailureStatus::off == _baro;
}

bool FailureDetectorHITL::isBaroStuck() const
{
	return FailureStatus::stuck == _baro;
}
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_MAGNETOMETER)
bool FailureDetectorHITL::isMagOk() const
{
	return FailureStatus::ok == _mag;
}

bool FailureDetectorHITL::isMagOff() const
{
	return FailureStatus::off == _mag;
}
#endif // CONFIG_SENSORS_VEHICLE_MAGNETOMETER


FakeSensors::FakeSensors(bool hil_enabled)
{
	if (hil_enabled) {
#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
		_fake_baro_publisher.start();
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA

#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
		_fake_gps_publisher.start();
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION
	}
}

void FakeSensors::update(const FailureDetectorHITL &detector)
{
#if defined(CONFIG_SENSORS_VEHICLE_GPS_POSITION)
	_fake_gps_publisher.setEnabled(detector.isGpsStuck());
#endif // CONFIG_SENSORS_VEHICLE_GPS_POSITION

#if defined(CONFIG_SENSORS_VEHICLE_AIR_DATA)
	_fake_baro_publisher.setEnabled(detector.isBaroStuck());
#endif // CONFIG_SENSORS_VEHICLE_AIR_DATA
}

