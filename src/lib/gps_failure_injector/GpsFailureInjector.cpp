/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "GpsFailureInjector.hpp"

#include <drivers/drv_hrt.h>

GpsFailureInjector::GpsFailureInjector(uint8_t owned_instances)
	: _owned_instances(owned_instances)
{
	_param_sys_failure_en = param_find("SYS_FAILURE_EN");
}

void GpsFailureInjector::update()
{
	int32_t sys_failure_en = 0;
	const bool enabled = _param_sys_failure_en != PARAM_INVALID
			     && param_get(_param_sys_failure_en, &sys_failure_en) == PX4_OK
			     && sys_failure_en == 1;

	if (!enabled) {
		// Clear any active failures so disabling the param at runtime
		// stops injection instead of latching the previous state.
		_gps_blocked_mask = 0;
		_gps_stuck_mask = 0;
		_gps_wrong_mask = 0;
		return;
	}

	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		const int failure_unit = static_cast<int>(lroundf(vehicle_command.param1));
		const int failure_type = static_cast<int>(lroundf(vehicle_command.param2));

		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE
		    || failure_unit != vehicle_command_s::FAILURE_UNIT_SENSOR_GPS) {
			continue;
		}

		// param3: 0 = all instances, otherwise 1-based instance index
		const int requested_instance = static_cast<int>(lroundf(vehicle_command.param3));

		if (requested_instance < 0 || requested_instance > GPS_MAX_INSTANCES) {
			vehicle_command_ack_s ack{};
			ack.command = vehicle_command.command;
			ack.from_external = false;
			ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
			ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(ack);
			continue;
		}

		const uint8_t requested_mask = (requested_instance == 0)
					       ? ALL_INSTANCES
					       : static_cast<uint8_t>(1u << (requested_instance - 1));

		// Only react to commands that touch instances we own.
		const uint8_t target_mask = requested_mask & _owned_instances;

		if (target_mask == 0) {
			continue;
		}

		bool supported = true;
		const char *action = nullptr;

		switch (failure_type) {
		case vehicle_command_s::FAILURE_TYPE_OK:
			_gps_blocked_mask &= ~target_mask;
			_gps_stuck_mask   &= ~target_mask;
			_gps_wrong_mask   &= ~target_mask;
			action = "ok";
			break;

		case vehicle_command_s::FAILURE_TYPE_OFF:
			_gps_blocked_mask |= target_mask;
			action = "off";
			break;

		case vehicle_command_s::FAILURE_TYPE_STUCK:
			_gps_stuck_mask |= target_mask;
			action = "stuck";
			break;

		case vehicle_command_s::FAILURE_TYPE_WRONG:
			_gps_wrong_mask |= target_mask;
			action = "wrong";
			break;

		default:
			supported = false;
			break;
		}

		if (action != nullptr) {
			for (int i = 0; i < GPS_MAX_INSTANCES; i++) {
				if (target_mask & (1u << i)) {
					PX4_INFO("CMD_INJECT_FAILURE, GPS %d %s", i + 1, action);
				}
			}
		}

		vehicle_command_ack_s ack{};
		ack.command = vehicle_command.command;
		ack.from_external = false;
		ack.result = supported ?
			     vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
			     vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		ack.timestamp = hrt_absolute_time();
		_command_ack_pub.publish(ack);
	}
}

void GpsFailureInjector::publish(int instance, sensor_gps_s gps, sensor_gps_s &snapshot,
				 uORB::PublicationMulti<sensor_gps_s> &pub)
{
	if (!isBlocked(instance)) {
		if (isStuck(instance)) {
			snapshot.timestamp = hrt_absolute_time();
			pub.publish(snapshot);

		} else {
			if (isWrong(instance)) {
				gps.latitude_deg  += 1.0;
				gps.longitude_deg += 1.0;
			}

			gps.timestamp = hrt_absolute_time();
			snapshot = gps;
			pub.publish(gps);
		}
	}
}
