/****************************************************************************
 *
 *   Copyright (c) 2014-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>

#include <uORB/PublicationMulti.hpp>

#include <uORB/topics/esc_status.h>

#include "CANopen.h"
#include "OD.h"
#include "ODRecord.hpp"

using namespace time_literals;

class MotorControllerTelemetry : public ODRecord
{
public:
	MotorControllerTelemetry() : ODRecord(OD_RECORD_TO_UORB)
	{
		OD_extension_init(OD_ENTRY_H5020_motorControllerTelemetry, &_OD_motor_controller_telemetry_extension);
	};

	~MotorControllerTelemetry()
	{
		OD_extension_init(OD_ENTRY_H5020_motorControllerTelemetry, nullptr);
	};

	bool update()
	{
		int32_t rpm;
		int16_t current, act_throttle, motor_temp;
		int8_t controller_temp;
		uint16_t voltage;
		uint8_t deadman_switch;

		if((_esc_status.timestamp - _last_published) > PUBLISH_INTERVAL) {
			_esc_status.esc_count = 0;
			_esc_status.counter++;

			/* Get various data from CAN and publish in uOrb */
			OD_get_i32(OD_ENTRY_H5020_motorControllerTelemetry, SUBIDX_MOTOR_VELOCITY, &rpm, false);
			OD_get_i16(OD_ENTRY_H5020_motorControllerTelemetry, SUBIDX_BATTERY_CURRENT, &current, false);
			OD_get_i16(OD_ENTRY_H5020_motorControllerTelemetry, SUBIDX_ACTUAL_THROTTLE, &act_throttle, false);
			OD_get_i16(OD_ENTRY_H5020_motorControllerTelemetry, SUBIDX_MOTOR_TEMPERATURE, &motor_temp, false);
			OD_get_i8(OD_ENTRY_H5020_motorControllerTelemetry, SUBIDX_TEMPERATURE, &controller_temp, false);
			OD_get_u16(OD_ENTRY_H5020_motorControllerTelemetry, SUBIDX_CAPACITOR_VOLTAGE, &voltage, false);
			OD_get_u8(OD_ENTRY_H5020_motorControllerTelemetry, SUBIDX_DEADMAN_SWITCH, &deadman_switch, false);

			_esc_status.esc_count++;
			_esc_status.esc[0].timestamp = _esc_status.timestamp;
			_esc_status.esc[0].esc_current = (float32_t)current / 16;
			_esc_status.esc[0].esc_rpm = rpm;
			_esc_status.esc[0].esc_throttle = (float32_t)act_throttle / 32767;
			_esc_status.esc[0].esc_address = 0;
			_esc_status.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + _esc_status.esc[0].esc_address;
			_esc_status.esc[0].esc_voltage = (float32_t)voltage / 16;
			_esc_status.esc[0].esc_temperature = (float32_t)controller_temp;
			_esc_status.esc[0].motor_temperature = (float32_t)motor_temp;
			if(deadman_switch > 0) {
				_esc_status.esc[0].esc_state = 1;
			} else {
				_esc_status.esc[0].esc_state = 0;
			}

			_esc_status_pub.publish(_esc_status);

			_last_published = _esc_status.timestamp;
			return true;
		} else {
			return false;
		}
	};

private:
	enum motor_controller_telemetry_subidx {
		SUBIDX_ACTUAL_THROTTLE = 1,
		SUBIDX_MOTOR_VELOCITY,
		SUBIDX_MOTOR_TEMPERATURE,
		SUBIDX_TEMPERATURE,
		SUBIDX_CAPACITOR_VOLTAGE,
		SUBIDX_BATTERY_CURRENT,
		SUBIDX_DEADMAN_SWITCH,
	};

	OD_extension_t _OD_motor_controller_telemetry_extension = {
		.object = (void *)&_esc_status.timestamp,
		.read = od_read_record,
		.write = od_write_record
	};

	esc_status_s 							_esc_status{};
	uORB::PublicationMulti<esc_status_s> 	_esc_status_pub{ORB_ID(esc_status)};
};
