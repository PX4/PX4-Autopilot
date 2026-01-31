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

#include "FailureInjector.hpp"

#include <parameters/param.h>
#include <uORB/topics/actuator_motors.h>

FailureInjector::FailureInjector()
{
	int32_t param_sys_failure_en = 0;

	if ((param_get(param_find("SYS_FAILURE_EN"), &param_sys_failure_en) == PX4_OK)
	    && (param_sys_failure_en == 1)) {
		_failure_injection_enabled = true;
	}
}

void FailureInjector::update()
{
	if (!_failure_injection_enabled) { return; }

	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);

		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE
		    || failure_unit != vehicle_command_s::FAILURE_UNIT_SYSTEM_MOTOR) {
			continue;
		}

		const int instance = static_cast<int>(vehicle_command.param3 + 0.5f);
		bool supported = false;

		for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
			if (instance != 0 && i != (instance - 1)) {
				continue;
			}

			switch (failure_type) {
			case vehicle_command_s::FAILURE_TYPE_OK:
				supported = true;
				PX4_INFO("CMD_INJECT_FAILURE, motor %d ok", i + 1);
				_motor_stop_mask &= ~(1 << i);
				_esc_telemetry_blocked_mask &= ~(1 << i);
				_esc_telemetry_wrong_mask &= ~(1 << i);
				break;

			case vehicle_command_s::FAILURE_TYPE_OFF:
				supported = true;
				PX4_INFO("CMD_INJECT_FAILURE, motor %d off", i + 1);
				_motor_stop_mask |= 1 << i;
				break;

			case vehicle_command_s::FAILURE_TYPE_STUCK:
				supported = true;
				PX4_INFO("CMD_INJECT_FAILURE, motor %d no esc telemetry", i + 1);
				_esc_telemetry_blocked_mask |= 1 << i;
				break;

			case vehicle_command_s::FAILURE_TYPE_WRONG:
				supported = true;
				PX4_INFO("CMD_INJECT_FAILURE, motor %d esc telemetry wrong", i);
				_esc_telemetry_wrong_mask |= 1 << i;
				break;
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

void FailureInjector::manipulateEscStatus(esc_status_s &status)
{
	if (!_failure_injection_enabled) { return; }

	if (_esc_telemetry_blocked_mask != 0 || _esc_telemetry_wrong_mask != 0) {
		for (int i = 0; i < status.esc_count; i++) {
			const unsigned i_esc = status.esc[i].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

			if (_esc_telemetry_blocked_mask & (1 << i_esc)) {
				unsigned function = status.esc[i].actuator_function;
				memset(&status.esc[i], 0, sizeof(status.esc[i]));
				status.esc[i].actuator_function = function;
				status.esc_online_flags &= ~(1 << i);

			} else if (_esc_telemetry_wrong_mask & (1 << i_esc)) {
				// Create wrong rerport for this motor by scaling key values up and down
				status.esc[i].esc_voltage *= 0.1f;
				status.esc[i].esc_current *= 0.1f;
				status.esc[i].esc_rpm *= 10.0f;
			}
		}
	}
}
