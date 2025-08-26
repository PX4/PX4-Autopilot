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

#include <uORB/topics/actuator_motors.h>

void FailureInjector::update()
{
	vehicle_command_s vehicle_command;

	while (_vehicle_command_sub.update(&vehicle_command)) {
		if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_INJECT_FAILURE) {
			continue;
		}

		bool handled = false;
		bool supported = false;

		const int failure_unit = static_cast<int>(vehicle_command.param1 + 0.5f);
		const int failure_type = static_cast<int>(vehicle_command.param2 + 0.5f);
		const int instance = static_cast<int>(vehicle_command.param3 + 0.5f);

		if (failure_unit == vehicle_command_s::FAILURE_UNIT_SYSTEM_MOTOR) {
			handled = true;

			if (failure_type == vehicle_command_s::FAILURE_TYPE_OK) {
				PX4_INFO("CMD_INJECT_FAILURE, motors ok");
				supported = false;

				// 0 to signal all
				if (instance == 0) {
					supported = true;

					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d ok", i);
						_esc_blocked &= ~(1 << i);
						_esc_wrong &= ~(1 << i);
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					supported = true;

					PX4_INFO("CMD_INJECT_FAILURE, motor %d ok", instance - 1);
					_esc_blocked &= ~(1 << (instance - 1));
					_esc_wrong &= ~(1 << (instance - 1));
				}
			}

			else if (failure_type == vehicle_command_s::FAILURE_TYPE_OFF) {
				PX4_WARN("CMD_INJECT_FAILURE, motors off");
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d off", i);
						_esc_blocked |= 1 << i;
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, motor %d off", instance - 1);
					_esc_blocked |= 1 << (instance - 1);
				}
			}

			else if (failure_type == vehicle_command_s::FAILURE_TYPE_WRONG) {
				PX4_INFO("CMD_INJECT_FAILURE, motors wrong");
				supported = true;

				// 0 to signal all
				if (instance == 0) {
					for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
						PX4_INFO("CMD_INJECT_FAILURE, motor %d wrong", i);
						_esc_wrong |= 1 << i;
					}

				} else if (instance >= 1 && instance <= esc_status_s::CONNECTED_ESC_MAX) {
					PX4_INFO("CMD_INJECT_FAILURE, motor %d wrong", instance - 1);
					_esc_wrong |= 1 << (instance - 1);
				}
			}
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
	}

}

void FailureInjector::manipulateEscStatus(esc_status_s &status)
{
	if (_esc_blocked != 0 || _esc_wrong != 0) {
		unsigned offline = 0;

		for (int i = 0; i < status.esc_count; i++) {
			const unsigned i_esc = status.esc[i].actuator_function - actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;

			if (_esc_blocked & (1 << i_esc)) {
				unsigned function = status.esc[i].actuator_function;
				memset(&status.esc[i], 0, sizeof(status.esc[i]));
				status.esc[i].actuator_function = function;
				offline |= 1 << i;

			} else if (_esc_wrong & (1 << i_esc)) {
				// Create wrong rerport for this motor by scaling key values up and down
				status.esc[i].esc_voltage *= 0.1f;
				status.esc[i].esc_current *= 0.1f;
				status.esc[i].esc_rpm *= 10.0f;
			}
		}

		status.esc_online_flags &= ~offline;
	}
}
