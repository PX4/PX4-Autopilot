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

#include <cstring>
#include <uORB/topics/actuator_motors.h>

void FailureInjector::update()
{
	failure_injection_s failure_injection;

	if (_failure_injection_sub.update(&failure_injection)) {
		_failure_config.set(failure_injection);
		rebuildMasks();
	}
}

void FailureInjector::rebuildMasks()
{
	_motor_stop_mask = 0;
	_esc_telemetry_blocked_mask = 0;
	_esc_telemetry_wrong_mask = 0;

	for (int i = 0; i < esc_status_s::CONNECTED_ESC_MAX; i++) {
		switch (_failure_config.mode(failure_injection_s::FAILURE_UNIT_SYSTEM_MOTOR, i + 1)) {
		case failure_injection::Mode::Off:
			_motor_stop_mask |= 1u << i;
			break;

		case failure_injection::Mode::Stuck:
			_esc_telemetry_blocked_mask |= 1u << i;
			break;

		case failure_injection::Mode::Wrong:
			_esc_telemetry_wrong_mask |= 1u << i;
			break;

		default:
			break;
		}
	}
}

void FailureInjector::manipulateEscStatus(esc_status_s &status)
{
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
