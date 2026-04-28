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

#include "ControlSurfacePreflightCheck.hpp"

#include <math.h>

#include <px4_platform_common/log.h>

void ControlSurfacePreflightCheck::handleCommand(hrt_abstime now, bool armed, bool is_tiltrotor)
{
	vehicle_command_s vehicle_command;

	if (!_vehicle_command_sub.update(&vehicle_command)) {
		return;
	}

	if (vehicle_command.command != vehicle_command_s::VEHICLE_CMD_ACTUATOR_GROUP_TEST) {
		return;
	}

	const uint8_t axis = (uint8_t) lroundf(vehicle_command.param1);
	const char *reject_reason = nullptr;
	actuator_armed_s actuator_armed;

	if (armed) {
		reject_reason = "armed";

	} else if (!_armed_sub.copy(&actuator_armed)) {
		reject_reason = "failed to get prearmed state";

	} else if (!actuator_armed.prearmed) {
		reject_reason = "not prearmed";

	} else if (axis == vehicle_command_s::ACTUATOR_TEST_GROUP_COLLECTIVE_TILT && !is_tiltrotor) {
		reject_reason = "tilt commanded but not available";
	}

	uint8_t result;

	if (reject_reason != nullptr) {
		result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
		PX4_WARN("Control surface preflight check rejected (%s)", reject_reason);

	} else {
		if (_running) {
			// Cancel the previous check (still targeted at its original requester).
			sendAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED, now);
		}

		_axis = axis;
		_input = vehicle_command.param2;
		_started = now;
		_running = true;
		result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;
	}

	// Store the command so sendAck() can target the originating system/component.
	_last_command = vehicle_command;
	sendAck(result, now);
}

void ControlSurfacePreflightCheck::updateState(hrt_abstime now, bool armed)
{
	if (!_running) {
		return;
	}

	if (armed) {
		_running = false;
		sendAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_CANCELLED, now);

	} else if (now - _started >= PREFLIGHT_CHECK_DURATION) {
		_running = false;
		sendAck(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED, now);
	}
}

void ControlSurfacePreflightCheck::applyOverrides(matrix::Vector<float, NUM_AXES> (&c)[MAX_NUM_MATRICES],
		bool is_vtol, ActuatorEffectiveness &effectiveness)
{
	// Clear/refresh tilt override every tick so it doesn't outlive the check
	// or any effectiveness-source change.
	const bool tilt_active = _running && _axis == vehicle_command_s::ACTUATOR_TEST_GROUP_COLLECTIVE_TILT;
	effectiveness.overrideCollectiveTilt(tilt_active, tilt_active ? _input : 0.f);

	// ACTUATOR_TEST_GROUP_{ROLL,PITCH,YAW}_TORQUE are 0/1/2 - used directly as torque-vector indices.
	if (!_running || _axis > vehicle_command_s::ACTUATOR_TEST_GROUP_YAW_TORQUE) {
		return;
	}

	// On a VTOL, instance 1 drives the fixed-wing surfaces; otherwise instance 0.
	const int instance = is_vtol ? 1 : 0;

	c[instance](0) = 0.f;
	c[instance](1) = 0.f;
	c[instance](2) = 0.f;
	c[instance](_axis) = _input;
}

void ControlSurfacePreflightCheck::sendAck(uint8_t result, hrt_abstime now)
{
	if (_last_command.from_external) {
		vehicle_command_ack_s command_ack{};
		command_ack.timestamp = now;
		command_ack.command = _last_command.command;
		command_ack.result = result;
		command_ack.target_system = _last_command.source_system;
		command_ack.target_component = _last_command.source_component;
		_command_ack_pub.publish(command_ack);
	}
}
