/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "gripper.h"

void Gripper::init(const GripperConfig &config)
{
	switch (config.type) {
	case GripperConfig::GripperType::SERVO:
		break;

	case GripperConfig::GripperType::UNDEFINED:

	// FALL THROUGH
	default:
		_valid = false;
		return;
	}

	switch (config.sensor) {
	case GripperConfig::GripperSensorType::ENCODER:
		// Handle encoder sensor setup
		_has_feedback_sensor = true;
		break;

	case GripperConfig::GripperSensorType::NONE:
	default:
		// No feedback sensor
		_has_feedback_sensor = false;
		break;
	}

	_timeout_us = config.timeout_us;

	// Reset internal states
	_state = GripperState::IDLE;
	_last_command_time = 0;
	_released_state_cache = false;
	_grabbed_state_cache = false;

	// We have valid gripper type & sensor configuration
	_valid = true;
}

void Gripper::deinit()
{
	// Reset Config variables
	_has_feedback_sensor = false;
	_timeout_us = 0;

	// Reset internal states
	_state = GripperState::IDLE;
	_last_command_time = 0;
	_released_state_cache = false;
	_grabbed_state_cache = false;

	// Mark gripper instance as invalid
	_valid = false;
}

void Gripper::grab()
{
	if (_state == GripperState::GRABBING || _state == GripperState::GRABBED) {
		return;
	}

	publish_gripper_command(gripper_s::COMMAND_GRAB);
	_state = GripperState::GRABBING;
	_last_command_time = hrt_absolute_time();
}

void Gripper::release()
{
	if (_state == GripperState::RELEASING || _state == GripperState::RELEASED) {
		return;
	}

	publish_gripper_command(gripper_s::COMMAND_RELEASE);
	_state = GripperState::RELEASING;
	_last_command_time = hrt_absolute_time();
}

void Gripper::update()
{
	const bool command_timed_out = (hrt_elapsed_time(&_last_command_time) >= _timeout_us);

	// Handle transition from intermediate state to definite state
	switch (_state) {
	case GripperState::GRABBING:
		if (_has_feedback_sensor) {
			// Handle feedback sensor input, return true for now (not supported)
			_grabbed_state_cache = true;
			_state = GripperState::GRABBED;
			break;
		}

		if (command_timed_out) {
			_grabbed_state_cache = true;
			_state = GripperState::GRABBED;
		}

		break;

	case GripperState::RELEASING:
		if (_has_feedback_sensor) {
			// Handle feedback sensor input, return true for now (not supported)
			_released_state_cache = true;
			_state = GripperState::RELEASED;
			break;
		}

		if (command_timed_out) {
			_released_state_cache = true;
			_state = GripperState::RELEASED;
		}

		break;

	default:
		// DO NOTHING
		break;
	}
}

void Gripper::publish_gripper_command(const int8_t gripper_command)
{
	gripper_s gripper{};
	gripper.timestamp = hrt_absolute_time();
	gripper.command = gripper_command;
	_gripper_pub.publish(gripper);
}

const char *Gripper::get_state_str() const
{
	switch (_state) {
	case GripperState::GRABBING:
		return "GRABBING";

	case GripperState::GRABBED:
		return "GRABBED";

	case GripperState::RELEASING:
		return "RELEASING";

	case GripperState::RELEASED:
		return "RELEASED";

	case GripperState::IDLE:
		return "IDLE";

	default:
		return "UNKNOWN";
	}
}
