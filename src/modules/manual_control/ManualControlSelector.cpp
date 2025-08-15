/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ManualControlSelector.hpp"

void ManualControlSelector::updateValidityOfChosenInput(uint64_t now)
{
	if (!isInputValid(_setpoint, now)) {
		_setpoint.valid = false;
		_instance = -1;
	}
}

void ManualControlSelector::evaluateAndSetBestInput(uint64_t now, const manual_control_setpoint_s inputs[],
		int input_count)
{
	updateValidityOfChosenInput(now);

	// If current input is still valid, keep it
	if (_setpoint.valid) {
		return;
	}

	int best_index = -1;
	uint64_t newest_timestamp = 0;

	for (int i = 0; i < input_count; ++i) {
		const auto &input = inputs[i];

		if (isInputValid(input, now)) {
			if (input.timestamp_sample > newest_timestamp) {
				newest_timestamp = input.timestamp_sample;
				best_index = i;
			}
		}
	}

	if (best_index >= 0) {
		const auto &selected_input = inputs[best_index];

		updateSetpoint(now, selected_input, best_index);

		if (_first_valid_source == manual_control_setpoint_s::SOURCE_UNKNOWN) {
			_first_valid_source = _setpoint.data_source;
		}

		if (isRc(selected_input.data_source)) {
			_timestamp_last_rc = selected_input.timestamp_sample;

		} else if (isMavlink(selected_input.data_source)) {
			_timestamp_last_mavlink = selected_input.timestamp_sample;
		}
	}
}

void ManualControlSelector::processInputSample(uint64_t now, const manual_control_setpoint_s &input, int instance)
{

	if (input.valid) {
		if (isRc(input.data_source)) {
			_timestamp_last_rc = input.timestamp_sample;
		}

		if (isMavlink(input.data_source)) {
			_timestamp_last_mavlink = input.timestamp_sample;
		}
	}

	// if our input and setpoint are valid and from the same source, update the setpoint with newest input
	const bool update_existing_input = _setpoint.valid && (input.data_source == _setpoint.data_source);

	if (isInputValid(input, now) && update_existing_input) {
		updateSetpoint(now, input, instance);
	}
}

void ManualControlSelector::updateSetpoint(uint64_t now, const manual_control_setpoint_s &input, int instance)
{
	_setpoint = input;
	_setpoint.valid = true;
	_setpoint.timestamp = now;
	_instance = instance;
}



bool ManualControlSelector::isInputValid(const manual_control_setpoint_s &input, uint64_t now) const
{
	// Check for timeout
	const bool sample_newer_than_timeout = now < input.timestamp_sample + _timeout;

	// Check if source matches the configuration
	bool match = false;

	switch (_rc_in_mode) { // COM_RC_IN_MODE
	case 0: // RC Transmitter only
		match = isRc(input.data_source);
		break;

	case 1: // Joystick only
		match = isMavlink(input.data_source);
		break;

	case 2: // RC and Joystick with fallback
		match = true;
		break;

	case 3: // RC or Joystick keep first
		match = (input.data_source == _first_valid_source)
			|| (_first_valid_source == manual_control_setpoint_s::SOURCE_UNKNOWN);
		break;

	case 5: // RC priority, Joystick fallback
		match = isRc(input.data_source) || (now > _timestamp_last_rc + _timeout);
		break;

	case 6: // Joystick priority, RC fallback
		match = isMavlink(input.data_source) || (now > _timestamp_last_mavlink + _timeout);
		break;

	case 4: // Stick input disabled
	default:
		break;
	}

	return sample_newer_than_timeout && input.valid && match;
}

manual_control_setpoint_s &ManualControlSelector::setpoint()
{
	return _setpoint;
}

bool ManualControlSelector::isRc(uint8_t source)
{
	return source == manual_control_setpoint_s::SOURCE_RC;
}

bool ManualControlSelector::isMavlink(uint8_t source)
{
	return (source == manual_control_setpoint_s::SOURCE_MAVLINK_0
		|| source == manual_control_setpoint_s::SOURCE_MAVLINK_1
		|| source == manual_control_setpoint_s::SOURCE_MAVLINK_2
		|| source == manual_control_setpoint_s::SOURCE_MAVLINK_3
		|| source == manual_control_setpoint_s::SOURCE_MAVLINK_4
		|| source == manual_control_setpoint_s::SOURCE_MAVLINK_5);
}
