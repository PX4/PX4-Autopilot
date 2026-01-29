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

void ManualControlSelector::updateWithNewInputSample(uint64_t now, const manual_control_setpoint_s &input, int instance)
{
	// First check if the chosen input got invalid, so it can get replaced
	updateValidityOfChosenInput(now);

	// Update with input sample if it's valid and should be chosen according to COM_RC_IN_MODE
	if (isInputValid(input, now)) {
		_setpoint = input;
		_setpoint.valid = true;
		_setpoint.timestamp = now; // timestamp_sample is preserved
		_instance = instance;

		if (_first_valid_source == manual_control_setpoint_s::SOURCE_UNKNOWN) {
			// initialize first valid source once
			_first_valid_source = _setpoint.data_source;
		}
	}
}

bool ManualControlSelector::isInputValid(const manual_control_setpoint_s &input, uint64_t now) const
{
	// Check for timeout
	const bool sample_newer_than_timeout = now < input.timestamp_sample + _timeout;

	// Check if source matches the configuration
	bool match = false;

	switch (_rc_in_mode) { // COM_RC_IN_MODE
	case RcInMode::RcOnly:
		match = isRc(input.data_source);
		break;

	case RcInMode::MavLinkOnly:
		match = isMavlink(input.data_source) && ((input.data_source == _setpoint.data_source) || !_setpoint.valid);
		break;

	case RcInMode::RcOrMavlinkWithFallback:
		match = (input.data_source == _setpoint.data_source) || !_setpoint.valid;
		break;

	case RcInMode::RcOrMavlinkKeepFirst:
		match = (input.data_source == _first_valid_source)
			|| (_first_valid_source == manual_control_setpoint_s::SOURCE_UNKNOWN);
		break;

	case RcInMode::PriorityRcThenMavlinkAscending:
		match = !_setpoint.valid || (input.data_source <= _setpoint.data_source);
		break;

	case RcInMode::PriorityMavlinkAscendingThenRc:
		match = !_setpoint.valid
			|| (isRc(input.data_source) && isRc(_setpoint.data_source))
			|| (isMavlink(input.data_source) && (isRc(_setpoint.data_source) || input.data_source <= _setpoint.data_source));
		break;

	case RcInMode::PriorityRcThenMavlinkDescending:
		match = !_setpoint.valid
			|| isRc(input.data_source)
			|| (isMavlink(input.data_source) && isMavlink(_setpoint.data_source) && input.data_source >= _setpoint.data_source);
		break;

	case RcInMode::PriorityMavlinkDescendingThenRc:
		match = !_setpoint.valid || (input.data_source >= _setpoint.data_source);
		break;

	case RcInMode::DisableManualControl:
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
