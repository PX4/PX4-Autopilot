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


namespace manual_control
{

void ManualControlSelector::updateValidityOfChosenInput(uint64_t now)
{
	if (!isInputValid(_setpoint.chosen_input, now)) {
		_setpoint.valid = false;
		_instance = -1;
	}
}

void ManualControlSelector::updateWithNewInputSample(uint64_t now, const manual_control_input_s &input, int instance)
{
	// First check if the chosen input got invalid, so it can get replaced
	updateValidityOfChosenInput(now);

	const bool update_existing_input = _setpoint.valid && input.data_source == _setpoint.chosen_input.data_source;
	const bool start_using_new_input = !_setpoint.valid;

	// Switch to new input if it's valid and we don't already have a valid one
	if (isInputValid(input, now) && (update_existing_input || start_using_new_input)) {
		_setpoint.chosen_input = input;
		_setpoint.valid = true;
		_instance = instance;
	}
}

bool ManualControlSelector::isInputValid(const manual_control_input_s &input, uint64_t now) const
{
	// Check for timeout
	const bool sample_from_the_past = now >= input.timestamp_sample;
	const bool sample_newer_than_timeout = now - input.timestamp_sample < _timeout;

	// Check if source matches the configuration
	const bool source_rc_matched = (_rc_in_mode == 0) && (input.data_source == manual_control_input_s::SOURCE_RC);
	const bool source_mavlink_matched = (_rc_in_mode == 1) &&
					    (input.data_source == manual_control_input_s::SOURCE_MAVLINK_0
					     || input.data_source == manual_control_input_s::SOURCE_MAVLINK_1
					     || input.data_source == manual_control_input_s::SOURCE_MAVLINK_2
					     || input.data_source == manual_control_input_s::SOURCE_MAVLINK_3
					     || input.data_source == manual_control_input_s::SOURCE_MAVLINK_4
					     || input.data_source == manual_control_input_s::SOURCE_MAVLINK_5);
	const bool source_any_matched = (_rc_in_mode == 3);

	return sample_from_the_past && sample_newer_than_timeout
	       && (source_rc_matched || source_mavlink_matched || source_any_matched);
}

manual_control_setpoint_s &ManualControlSelector::setpoint()
{
	return _setpoint;
}

} // namespace manual_control
