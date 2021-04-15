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

void ManualControlSelector::update_time_only(uint64_t now)
{
	if (_setpoint.timestamp_sample + _timeout < now) {
		_setpoint.valid = false;
		_instance = -1;
	}
}

void ManualControlSelector::update_manual_control_input(uint64_t now, const manual_control_input_s &input, int instance)
{
	// This method requires the current timestamp explicitly in order to prevent weird cases
	// if the timestamp_sample of some source is invalid/wrong.

	// If previous setpoint is timed out, set it invalid, so it can get replaced below.
	update_time_only(now);

	if (_rc_in_mode == 0 && input.data_source == manual_control_input_s::SOURCE_RC) {
		_setpoint = setpoint_from_input(input);
		_setpoint.valid = true;
		_instance = instance;

	} else if (_rc_in_mode == 1 && (input.data_source == manual_control_input_s::SOURCE_MAVLINK_0
					|| input.data_source == manual_control_input_s::SOURCE_MAVLINK_1
					|| input.data_source == manual_control_input_s::SOURCE_MAVLINK_2
					|| input.data_source == manual_control_input_s::SOURCE_MAVLINK_3
					|| input.data_source == manual_control_input_s::SOURCE_MAVLINK_4
					|| input.data_source == manual_control_input_s::SOURCE_MAVLINK_5)) {

		// We only stick to the first discovered mavlink channel.
		if (_setpoint.data_source == input.data_source || !_setpoint.valid) {
			_setpoint = setpoint_from_input(input);
			_setpoint.valid = true;
			_instance = instance;
		}

	} else if (_rc_in_mode == 2) {
		// FIXME: what to do in the legacy case?
	} else if (_rc_in_mode == 3) {

		// We only stick to the first discovered mavlink channel.
		if (_setpoint.data_source == input.data_source || !_setpoint.valid) {
			_setpoint = setpoint_from_input(input);
			_setpoint.valid = true;
			_instance = instance;
		}

	} else {
		// FIXME: param value unknown, what to do?
	}
}

manual_control_setpoint_s ManualControlSelector::setpoint_from_input(const manual_control_input_s &input)
{
	manual_control_setpoint_s setpoint;
	setpoint.timestamp_sample = input.timestamp_sample;
	setpoint.x = input.x;
	setpoint.y = input.y;
	setpoint.z = input.z;
	setpoint.r = input.r;
	// FIXME: what's that?
	//setpoint.vx = (input.x - _manual_control_input[i].x) * dt_inv;
	//setpoint.vy = (input.y - _manual_control_input[i].y) * dt_inv;
	//setpoint.vz = (input.z - _manual_control_input[i].z) * dt_inv;
	//setpoint.vr = (input.r - _manual_control_input[i].r) * dt_inv;
	setpoint.flaps = input.flaps;
	setpoint.aux1 = input.aux1;
	setpoint.aux2 = input.aux2;
	setpoint.aux3 = input.aux3;
	setpoint.aux4 = input.aux4;
	setpoint.aux5 = input.aux5;
	setpoint.aux6 = input.aux6;
	setpoint.data_source = input.data_source;

	return setpoint;
}

manual_control_setpoint_s &ManualControlSelector::setpoint()
{
	return _setpoint;
}

} // namespace manual_control
