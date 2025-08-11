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

#pragma once

#include <stdint.h>
#include <uORB/topics/manual_control_setpoint.h>

class ManualControlSelector
{
public:
	void setRcInMode(int32_t rc_in_mode) { _rc_in_mode = rc_in_mode; }
	void setTimeout(uint64_t timeout) { _timeout = timeout; }
	void updateValidityOfChosenInput(uint64_t now);
	void updateWithNewInputSample(uint64_t now, const manual_control_setpoint_s &input, int instance);
	manual_control_setpoint_s &setpoint();
	int instance() const { return _instance; };

private:
	bool isInputValid(const manual_control_setpoint_s &input, uint64_t now) const;
	static bool isRc(uint8_t source);
	static bool isMavlink(uint8_t source);

	manual_control_setpoint_s _setpoint{};
	uint64_t _timeout{0};
	int32_t _rc_in_mode{0};
	int _instance{-1};
	uint8_t _first_valid_source{manual_control_setpoint_s::SOURCE_UNKNOWN};
	uint64_t _timestamp_last_rc{0};
	uint64_t _timestamp_last_mavlink{0};
};
