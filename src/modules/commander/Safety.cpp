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

/**
 * @file Safety.cpp
 */

#include "Safety.hpp"
#include <circuit_breaker/circuit_breaker.h>

using namespace time_literals;

Safety::Safety()
{
	/*
	 * Safety can be turned off with the CBRK_IO_SAFETY parameter.
	 */
	_safety_disabled = circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY);
}

void Safety::safetyButtonHandler()
{
	if (_safety_disabled) {
		_safety.safety_switch_available = true;
		_safety.safety_off = true;

	} else {

		if (!_safety.safety_switch_available && _safety_button_sub.advertised()) {
			_safety.safety_switch_available = true;
		}

		button_event_s button_event;

		while (_safety_button_sub.update(&button_event)) {
			_safety.safety_off |= button_event.triggered; // triggered safety button activates safety off
		}
	}

	// publish immediately on change, otherwise at 1 Hz for logging
	if ((hrt_elapsed_time(&_safety.timestamp) >= 1_s) ||
	    (_safety.safety_off != _previous_safety_off)) {
		_safety.timestamp = hrt_absolute_time();
		_safety_pub.publish(_safety);
	}

	_previous_safety_off = _safety.safety_off;
}

void Safety::enableSafety()
{
	_safety.safety_off = false;
}
