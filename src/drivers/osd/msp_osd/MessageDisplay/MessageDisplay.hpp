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

/* Class for displaying messages to a limited screen.
 */

#pragma once

#include <string.h>
#include <stdint.h>

namespace msp_osd
{

// character size limitations
#define MSG_BUFFER_SIZE 250

// size of available characters, accounting for null terminator
//  note: the craft_name seems to think it has 15 chars. From testing
//        that seems incorrect
#define FULL_MSG_LENGTH 12
#define FULL_MSG_BUFFER 13

// supported message types
enum MessageDisplayType {
	WARNING,
	FLIGHT_MODE,
	ARMING,
	STATUS,
	HEADING
};

// display information
class MessageDisplay
{
	// working information
	char warning_msg[MSG_BUFFER_SIZE] {""};
	char flight_mode_msg[MSG_BUFFER_SIZE] {"???"};
	char arming_msg[MSG_BUFFER_SIZE] {"????"};
	char heading_msg[MSG_BUFFER_SIZE] {"??"};
	// currently unused:
	char status_msg[MSG_BUFFER_SIZE] {""};

	// the full message and the part we're currently displaying
	char full_message[MSG_BUFFER_SIZE] {"INITIALIZING"};

	// current index we're displaying
	uint16_t index{0};

	// last update timestamp
	uint64_t last_update_{0};
	bool updated_{false};

	// dwell duration update period (us)
	uint64_t period_{125'000};
	uint64_t dwell_{500'000};

public:
	MessageDisplay() = default;
	MessageDisplay(const uint64_t period, const uint64_t dwell) : period_(period), dwell_(dwell) {}

	// set the given string
	void set(const MessageDisplayType mode, const char *string);

	// get the latest subsection of the message
	void get(char *string, const uint32_t current_time);

	// update local parameters
	void set_period(const uint64_t period) { period_ = period; }
	void set_dwell(const uint64_t dwell) { dwell_ = dwell; }
};


} // namespace msp_osd
