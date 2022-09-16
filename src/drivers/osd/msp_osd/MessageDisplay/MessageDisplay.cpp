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

/* Implementation of MessageDisplay class.
 */

#include <math.h>
#include "MessageDisplay.hpp"

namespace msp_osd
{

void MessageDisplay::set(const MessageDisplayType mode, const char *string)
{
	switch (mode) {
	case MessageDisplayType::WARNING:
		if (strcmp(warning_msg, string) != 0) {
			warning_msg[MSG_BUFFER_SIZE - 1] = '\0';
			strncpy(warning_msg, string, MSG_BUFFER_SIZE - 1);
			updated_ = true;
		}

		break;

	case MessageDisplayType::FLIGHT_MODE:
		if (strcmp(flight_mode_msg, string) != 0) {
			flight_mode_msg[MSG_BUFFER_SIZE - 1] = '\0';
			strncpy(flight_mode_msg, string, MSG_BUFFER_SIZE - 1);
			updated_ = true;
		}

		break;

	case MessageDisplayType::ARMING:
		if (strcmp(arming_msg, string) != 0) {
			arming_msg[MSG_BUFFER_SIZE - 1] = '\0';
			strncpy(arming_msg, string, MSG_BUFFER_SIZE - 1);
			updated_ = true;
		}

		break;

	case MessageDisplayType::STATUS:
		if (strcmp(status_msg, string) != 0) {
			status_msg[MSG_BUFFER_SIZE - 1] = '\0';
			strncpy(status_msg, string, MSG_BUFFER_SIZE - 1);
			updated_ = true;
		}

		break;

	case MessageDisplayType::HEADING:
		if (strcmp(heading_msg, string) != 0) {
			heading_msg[MSG_BUFFER_SIZE - 1] = '\0';
			strncpy(heading_msg, string, MSG_BUFFER_SIZE - 1);
			updated_ = true;
		}

		break;

	default:
		// PX4_ERR("Received unsupported message display mode.");
		break;
	}
}

void MessageDisplay::get(char *string, const uint32_t current_time)
{
	// clear input sting
	string[0] = '\0';

	// check if we should update the full message (and reset display)
	if (updated_) {
		// full_message = "Flight Mode: " + flight_mode_msg + " - ARMED: " + arming_msg + " - STATUS: " + status_msg + " - WARNING: " + warning_msg + "            ";

		// reset and construct full message
		strcpy(full_message, "");
		strncat(full_message, flight_mode_msg, 3);	// first three characters of Flight Mode
		strcat(full_message, "|");
		strncat(full_message, arming_msg, 4);		// first four characters of Arming Message
		strcat(full_message, "|");
		strncat(full_message, heading_msg, 2);		// first two characters of Heading

		// add a warning message, if it's not empty
		if (strlen(warning_msg) != 0) {
			// copy as much of warning message as we can fit
			strcat(full_message, "   WARN: ");
			const size_t chars_used {20};
			strncat(full_message, warning_msg, MSG_BUFFER_SIZE - chars_used - FULL_MSG_LENGTH - 1);

			// pad with one full length of terminal whitespace
			for (unsigned i = 0; i != FULL_MSG_LENGTH; ++i) {
				strcat(full_message, " ");
			}
		}

		// reset display variables
		index = 0;
		updated_ = false;
	}

	// handle edge case where full message is short
	if (strlen(full_message) < FULL_MSG_LENGTH) {
		strncpy(string, full_message, FULL_MSG_LENGTH);
		string[FULL_MSG_LENGTH] = '\0';
		return;
	}

	// check if we should update the sub-message (giving extra time to the beginning)
	uint32_t dt = current_time - last_update_;

	if ((index == 0 && dt >= dwell_) || (index != 0 && dt >= period_)) {
		// scroll through message by updating index
		if (++index > strlen(full_message) - FULL_MSG_LENGTH) {
			index = 0;
		}

		// save timestamp
		last_update_ = current_time;
	}

	// reset update flag and return latest message
	strncpy(string, full_message + index, FULL_MSG_LENGTH);
	string[FULL_MSG_LENGTH] = '\0';
}

} // namespace msp_osd
