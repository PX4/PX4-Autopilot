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

#include "MessageDisplay.hpp"

namespace osd
{

void MessageDisplay::set(const char *string)
{
	if (strcmp(message_, string) != 0) {
		strncpy(message_, string, sizeof(message_) - 1);
		message_[sizeof(message_) - 1] = '\0';
		index = 0;
		updated_ = true;
	}
}

void MessageDisplay::get(char *string, uint64_t current_time)
{
	// clear input sting
	string[0] = '\0';

	// check if we should reset the display position
	if (updated_) {
		index = 0;
		last_update_ = current_time;
		updated_ = false;
	}

	// handle edge case where the message is short
	if (strlen(message_) < FULL_MSG_LENGTH) {
		strncpy(string, message_, FULL_MSG_LENGTH);
		string[FULL_MSG_LENGTH] = '\0';
		return;
	}

	// check if we should update the sub-message (giving extra time to the
	// beginning)
	const uint64_t dt = current_time - last_update_;

	if ((index == 0 && dt >= dwell_) || (index != 0 && dt >= period_)) {
		// scroll through message by updating index
		if (++index > strlen(message_) - FULL_MSG_LENGTH) {
			index = 0;
		}

		// save timestamp
		last_update_ = current_time;
	}

	// reset update flag and return latest message
	strncpy(string, message_ + index, FULL_MSG_LENGTH);
	string[FULL_MSG_LENGTH] = '\0';
}

} // namespace osd
