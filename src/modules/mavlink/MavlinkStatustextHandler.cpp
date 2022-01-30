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

#include "MavlinkStatustextHandler.hpp"
#include <mathlib/mathlib.h>

bool MavlinkStatustextHandler::should_publish_previous(const mavlink_statustext_t &msg_statustext)
{
	// Check if the previous message has not been published yet. This can
	// happen if the last chunk has been dropped, let's publish what we have.
	if (_last_log_id > 0 && msg_statustext.id != _last_log_id) {
		// We add a note that a part is missing at the end.
		const size_t offset = strlen(_log_msg.text);
		strncpy(_log_msg.text + offset, "[ missing ... ]",
			sizeof(_log_msg.text) - offset);
		_log_msg.text[sizeof(_log_msg.text) - 1] = '\0';

		return true;
	}

	return false;
}

bool MavlinkStatustextHandler::should_publish_current(const mavlink_statustext_t &msg_statustext, const uint64_t &now)
{
	if (msg_statustext.id > 0) {
		// Multi statustext message.
		if (_last_log_id != msg_statustext.id) {
			// On the first one arriving, we save the timestamp and severity.
			_log_msg.timestamp = now;
			_log_msg.severity = msg_statustext.severity;
		}

		if (msg_statustext.chunk_seq == 0) {
			// We start from 0 with a new message.
			strncpy(_log_msg.text, msg_statustext.text,
				math::min(sizeof(_log_msg.text), sizeof(msg_statustext.text)));
			_log_msg.text[sizeof(msg_statustext.text)] = '\0';

		} else {
			if (msg_statustext.chunk_seq != _last_log_chunk_seq + 1) {
				const size_t offset = strlen(_log_msg.text);
				strncpy(_log_msg.text + offset, "[ missing ... ]",
					sizeof(_log_msg.text) - offset);
				_log_msg.text[sizeof(_log_msg.text) - offset - 1] = '\0';
			}

			// We add a consecutive chunk.
			const size_t offset = strlen(_log_msg.text);
			const size_t max_to_add = math::min(sizeof(_log_msg.text) - offset - 1, sizeof(msg_statustext.text));
			strncpy(_log_msg.text + offset, msg_statustext.text, max_to_add);
			_log_msg.text[math::min(offset + max_to_add, sizeof(_log_msg.text) - 1)] = '\0';
		}

		_last_log_chunk_seq = msg_statustext.chunk_seq;

		const bool publication_message_is_full = sizeof(_log_msg.text) - 1 == strlen(_log_msg.text);
		const bool found_zero_termination = strnlen(msg_statustext.text,
						    sizeof(msg_statustext.text)) < sizeof(msg_statustext.text);

		if (publication_message_is_full || found_zero_termination) {
			_last_log_id = 0;
			_last_log_chunk_seq = -1;
			return true;

		} else {
			_last_log_id = msg_statustext.id;
			return false;
		}

	} else {
		// Single statustext message.
		_log_msg.timestamp = now;
		_log_msg.severity = msg_statustext.severity;

		static_assert(sizeof(_log_msg.text) > sizeof(msg_statustext.text),
			      "_log_msg.text not big enough to hold msg_statustext.text");

		strncpy(_log_msg.text, msg_statustext.text,
			math::min(sizeof(_log_msg.text),
				  sizeof(msg_statustext.text)));

		// We need to 0-terminate after the copied text which does not have to be
		// 0-terminated on the wire.
		_log_msg.text[sizeof(msg_statustext.text) - 1] = '\0';

		_last_log_id = 0;
		return true;

	}
}
