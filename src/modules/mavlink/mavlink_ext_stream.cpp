/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file mavlink_ext_stream.cpp
 *
 * External MAVLink outbound stream registry implementation.
 */

#include "mavlink_ext_stream.h"

#include <px4_platform_common/atomic.h>
#include <cstring>

struct mavlink_ext_stream_entry_t {
	uint32_t msg_id;
	const char *name;
	mavlink_ext_stream_fn fn;
	void *user_data;
};

static mavlink_ext_stream_entry_t _streams[MAVLINK_EXT_STREAM_MAX] {};
static px4::atomic<unsigned> _stream_count {0};

int mavlink_ext_stream_register(uint32_t msg_id, const char *name,
				mavlink_ext_stream_fn fn, void *user_data)
{
	if (!fn) {
		return -1;
	}

	unsigned count = _stream_count.load();

	// Check for duplicate
	for (unsigned i = 0; i < count; i++) {
		if (_streams[i].msg_id == msg_id) {
			return -1;
		}
	}

	if (count >= MAVLINK_EXT_STREAM_MAX) {
		return -1;
	}

	_streams[count].msg_id = msg_id;
	_streams[count].name = name;
	_streams[count].fn = fn;
	_streams[count].user_data = user_data;
	_stream_count.store(count + 1);

	return 0;
}

int mavlink_ext_stream_unregister(uint32_t msg_id)
{
	unsigned count = _stream_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (_streams[i].msg_id == msg_id) {
			if (i < count - 1) {
				memmove(&_streams[i], &_streams[i + 1],
					(count - i - 1) * sizeof(mavlink_ext_stream_entry_t));
			}

			_stream_count.store(count - 1);
			return 0;
		}
	}

	return -1;
}

void mavlink_ext_stream_dispatch(uint8_t channel)
{
	unsigned count = _stream_count.load();

	for (unsigned i = 0; i < count; i++) {
		_streams[i].fn(channel, _streams[i].user_data);
	}
}
