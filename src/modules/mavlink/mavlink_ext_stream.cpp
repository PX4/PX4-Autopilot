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
 */

#include "mavlink_ext_stream.h"

#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <pthread.h>

struct mavlink_ext_stream_entry_t {
	uint32_t msg_id;
	const char *name;
	mavlink_ext_stream_fn fn;
	void *user_data;
	int interval_us;        // -1 = unlimited, 0 = disabled
	hrt_abstime last_sent;
};

static mavlink_ext_stream_entry_t _streams[MAVLINK_EXT_STREAM_MAX] {};
static px4::atomic<unsigned> _stream_count {0};
static pthread_mutex_t _stream_mutex = PTHREAD_MUTEX_INITIALIZER;

int mavlink_ext_stream_register(uint32_t msg_id, const char *name,
				mavlink_ext_stream_fn fn, void *user_data,
				int interval_us)
{
	if (!fn) {
		return -1;
	}

	pthread_mutex_lock(&_stream_mutex);

	unsigned count = _stream_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (_streams[i].msg_id == msg_id) {
			pthread_mutex_unlock(&_stream_mutex);
			return -1;
		}
	}

	if (count >= MAVLINK_EXT_STREAM_MAX) {
		pthread_mutex_unlock(&_stream_mutex);
		return -1;
	}

	_streams[count].msg_id = msg_id;
	_streams[count].name = name;
	_streams[count].fn = fn;
	_streams[count].user_data = user_data;
	_streams[count].interval_us = interval_us;
	_streams[count].last_sent = 0;
	_stream_count.store(count + 1);

	pthread_mutex_unlock(&_stream_mutex);

	return 0;
}

int mavlink_ext_stream_unregister(uint32_t msg_id)
{
	pthread_mutex_lock(&_stream_mutex);

	unsigned count = _stream_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (_streams[i].msg_id == msg_id) {
			if (i < count - 1) {
				memmove(&_streams[i], &_streams[i + 1],
					(count - i - 1) * sizeof(mavlink_ext_stream_entry_t));
			}

			_stream_count.store(count - 1);
			pthread_mutex_unlock(&_stream_mutex);
			return 0;
		}
	}

	pthread_mutex_unlock(&_stream_mutex);
	return -1;
}

void mavlink_ext_stream_dispatch(uint8_t channel)
{
	unsigned count = _stream_count.load();
	hrt_abstime now = hrt_absolute_time();

	for (unsigned i = 0; i < count; i++) {
		if (_streams[i].interval_us == 0) {
			continue;
		}

		if (_streams[i].interval_us > 0) {
			if (now - _streams[i].last_sent < (hrt_abstime)_streams[i].interval_us) {
				continue;
			}
		}

		if (_streams[i].fn(channel, _streams[i].user_data)) {
			_streams[i].last_sent = now;
		}
	}
}

int mavlink_ext_stream_set_interval(uint32_t msg_id, int interval_us)
{
	unsigned count = _stream_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (_streams[i].msg_id == msg_id) {
			_streams[i].interval_us = interval_us;
			return 0;
		}
	}

	return -1;
}
