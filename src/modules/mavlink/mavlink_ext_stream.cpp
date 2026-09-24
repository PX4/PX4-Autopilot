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
 * mavlink_ext_send() lives in mavlink_main.cpp next to the instance table.
 */

#include "mavlink_bridge_header.h"	// MAVLINK_COMM_NUM_BUFFERS
#include "mavlink_ext_stream.h"

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/atomic.h>
#include <pthread.h>

struct mavlink_ext_stream_entry_t {
	uint32_t msg_id;
	mavlink_ext_send_fn fn;
	void *user_data;
	int32_t default_interval_us;
	// SET_MESSAGE_INTERVAL acts on one link, so rate state is kept per channel.
	int32_t interval_us[MAVLINK_COMM_NUM_BUFFERS];
	hrt_abstime last_sent[MAVLINK_COMM_NUM_BUFFERS];
};

// Locking mirrors mavlink_ext_handler.cpp: one mutex over registration,
// dispatch and unregistration, held across the callback so unregister()
// returns only once no callback is running. The count is atomic only so
// dispatch can skip the lock while nothing is registered.
static mavlink_ext_stream_entry_t stream_table[MAVLINK_EXT_STREAM_MAX] {};
static px4::atomic<unsigned> stream_count {0};
static pthread_mutex_t stream_mutex = PTHREAD_MUTEX_INITIALIZER;

static int stream_find(uint32_t msg_id)
{
	const unsigned count = stream_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (stream_table[i].msg_id == msg_id) {
			return (int)i;
		}
	}

	return -1;
}

int mavlink_ext_stream_register(uint32_t msg_id, mavlink_ext_send_fn fn, void *user_data, int32_t interval_us)
{
	if (fn == nullptr || interval_us < MAVLINK_EXT_STREAM_UNLIMITED) {
		return -1;
	}

	LockGuard lg{stream_mutex};
	const unsigned count = stream_count.load();

	if (count >= MAVLINK_EXT_STREAM_MAX || stream_find(msg_id) >= 0) {
		return -1;
	}

	mavlink_ext_stream_entry_t &entry = stream_table[count];
	entry = {};
	entry.msg_id = msg_id;
	entry.fn = fn;
	entry.user_data = user_data;
	entry.default_interval_us = interval_us;

	for (int32_t &channel_interval : entry.interval_us) {
		channel_interval = interval_us;
	}

	stream_count.store(count + 1);
	return 0;
}

int mavlink_ext_stream_unregister(uint32_t msg_id)
{
	LockGuard lg{stream_mutex};
	const int i = stream_find(msg_id);

	if (i < 0) {
		return -1;
	}

	// Table order is irrelevant to dispatch: fill the hole with the last entry.
	const unsigned last = stream_count.load() - 1;
	stream_table[i] = stream_table[last];
	stream_table[last] = {};
	stream_count.store(last);
	return 0;
}

int mavlink_ext_stream_set_interval(uint8_t channel, uint32_t msg_id, int32_t interval_us)
{
	if (channel >= MAVLINK_COMM_NUM_BUFFERS || interval_us < MAVLINK_EXT_STREAM_DEFAULT) {
		return -1;
	}

	LockGuard lg{stream_mutex};
	const int i = stream_find(msg_id);

	if (i < 0) {
		return -1;
	}

	mavlink_ext_stream_entry_t &entry = stream_table[i];
	entry.interval_us[channel] = (interval_us == MAVLINK_EXT_STREAM_DEFAULT) ? entry.default_interval_us : interval_us;
	entry.last_sent[channel] = 0;
	return 0;
}

void mavlink_ext_stream_dispatch(uint8_t channel)
{
	if (channel >= MAVLINK_COMM_NUM_BUFFERS || stream_count.load() == 0) {
		return;
	}

	LockGuard lg{stream_mutex};
	const hrt_abstime now = hrt_absolute_time();
	const unsigned count = stream_count.load();

	for (unsigned i = 0; i < count; i++) {
		mavlink_ext_stream_entry_t &entry = stream_table[i];
		const int32_t interval_us = entry.interval_us[channel];

		if (interval_us == MAVLINK_EXT_STREAM_DISABLED) {
			continue;
		}

		if (interval_us > 0 && (now - entry.last_sent[channel]) < (hrt_abstime)interval_us) {
			continue;
		}

		if (entry.fn(channel, entry.user_data)) {
			entry.last_sent[channel] = now;
		}
	}
}
