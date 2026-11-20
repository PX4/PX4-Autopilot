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
 * @file mavlink_ext_handler.cpp
 */

#include "mavlink_bridge_header.h"
#include "mavlink_ext_handler.h"

#include <containers/LockGuard.hpp>
#include <px4_platform_common/atomic.h>
#include <pthread.h>

struct mavlink_ext_handler_entry_t {
	uint32_t msg_id;
	mavlink_ext_handler_fn handler;
	void *user_data;
};

// One mutex serialises registration, dispatch and unregistration. Holding it
// across the handler call is what lets unregister() promise that no handler is
// still running when it returns. The count is atomic only so dispatch can skip
// the lock while nothing is registered; every table access is under the mutex.
static mavlink_ext_handler_entry_t handler_table[MAVLINK_EXT_HANDLER_MAX] {};
static px4::atomic<unsigned> handler_count {0};
static pthread_mutex_t handler_mutex = PTHREAD_MUTEX_INITIALIZER;

static int handler_find(uint32_t msg_id)
{
	const unsigned count = handler_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (handler_table[i].msg_id == msg_id) {
			return (int)i;
		}
	}

	return -1;
}

int mavlink_ext_handler_register(uint32_t msg_id, mavlink_ext_handler_fn handler, void *user_data)
{
	if (handler == nullptr) {
		return -1;
	}

	LockGuard lg{handler_mutex};
	const unsigned count = handler_count.load();

	if (count >= MAVLINK_EXT_HANDLER_MAX || handler_find(msg_id) >= 0) {
		return -1;
	}

	handler_table[count] = {msg_id, handler, user_data};
	handler_count.store(count + 1);
	return 0;
}

int mavlink_ext_handler_unregister(uint32_t msg_id)
{
	LockGuard lg{handler_mutex};
	const int i = handler_find(msg_id);

	if (i < 0) {
		return -1;
	}

	// Table order is irrelevant to dispatch: fill the hole with the last entry.
	const unsigned last = handler_count.load() - 1;
	handler_table[i] = handler_table[last];
	handler_table[last] = {};
	handler_count.store(last);
	return 0;
}

bool mavlink_ext_handler_dispatch(const mavlink_message_t *msg)
{
	if (msg == nullptr || handler_count.load() == 0) {
		return false;
	}

	LockGuard lg{handler_mutex};
	const int i = handler_find(msg->msgid);
	return (i >= 0) && handler_table[i].handler(msg, handler_table[i].user_data);
}
