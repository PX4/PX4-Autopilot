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
 *
 * External MAVLink message handler registry implementation.
 *
 * Uses a simple static array with atomic count for lock-free dispatch.
 * Registration is not thread-safe with itself (callers must not register
 * concurrently), but dispatch is safe to call from the mavlink receiver
 * thread while registration happens from a module init context.
 */

#include "mavlink_bridge_header.h"  // Full mavlink_message_t definition (for ->msgid)
#include "mavlink_ext_handler.h"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>
#include <cstring>

struct mavlink_ext_handler_entry_t {
	uint32_t msg_id;
	mavlink_ext_handler_fn handler;
	void *user_data;
};

static mavlink_ext_handler_entry_t _handlers[MAVLINK_EXT_HANDLER_MAX] {};
static px4::atomic<unsigned> _handler_count {0};

int mavlink_ext_handler_register(uint32_t msg_id, mavlink_ext_handler_fn handler, void *user_data)
{
	if (!handler) {
		return -1;
	}

	unsigned count = _handler_count.load();

	// Check for duplicate
	for (unsigned i = 0; i < count; i++) {
		if (_handlers[i].msg_id == msg_id) {
			return -1;
		}
	}

	if (count >= MAVLINK_EXT_HANDLER_MAX) {
		return -1;
	}

	// Write entry before publishing count (release semantics via atomic store)
	_handlers[count].msg_id = msg_id;
	_handlers[count].handler = handler;
	_handlers[count].user_data = user_data;
	_handler_count.store(count + 1);

	PX4_INFO("ext_handler: registered msgid %lu (count=%u)", (unsigned long)msg_id, count + 1);

	return 0;
}

int mavlink_ext_handler_unregister(uint32_t msg_id)
{
	unsigned count = _handler_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (_handlers[i].msg_id == msg_id) {
			// Shift remaining entries down
			if (i < count - 1) {
				memmove(&_handlers[i], &_handlers[i + 1],
					(count - i - 1) * sizeof(mavlink_ext_handler_entry_t));
			}

			_handler_count.store(count - 1);
			return 0;
		}
	}

	return -1;
}

bool mavlink_ext_handler_dispatch(const mavlink_message_t *msg)
{
	if (!msg) {
		return false;
	}

	unsigned count = _handler_count.load();

	for (unsigned i = 0; i < count; i++) {
		if (_handlers[i].msg_id == msg->msgid) {
			PX4_DEBUG("ext_handler: dispatching msgid %lu", (unsigned long)msg->msgid);
			return _handlers[i].handler(msg, _handlers[i].user_data);
		}
	}

	return false;
}
