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
 * @file mavlink_ext_handler.h
 *
 * Inbound MAVLink message handlers for out-of-tree modules.
 *
 * MavlinkReceiver::handle_message() dispatches every message ID it does not
 * handle itself to the handler registered for that ID, so out-of-tree modules
 * consume custom dialect messages without patching the receiver.
 *
 * Compiled only when EXTERNAL_MODULES_LOCATION is set.
 */

#pragma once

#include <cstdint>

struct __mavlink_message;
typedef struct __mavlink_message mavlink_message_t;

/**
 * Handler callback, invoked on the receiving mavlink instance's receiver thread
 * with the registry mutex held: keep it short and never call
 * mavlink_ext_handler_register()/unregister() from inside it.
 *
 * @param msg        CRC-validated message; decode with mavlink_msg_<name>_decode()
 * @param user_data  Pointer passed at registration
 * @return true if the message was consumed
 */
typedef bool (*mavlink_ext_handler_fn)(const mavlink_message_t *msg, void *user_data);

static constexpr unsigned MAVLINK_EXT_HANDLER_MAX = 8;

/** @return 0 on success, -1 if handler is null, the table is full or msg_id is already registered */
int mavlink_ext_handler_register(uint32_t msg_id, mavlink_ext_handler_fn handler, void *user_data);

/**
 * Returns only after any in-flight invocation of the handler has completed,
 * so user_data may be freed afterwards.
 * @return 0 on success, -1 if msg_id is not registered
 */
int mavlink_ext_handler_unregister(uint32_t msg_id);

/** Called by MavlinkReceiver::handle_message() for message IDs it does not handle. */
bool mavlink_ext_handler_dispatch(const mavlink_message_t *msg);
