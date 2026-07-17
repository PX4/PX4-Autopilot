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
 * Generic external MAVLink message handler registration.
 *
 * Allows out-of-tree / external modules to register callbacks for
 * custom MAVLink message IDs without modifying mavlink_receiver.cpp.
 * Callbacks are invoked from the receiver's default switch case.
 */

#pragma once

#include <cstdint>

// Forward declaration — the full definition comes from the dialect headers
struct __mavlink_message;
typedef struct __mavlink_message mavlink_message_t;

/**
 * Callback signature for external MAVLink message handlers.
 * Receives the raw mavlink_message_t; the handler is responsible for
 * decoding (e.g. mavlink_msg_*_decode) and publishing to uORB.
 *
 * @param msg        Parsed MAVLink message (CRC already validated)
 * @param user_data  Opaque pointer passed at registration time
 * @return true if the message was handled
 */
typedef bool (*mavlink_ext_handler_fn)(const mavlink_message_t *msg, void *user_data);

/** Maximum number of concurrently registered external handlers */
static constexpr unsigned MAVLINK_EXT_HANDLER_MAX = 8;

/**
 * Register a handler for a custom MAVLink message ID.
 *
 * @param msg_id     MAVLink message ID to handle
 * @param handler    Callback function
 * @param user_data  Opaque context pointer (e.g. module instance)
 * @return 0 on success, -1 if table full or msg_id already registered
 */
int mavlink_ext_handler_register(uint32_t msg_id, mavlink_ext_handler_fn handler, void *user_data);

/**
 * Unregister a previously registered handler.
 *
 * @param msg_id  MAVLink message ID to unregister
 * @return 0 on success, -1 if msg_id not found
 */
int mavlink_ext_handler_unregister(uint32_t msg_id);

/**
 * Dispatch a message to registered external handlers.
 * Called from MavlinkReceiver::handle_message() default case.
 *
 * @param msg  Parsed MAVLink message
 * @return true if a handler was found and invoked
 */
bool mavlink_ext_handler_dispatch(const mavlink_message_t *msg);
