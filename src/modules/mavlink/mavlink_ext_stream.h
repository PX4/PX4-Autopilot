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
 * @file mavlink_ext_stream.h
 *
 * Generic external MAVLink outbound stream registration.
 *
 * Allows out-of-tree / external modules to register callbacks that
 * emit custom MAVLink messages on all active channels. The callbacks
 * are invoked from the mavlink module's stream update loop.
 *
 * The callback receives a mavlink_channel_t and should call the
 * appropriate mavlink_msg_*_send_struct() to emit the message.
 */

#pragma once

#include <cstdint>

/**
 * Callback signature for external outbound streams.
 *
 * Called from the mavlink main loop for each active mavlink instance.
 * The callback should check for new data (e.g. uORB subscription) and
 * send a MAVLink message via mavlink_msg_*_send_struct(channel, &msg).
 *
 * @param channel    MAVLink channel index (cast to mavlink_channel_t in callback)
 * @param user_data  Opaque pointer passed at registration time
 * @return true if a message was sent
 */
typedef bool (*mavlink_ext_stream_fn)(uint8_t channel, void *user_data);

/** Maximum number of concurrently registered external streams */
static constexpr unsigned MAVLINK_EXT_STREAM_MAX = 8;

/**
 * Register an external outbound stream.
 *
 * @param msg_id     MAVLink message ID (for identification/logging)
 * @param name       Human-readable stream name (for `mavlink stream` command)
 * @param fn         Callback function invoked each iteration
 * @param user_data  Opaque context pointer
 * @return 0 on success, -1 if table full or msg_id already registered
 */
int mavlink_ext_stream_register(uint32_t msg_id, const char *name,
				mavlink_ext_stream_fn fn, void *user_data);

/**
 * Unregister a previously registered external stream.
 *
 * @param msg_id  MAVLink message ID to unregister
 * @return 0 on success, -1 if not found
 */
int mavlink_ext_stream_unregister(uint32_t msg_id);

/**
 * Dispatch all registered external streams on a given channel.
 * Called from Mavlink::task_main() after the regular stream update loop.
 *
 * @param chan  MAVLink channel to emit on
 */
void mavlink_ext_stream_dispatch(uint8_t channel);
