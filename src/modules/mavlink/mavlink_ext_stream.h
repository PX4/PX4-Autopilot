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
 * Outbound MAVLink messages for out-of-tree modules: periodic streams driven
 * from each mavlink instance's main loop, and one-shot sends to every running
 * instance.
 *
 * Compiled only when EXTERNAL_MODULES_LOCATION is set.
 */

#pragma once

#include <cstdint>

/**
 * Send callback: emit on `channel` with mavlink_msg_<name>_send_struct() and
 * return true if a message was sent.
 *
 * As a stream callback it runs on that instance's main thread with the
 * registry mutex and the channel send lock held: keep it short and never call
 * mavlink_ext_stream_register()/unregister() or mavlink_ext_send() from it.
 */
typedef bool (*mavlink_ext_send_fn)(uint8_t channel, void *user_data);

static constexpr unsigned MAVLINK_EXT_STREAM_MAX = 8;

/** Interval values; a positive value is the minimum spacing in microseconds. */
static constexpr int32_t MAVLINK_EXT_STREAM_UNLIMITED = -1;	///< send on every main loop iteration
static constexpr int32_t MAVLINK_EXT_STREAM_DISABLED = 0;
static constexpr int32_t MAVLINK_EXT_STREAM_DEFAULT = -2;	///< set_interval only: restore the registered interval

/**
 * Register a stream. interval_us applies on every channel until changed per
 * channel by mavlink_ext_stream_set_interval().
 * @return 0 on success, -1 if fn is null, interval_us is invalid, the table is full or msg_id is already registered
 */
int mavlink_ext_stream_register(uint32_t msg_id, mavlink_ext_send_fn fn, void *user_data,
				int32_t interval_us = MAVLINK_EXT_STREAM_UNLIMITED);

/**
 * Returns only after any in-flight invocation of the callback has completed,
 * so user_data may be freed afterwards.
 * @return 0 on success, -1 if msg_id is not registered
 */
int mavlink_ext_stream_unregister(uint32_t msg_id);

/**
 * Per-channel rate control. Backs SET_MESSAGE_INTERVAL for external streams so
 * a GCS controls them on its own link exactly like built-in streams.
 * @return 0 on success, -1 if msg_id is not registered or channel/interval_us is invalid
 */
int mavlink_ext_stream_set_interval(uint8_t channel, uint32_t msg_id, int32_t interval_us);

/** Called by Mavlink::task_main() once per loop iteration, after the built-in streams. */
void mavlink_ext_stream_dispatch(uint8_t channel);

/**
 * One-shot send: invokes fn once per running mavlink instance on the caller's
 * thread while holding that instance's send lock. Safe from handler callbacks
 * and module threads, never from a stream callback (see mavlink_ext_send_fn).
 * @return number of instances on which fn returned true, -1 if fn is null
 */
int mavlink_ext_send(mavlink_ext_send_fn fn, void *user_data);
