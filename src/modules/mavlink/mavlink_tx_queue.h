/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file mavlink_tx_queue.h
 *
 * Thread-safe TX queue for MAVLink service messages.
 *
 * All service responses (parameters, FTP, missions, etc.) are encoded and
 * pushed into this queue from any thread, then drained by the TX thread
 * under bandwidth-aware scheduling.
 */

#pragma once

#include <pthread.h>
#include <lib/variable_length_ringbuffer/VariableLengthRingbuffer.hpp>
#include "mavlink_bridge_header.h"

class Mavlink;

class MavlinkTxQueue
{
public:
	MavlinkTxQueue() = default;
	~MavlinkTxQueue() { pthread_mutex_destroy(&_mutex); }

	bool init(size_t buffer_size = 4096)
	{
		pthread_mutex_init(&_mutex, nullptr);
		return _ringbuffer.allocate(buffer_size);
	}

	/**
	 * Push an encoded mavlink message into the queue as wire bytes.
	 * Thread-safe: may be called from RX or TX thread.
	 *
	 * @return true if message was queued, false if queue is full
	 */
	bool push(const mavlink_message_t &msg)
	{
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

		pthread_mutex_lock(&_mutex);
		const bool ok = _ringbuffer.push_back(buf, len);
		pthread_mutex_unlock(&_mutex);

		return ok;
	}

	/**
	 * Drain queued wire-format messages, sending up to max_bytes.
	 * Called from TX thread only.
	 *
	 * Uses Mavlink::send_start/send_bytes/send_finish to go through the
	 * standard send path with buffer-full detection and byte accounting.
	 *
	 * @param mavlink	Reference to Mavlink instance for sending
	 * @param max_bytes	Maximum bytes to send this cycle (0 = unlimited)
	 * @return		Total bytes drained from the queue
	 */
	unsigned drain(Mavlink &mavlink, unsigned max_bytes);

private:
	VariableLengthRingbuffer _ringbuffer{};
	pthread_mutex_t _mutex{};
};
