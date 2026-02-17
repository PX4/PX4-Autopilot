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

#include "mavlink_tx_queue.h"
#include "mavlink_main.h"

unsigned MavlinkTxQueue::drain(Mavlink &mavlink, unsigned max_bytes)
{
	unsigned total_sent = 0;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	while (max_bytes == 0 || total_sent < max_bytes) {
		size_t len;

		pthread_mutex_lock(&_mutex);
		len = _ringbuffer.pop_front(buf, sizeof(buf));
		pthread_mutex_unlock(&_mutex);

		if (len == 0) {
			break; // queue empty
		}

		// Check budget before sending
		if (max_bytes > 0 && (total_sent + len) > max_bytes) {
			// Message won't fit in remaining budget.
			// Unfortunately we already popped it, so we have to
			// push it back or just send it. Since partial-message
			// re-queue is complex, just send it and exceed budget
			// slightly for this cycle.
		}

		// Send through the standard Mavlink send path
		mavlink.send_start(len);
		mavlink.send_bytes(buf, len);
		mavlink.send_finish();

		total_sent += len;
	}

	return total_sent;
}
