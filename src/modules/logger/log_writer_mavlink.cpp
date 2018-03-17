/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include "log_writer_mavlink.h"
#include "messages.h"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_log.h>
#include <px4_posix.h>

namespace px4
{
namespace logger
{


LogWriterMavlink::LogWriterMavlink(unsigned int queue_size) :
	_queue_size(queue_size)
{
	_ulog_stream_data.length = 0;
}

bool LogWriterMavlink::init()
{
	return true;
}

LogWriterMavlink::~LogWriterMavlink()
{
	if (_ulog_stream_ack_sub >= 0) {
		orb_unsubscribe(_ulog_stream_ack_sub);
	}

	if (_ulog_stream_pub) {
		orb_unadvertise(_ulog_stream_pub);
	}
}

void LogWriterMavlink::start_log()
{
	if (_ulog_stream_ack_sub == -1) {
		_ulog_stream_ack_sub = orb_subscribe(ORB_ID(ulog_stream_ack));
	}

	// make sure we don't get any stale ack's by doing an orb_copy
	ulog_stream_ack_s ack;
	orb_copy(ORB_ID(ulog_stream_ack), _ulog_stream_ack_sub, &ack);
	_ulog_stream_data.sequence = 0;
	_ulog_stream_data.length = 0;
	_ulog_stream_data.first_message_offset = 0;
	_is_started = true;
}

void LogWriterMavlink::stop_log()
{
	_ulog_stream_data.length = 0;
	_is_started = false;
}

int LogWriterMavlink::write_message(void *ptr, size_t size)
{
	if (!is_started()) {
		return 0;
	}

	const uint8_t data_len = (uint8_t)sizeof(_ulog_stream_data.data);
	uint8_t *ptr_data = (uint8_t *)ptr;

	if (_ulog_stream_data.first_message_offset == 255) {
		_ulog_stream_data.first_message_offset = _ulog_stream_data.length;
	}

	while (size > 0) {
		size_t send_len = math::min((size_t)data_len - _ulog_stream_data.length, size);
		memcpy(_ulog_stream_data.data + _ulog_stream_data.length, ptr_data, send_len);
		_ulog_stream_data.length += send_len;
		ptr_data += send_len;
		size -= send_len;

		if (_ulog_stream_data.length >= data_len) {
			if (publish_message()) {
				return -2;
			}
		}
	}

	return 0;
}

void LogWriterMavlink::set_need_reliable_transfer(bool need_reliable)
{
	if (!need_reliable && _need_reliable_transfer) {
		if (_ulog_stream_data.length > 0) {
			// make sure to send previous data using reliable transfer
			publish_message();
		}
	}

	_need_reliable_transfer = need_reliable;
}

int LogWriterMavlink::publish_message()
{
	_ulog_stream_data.timestamp = hrt_absolute_time();
	_ulog_stream_data.flags = 0;

	if (_need_reliable_transfer) {
		_ulog_stream_data.flags = _ulog_stream_data.FLAGS_NEED_ACK;
	}

	if (_ulog_stream_pub == nullptr) {
		_ulog_stream_pub = orb_advertise_queue(ORB_ID(ulog_stream), &_ulog_stream_data, _queue_size);

	} else {
		orb_publish(ORB_ID(ulog_stream), _ulog_stream_pub, &_ulog_stream_data);
	}

	if (_need_reliable_transfer) {
		// we need to wait for an ack. Note that this blocks the main logger thread, so if a file logging
		// is already running, it will miss samples.
		px4_pollfd_struct_t fds[1];
		fds[0].fd = _ulog_stream_ack_sub;
		fds[0].events = POLLIN;
		bool got_ack = false;
		const int timeout_ms = ulog_stream_ack_s::ACK_TIMEOUT * ulog_stream_ack_s::ACK_MAX_TRIES;

		hrt_abstime started = hrt_absolute_time();

		do {
			int ret = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), timeout_ms);

			if (ret <= 0) {
				break;
			}

			if (fds[0].revents & POLLIN) {
				ulog_stream_ack_s ack;
				orb_copy(ORB_ID(ulog_stream_ack), _ulog_stream_ack_sub, &ack);

				if (ack.sequence == _ulog_stream_data.sequence) {
					got_ack = true;
				}

			} else {
				break;
			}
		} while (!got_ack && hrt_elapsed_time(&started) / 1000 < timeout_ms);

		if (!got_ack) {
			PX4_ERR("Ack timeout. Stopping mavlink log");
			stop_log();
			return -2;
		}

		PX4_DEBUG("got ack in %i ms", (int)(hrt_elapsed_time(&started) / 1000));
	}

	_ulog_stream_data.sequence++;
	_ulog_stream_data.length = 0;
	_ulog_stream_data.first_message_offset = 255;
	return 0;
}

}
}
