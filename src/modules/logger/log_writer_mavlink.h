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

#pragma once

#include <stdint.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/ulog_stream.h>
#include <uORB/topics/ulog_stream_ack.h>
#include <containers/List.hpp>
#include "messages.h"

#ifdef LOGGER_PARALLEL_LOGGING
static constexpr size_t LOGGER_ULOG_STREAM_DATA_LEN {249}; // Size of ulog_stream data buffer
static constexpr size_t LOGGER_RELIABLE_FIFO_WAIT_THRESHOLD{10}; // Msg count threshold for wait fifo trigger
#endif

namespace px4
{
namespace logger
{

#ifdef LOGGER_PARALLEL_LOGGING
class ReliableMsg : public ListNode<ReliableMsg *>
{
public:
	uint16_t len;
	ULogWriteType wrtype;
	uint8_t data[LOGGER_ULOG_STREAM_DATA_LEN];
};

class ReliableFifo : public List<ReliableMsg *>
{
public:
	pthread_mutex_t	mtx;
	pthread_cond_t cv;
	px4::atomic_bool sender_should_exit{false};
	bool sending{false};
};

#endif

/**
 * @class LogWriterMavlink
 * Writes logging data to uORB, and then sent via mavlink
 */
class LogWriterMavlink
{
public:
	LogWriterMavlink();
	~LogWriterMavlink();

	bool init();

	void start_log();

	void stop_log();

	bool is_started() const { return _is_started; }

	/** @see LogWriter::write_message() */
	int write_message(void *ptr, size_t size, ULogWriteType wrtype = ULogWriteType::BEST_EFFORT);
#ifdef LOGGER_PARALLEL_LOGGING
	void allow_delayed_sending()
	{
		PX4_INFO("allow_delayed_sending");
		_delayed_sending_allowed = true;
	}
	void stop_log_req()
	{
		_stop_log_request = true;
	}
	int write_reliable_message(void *ptr, size_t size, ULogWriteType wrtype);
	bool reliable_fifo_is_sending();
	void wait_fifo_count(size_t count);
	void wait_fifos_empty();
#else
	void set_need_reliable_transfer(bool need_reliable);
#endif

	bool need_reliable_transfer() const
	{
		return _need_reliable_transfer;
	}

private:
#ifdef LOGGER_PARALLEL_LOGGING
	static void *mav_reliable_sender_helper(void *context);
	void mav_reliable_sender();

	ReliableMsg *reliable_fifo_pop();
	bool reliable_fifo_push(ReliableMsg *node, bool delayed);
	void reliable_fifo_set_sender_idle();

	size_t reliable_fifo_count();
#endif

	/** publish message, wait for ack if needed & reset message */
	int publish_message(ULogWriteType wrtype = ULogWriteType::BEST_EFFORT);

	ulog_stream_s _ulog_stream_data{};
	uORB::Publication<ulog_stream_s> _ulog_stream_pub{ORB_ID(ulog_stream)};
	orb_sub_t _ulog_stream_ack_sub{ORB_SUB_INVALID};
	bool _need_reliable_transfer{false};
	bool _is_started{false};
#ifdef LOGGER_PARALLEL_LOGGING
	ulog_stream_s _ulog_stream_acked_data {};
	uORB::Publication<ulog_stream_s> _ulog_stream_acked_pub{ORB_ID(ulog_stream_acked)};
	ReliableFifo _fifo;
	ReliableFifo _fifo_delayed;
	bool _delayed_sending_allowed = false;
	pthread_t _mav_reliable_sender_thread = 0;
	bool _stop_log_request = false;
#endif
};

}
}
