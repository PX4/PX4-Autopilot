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

/**
 * @file mavlink_ulog.h
 * ULog data streaming via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/sem.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ulog_stream.h>
#include <uORB/topics/ulog_stream_ack.h>

#include "mavlink_bridge_header.h"

using namespace time_literals;

/**
 * @class MavlinkULog
 * ULog streaming class. At most one instance (stream) can exist, assigned to a specific mavlink channel.
 */
class MavlinkULog
{
public:
	/**
	 * initialize: call this once on startup (this function is not thread-safe!)
	 */
	static void initialize();

	/**
	 * try to start a new stream. This fails if a stream is already running.
	 * thread-safe
	 * @param datarate maximum link data rate in B/s
	 * @param max_rate_factor let ulog streaming use a maximum of max_rate_factor * datarate
	 * @param target_system ID for mavlink message
	 * @param target_component ID for mavlink message
	 * @return instance, or nullptr
	 */
	static MavlinkULog *try_start(int datarate, float max_rate_factor, uint8_t target_system, uint8_t target_component);

	/**
	 * stop the stream. It also deletes the singleton object, so make sure cleanup
	 * all pointers to it.
	 * thread-safe
	 */
	void stop();

	/**
	 * periodic update method: check for ulog stream messages and handle retransmission.
	 * @return 0 on success, <0 otherwise
	 */
	int handle_update(mavlink_channel_t channel);

	/** ack from mavlink for a data message */
	void handle_ack(mavlink_logging_ack_t ack);

	/** this is called when we got an vehicle_command_ack from the logger */
	void start_ack_received();

	float current_data_rate() const { return _current_rate_factor; }
	float maximum_data_rate() const { return _max_rate_factor; }

private:

	MavlinkULog(int datarate, float max_rate_factor, uint8_t target_system, uint8_t target_component);

	~MavlinkULog();

	static void lock()
	{
		do {} while (px4_sem_wait(&_lock) != 0);
	}

	static void unlock()
	{
		px4_sem_post(&_lock);
	}

	void publish_ack(uint16_t sequence);

	static px4_sem_t _lock;
	static bool _init;
	static MavlinkULog *_instance;
	static constexpr hrt_abstime _rate_calculation_delta_t = 100_ms; ///< rate update interval

	uORB::SubscriptionData<ulog_stream_s> _ulog_stream_sub{ORB_ID(ulog_stream)};
	uORB::Publication<ulog_stream_ack_s> _ulog_stream_ack_pub{ORB_ID(ulog_stream_ack)};
	uint16_t _wait_for_ack_sequence;
	uint8_t _sent_tries = 0;
	volatile bool _ack_received = false; ///< set to true if a matching ack received
	hrt_abstime _last_sent_time = 0; ///< last time we sent a message that requires an ack
	bool _waiting_for_initial_ack = false;
	const uint8_t _target_system;
	const uint8_t _target_component;

	const float _max_rate_factor; ///< maximum rate percentage at which we're allowed to push data
	const int _max_num_messages; ///< maximum number of messages we can send within _rate_calculation_delta_t
	float _current_rate_factor; ///< currently used rate percentage
	int _current_num_msgs = 0;  ///< number of messages sent within the current time interval
	hrt_abstime _next_rate_check; ///< next timestamp at which to update the rate

	perf_counter_t _msg_missed_ulog_stream_perf{perf_alloc(PC_COUNT, MODULE_NAME": ulog_stream messages missed")};

	/* do not allow copying this class */
	MavlinkULog(const MavlinkULog &) = delete;
	MavlinkULog operator=(const MavlinkULog &) = delete;
};
