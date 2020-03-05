@###############################################
@#
@# EmPy template for generating microRTPS_timesync.h file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - multi_topics (List) list of all multi-topic names
@#  - ids (List) list of all RTPS msg ids
@###############################################
@{
import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

package = package[0]
fastrtpsgen_version = fastrtpsgen_version[0]
try:
    ros2_distro = ros2_distro[0].decode("utf-8")
except AttributeError:
    ros2_distro = ros2_distro[0]
}@
/****************************************************************************
 *
 * Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*!
 * @@file microRTPS_timesync.h
 * @@brief Adds time sync for the microRTPS bridge
 * @@author Nuno Marques <nuno.marques@@dronesolutions.io>
 * @@author Julian Kent <julian@@auterion.com>
 */

#pragma once

#include <atomic>
#include <functional>
#include <thread>

@[if ros2_distro]@
#include "Timesync_Publisher.h"
#include "Timesync_Subscriber.h"
@[else]@
#include "timesync_Publisher.h"
#include "timesync_Subscriber.h"
@[end if]@

static constexpr double ALPHA_INITIAL = 0.05;
static constexpr double ALPHA_FINAL = 0.003;
static constexpr double BETA_INTIIAL = 0.05;
static constexpr double BETA_FINAL = 0.003;
static constexpr int WINDOW_SIZE = 500;
static constexpr int64_t UNKNOWN = 0;
static constexpr int64_t TRIGGER_RESET_THRESHOLD_NS = 100ll * 1000ll * 1000ll;
static constexpr int REQUEST_RESET_COUNTER_THRESHOLD = 5;

class TimeSync {
public:
	TimeSync();
	virtual ~TimeSync();

@[if ros2_distro]@
	void start(const Timesync_Publisher* pub);
@[else]@
	void start(const timesync_Publisher* pub);
@[end if]@

	void reset();
	void stop();

	void setNewOffsetCB(std::function<void(int64_t)> callback);

	/**
	 * Returns clock monotonic time in nanoseconds
	 */
	inline int64_t getSystemMonoNanos() {
		timespec t;
		clock_gettime(CLOCK_MONOTONIC_RAW, &t);
		return (int64_t)t.tv_sec * 1000000000ll + t.tv_nsec;
	}

	/**
	 * Returns system realtime in seconds
	 */
	inline uint64_t getSystemTime() {
		timespec t;
		clock_gettime(CLOCK_REALTIME, &t);
		return (uint64_t)t.tv_sec + t.tv_nsec;
	}

	bool addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns);

@[if 1.5 <= fastrtpsgen_version <= 1.7]@
@[    if ros2_distro]@
	void processTimesyncMsg(const @(package)::msg::dds_::Timesync_* msg);

	@(package)::msg::dds_::Timesync_ newTimesyncMsg();
@[    else]@
	void processTimesyncMsg(const timesync_* msg);

	timesync_ newTimesyncMsg();
@[    end if]@
@[else]@
@[    if ros2_distro]@
	void processTimesyncMsg(const @(package)::msg::Timesync* msg);

	@(package)::msg::Timesync newTimesyncMsg();
@[    else]@
	void processTimesyncMsg(const timesync* msg);

	timesync newTimesyncMsg();
@[    end if]@
@[end if]@

	inline void applyOffset(uint64_t &timestamp) { timestamp += _offset_ns.load(); }

private:
	std::atomic<int64_t> _offset_ns;
	int64_t _skew_ns_per_sync;
	int64_t _num_samples;

	int32_t _request_reset_counter;
	uint8_t _last_msg_seq;

@[if ros2_distro]@
	Timesync_Publisher _timesync_pub;
	Timesync_Subscriber _timesync_sub;
@[else]@
	timesync_Publisher _timesync_pub;
	timesync_Subscriber _timesync_sub;
@[end if]@

	std::unique_ptr<std::thread> _send_timesync_thread;
	std::atomic<bool> _request_stop{false};

	std::function<void(int64_t)> _notifyNewOffset;
};
