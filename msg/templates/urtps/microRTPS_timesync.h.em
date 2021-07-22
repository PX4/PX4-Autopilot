@###############################################
@#
@# EmPy template for generating microRTPS_timesync.h file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - package (List[str]) messages package name. Defaulted to 'px4'
@#  - ros2_distro (List[str]) ROS2 distro name
@###############################################
@{
import genmsg.msgs
from packaging import version
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

package = package[0]
fastrtps_version = fastrtps_version[0]
try:
    ros2_distro = ros2_distro[0].decode("utf-8")
except AttributeError:
    ros2_distro = ros2_distro[0]
}@
/****************************************************************************
 *
 * Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
#include "TimesyncStatus_Publisher.h"

#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
@[else]@
#include "timesync_Publisher.h"
#include "timesync_status_Publisher.h"
@[end if]@

static constexpr double ALPHA_INITIAL = 0.05;
static constexpr double ALPHA_FINAL = 0.003;
static constexpr double BETA_INITIAL = 0.05;
static constexpr double BETA_FINAL = 0.003;
static constexpr int WINDOW_SIZE = 500;
static constexpr int64_t UNKNOWN = 0;
static constexpr int64_t TRIGGER_RESET_THRESHOLD_NS = 100ll * 1000ll * 1000ll;
static constexpr int REQUEST_RESET_COUNTER_THRESHOLD = 5;

@# Sets the timesync DDS type according to the FastRTPS and ROS2 version
@[if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
@[    if ros2_distro]@
using timesync_msg_t = @(package)::msg::dds_::Timesync_;
using timesync_status_msg_t = @(package)::msg::dds_::TimesyncStatus_;
@[    else]@
using timesync_msg_t = timesync_;
using timesync_status_msg_t = timesync_status_;
@[    end if]@
@[else]@
@[    if ros2_distro]@
using timesync_msg_t = @(package)::msg::Timesync;
using timesync_status_msg_t = @(package)::msg::TimesyncStatus;
@[    else]@
using timesync_msg_t = timesync;
using timesync_status_msg_t = timesync_status;
@[    end if]@
@[end if]@
@# Sets the timesync publisher entity depending on using ROS2 or not
@[if ros2_distro]@
using TimesyncPublisher = Timesync_Publisher;
using TimesyncStatusPublisher = TimesyncStatus_Publisher;
@[else]@
using TimesyncPublisher = timesync_Publisher;
using TimesyncStatusPublisher = timesync_status_Publisher;
@[end if]@

class TimeSync
{
public:
	TimeSync(bool debug);
	virtual ~TimeSync();

	/**
	 * @@brief Starts the timesync publishing thread
	 * @@param[in] pub The timesync publisher entity to use
	 */
	void start(TimesyncPublisher *pub);

	/**
	 * @@brief Init and run the timesync status publisher thread
	 * @@param[in] pub The timesync status publisher entity to use
	 */
	void init_status_pub(TimesyncStatusPublisher *status_pub);

	/**
	 * @@brief Resets the filter
	 */
	void reset();

	/**
	 * @@brief Stops the timesync publishing thread
	 */
	void stop();

@[if ros2_distro]@
	/**
	 * @@brief Get ROS time in nanoseconds. This will match the system time, which
	 *         corresponds to the system-wide real time since epoch. If use_sim_time
	 *         is set, the simulation time is grabbed by the node and used instead
	 *         More info about ROS2 clock and time in:
	 *         https://design.ros2.org/articles/clock_and_time.html
	 * @@return ROS time in nanoseconds
	 */
	uint64_t getROSTimeNSec() const;

	/**
	 * @@brief Get ROS time in microseconds. Fetches the time from getROSTimeNSec()
	 *         and converts it to microseconds
	 * @@return ROS time in microseconds
	 */
	uint64_t getROSTimeUSec() const;
@[else]@
	/**
	 * @@brief Get clock monotonic time (raw) in nanoseconds
	 * @@return Steady CLOCK_MONOTONIC time in nanoseconds
	 */
	uint64_t getSteadyTimeNSec() const;

	/**
	 * @@brief Get clock monotonic time (raw) in microseconds
	 * @@return Steady CLOCK_MONOTONIC time in microseconds
	 */
	uint64_t getSteadyTimeUSec() const;
@[end if]@

	/**
	 * @@brief Adds a time offset measurement to be filtered
	 * @@param[in] local_t1_ns The agent CLOCK_MONOTONIC_RAW time in nanoseconds when the message was sent
	 * @@param[in] remote_t2_ns The (client) remote CLOCK_MONOTONIC time in nanoseconds
	 * @@param[in] local_t3_ns The agent current CLOCK_MONOTONIC time in nanoseconds
	 * @@return true or false depending if the time offset was updated
	 */
	bool addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns);

	/**
	 * @@brief Processes DDS timesync message
	 * @@param[in,out] msg The timestamp msg to be processed
	 */
	void processTimesyncMsg(timesync_msg_t *msg, TimesyncPublisher *pub);

	/**
	 * @@brief Creates a new timesync DDS message to be sent from the agent to the client
	 * @@return A new timesync message with the origin in the agent and with the agent timestamp
	 */
	timesync_msg_t newTimesyncMsg();

	/**
	 * @@brief Creates a new timesync status DDS message to be sent from the agent to the client
	 * @@return A new timesync status message with the origin in the agent and with the agent timestamp
	 */
	timesync_status_msg_t newTimesyncStatusMsg();

	/**
	 * @@brief Get the time sync offset in nanoseconds
	 * @@return The offset in nanoseconds
	 */
	inline int64_t getOffset() { return _offset_ns.load(); }

	/**
	 * @@brief Sums the time sync offset to the timestamp
	 * @@param[in,out] timestamp The timestamp to add the offset to
	 */
	inline void addOffset(uint64_t &timestamp) { timestamp = (timestamp * 1000LL + _offset_ns.load()) / 1000ULL; }

	/**
	 * @@brief Substracts the time sync offset to the timestamp
	 * @@param[in,out] timestamp The timestamp to subtract the offset of
	 */
	inline void subtractOffset(uint64_t &timestamp) { timestamp = (timestamp * 1000LL - _offset_ns.load()) / 1000ULL; }

private:
	std::atomic<int64_t> _offset_ns;
	std::atomic<int64_t> _offset_prev;
	std::atomic<uint64_t> _remote_time_stamp;
	std::atomic<uint32_t> _rtti;

@[if ros2_distro]@
	/**
	 * @@brief A ROS2 node to fetch the ROS time to be used for timesync
	 */
	std::shared_ptr<rclcpp::Node> _timesync_node;
@[end if]@

	int64_t _skew_ns_per_sync;
	int64_t _num_samples;

	int32_t _request_reset_counter;
	uint8_t _last_msg_seq;
	uint8_t _last_remote_msg_seq;

	bool _debug;

	std::unique_ptr<std::thread> _send_timesync_thread;
	std::unique_ptr<std::thread> _send_timesync_status_thread;
@[if ros2_distro]@
	std::unique_ptr<std::thread> _timesync_node_thread;
@[end if]@
	std::atomic<bool> _request_stop{false};

	/**
	 * @@brief Updates the offset of the time sync filter
	 * @@param[in] offset The value of the offset to update to
	 */
	inline void updateOffset(const uint64_t &offset) { _offset_ns.store(offset, std::memory_order_relaxed); }

	/** Timesync msg Getters **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
	inline uint64_t getMsgTimestamp(const timesync_msg_t *msg) { return msg->timestamp_(); }
	inline uint8_t getMsgSeq(const timesync_msg_t *msg) { return msg->seq_(); }
	inline int64_t getMsgTC1(const timesync_msg_t *msg) { return msg->tc1_(); }
	inline int64_t getMsgTS1(const timesync_msg_t *msg) { return msg->ts1_(); }
@[elif ros2_distro]@
	inline uint64_t getMsgTimestamp(const timesync_msg_t *msg) { return msg->timestamp(); }
	inline uint8_t getMsgSeq(const timesync_msg_t *msg) { return msg->seq(); }
	inline int64_t getMsgTC1(const timesync_msg_t *msg) { return msg->tc1(); }
	inline int64_t getMsgTS1(const timesync_msg_t *msg) { return msg->ts1(); }
	@[end if]@

	/** Common timestamp setter **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
	template <typename T>
	inline void setMsgTimestamp(T *msg, const uint64_t &timestamp) { msg->timestamp_() = timestamp; }
@[elif ros2_distro]@
	template <typename T>
	inline void setMsgTimestamp(T *msg, const uint64_t &timestamp) { msg->timestamp() = timestamp; }
@[end if]@

	/** Timesync msg Setters **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
	inline void setMsgSeq(timesync_msg_t *msg, const uint8_t &seq) { msg->seq_() = seq; }
	inline void setMsgTC1(timesync_msg_t *msg, const int64_t &tc1) { msg->tc1_() = tc1; }
	inline void setMsgTS1(timesync_msg_t *msg, const int64_t &ts1) { msg->ts1_() = ts1; }
@[elif ros2_distro]@
	inline void setMsgSeq(timesync_msg_t *msg, const uint8_t &seq) { msg->seq() = seq; }
	inline void setMsgTC1(timesync_msg_t *msg, const int64_t &tc1) { msg->tc1() = tc1; }
	inline void setMsgTS1(timesync_msg_t *msg, const int64_t &ts1) { msg->ts1() = ts1; }
@[end if]@

	/** Timesync Status msg Setters **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
	inline void setMsgSourceProtocol(timesync_status_msg_t *msg, const uint8_t &source_protocol) { msg->source_protocol_() = source_protocol; }
	inline void setMsgRemoteTimeStamp(timesync_status_msg_t *msg, const uint64_t &remote_timestamp) { msg->remote_timestamp_() = remote_timestamp; }
	inline void setMsgObservedOffset(timesync_status_msg_t *msg, const int64_t &observed_offset) { msg->observed_offset_() = observed_offset; }
	inline void setMsgEstimatedOffset(timesync_status_msg_t *msg, const int64_t &estimated_offset) { msg->estimated_offset_() = estimated_offset; }
	inline void setMsgRoundTripTime(timesync_status_msg_t *msg, const uint32_t &round_trip_time) { msg->round_trip_time_() = round_trip_time; }
@[elif ros2_distro]@
	inline void setMsgSourceProtocol(timesync_status_msg_t *msg, const uint8_t &source_protocol) { msg->source_protocol() = source_protocol; }
	inline void setMsgRemoteTimeStamp(timesync_status_msg_t *msg, const uint64_t &remote_timestamp) { msg->remote_timestamp() = remote_timestamp; }
	inline void setMsgObservedOffset(timesync_status_msg_t *msg, const int64_t &observed_offset) { msg->observed_offset() = observed_offset; }
	inline void setMsgEstimatedOffset(timesync_status_msg_t *msg, const int64_t &estimated_offset) { msg->estimated_offset() = estimated_offset; }
	inline void setMsgRoundTripTime(timesync_status_msg_t *msg, const uint32_t &round_trip_time) { msg->round_trip_time() = round_trip_time; }
@[end if]@
};
