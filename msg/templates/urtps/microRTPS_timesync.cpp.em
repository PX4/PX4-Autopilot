@###############################################
@#
@# EmPy template for generating microRTPS_timesync.cpp file
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
 * @@file microRTPS_timesync.cpp
 * @@brief source code for time sync implementation
 */

#include <time.h>
#include <cmath>
#include <iostream>

#include "microRTPS_timesync.h"

TimeSync::TimeSync(bool debug)
	: _offset_ns(-1),
@[if ros2_distro]@
	  _timesync_node(std::make_shared<rclcpp::Node>("timesync_node")),
@[end if]@
	  _skew_ns_per_sync(0.0),
	  _num_samples(0),
	  _request_reset_counter(0),
	  _last_msg_seq(0),
	  _last_remote_msg_seq(0),
	  _debug(debug)
{ }

TimeSync::~TimeSync() { stop(); }

void TimeSync::start(TimesyncPublisher *pub)
{
	stop();

@[if ros2_distro]@
	auto spin_node = [this]() {
		rclcpp::spin(_timesync_node);
	};
@[end if]@

	auto run = [this, pub]() {
		while (!_request_stop) {
			timesync_msg_t msg = newTimesyncMsg();

			pub->publish(&msg);

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	};
	_request_stop = false;
@[if ros2_distro]@
	_timesync_node_thread.reset(new std::thread(spin_node));
@[end if]@
	_send_timesync_thread.reset(new std::thread(run));
}

void TimeSync::init_status_pub(TimesyncStatusPublisher *status_pub)
{
	auto run = [this, status_pub]() {
		while (!_request_stop) {
			timesync_status_msg_t status_msg = newTimesyncStatusMsg();

			status_pub->publish(&status_msg);

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	};
	_request_stop = false;
	_send_timesync_status_thread.reset(new std::thread(run));
}

void TimeSync::stop()
{
	_request_stop = true;

@[if ros2_distro]@
	if (_timesync_node_thread && _timesync_node_thread->joinable()) { _timesync_node_thread->join(); }
@[end if]@
	if (_send_timesync_thread && _send_timesync_thread->joinable()) { _send_timesync_thread->join(); }
	if (_send_timesync_status_thread && _send_timesync_status_thread->joinable()) { _send_timesync_status_thread->join(); }
}

void TimeSync::reset()
{
	_num_samples = 0;
	_request_reset_counter = 0;
}

@[if ros2_distro]@
uint64_t TimeSync::getROSTimeNSec() const
{
	return _timesync_node->now().nanoseconds();
}

uint64_t TimeSync::getROSTimeUSec() const
{
	return RCL_NS_TO_US(getROSTimeNSec());
}
@[else]@
uint64_t TimeSync::getSteadyTimeNSec() const
{
	auto time = std::chrono::steady_clock::now();
	return std::chrono::time_point_cast<std::chrono::nanoseconds>(time).time_since_epoch().count();
}

uint64_t TimeSync::getSteadyTimeUSec() const
{
	auto time = std::chrono::steady_clock::now();
	return std::chrono::time_point_cast<std::chrono::microseconds>(time).time_since_epoch().count();
}
@[end if]@

bool TimeSync::addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns)
{
	_rtti = local_t3_ns - local_t1_ns;
	_remote_time_stamp = remote_t2_ns;

	// assume rtti is evenly split both directions
	int64_t remote_t3_ns = remote_t2_ns + _rtti.load() / 2ll;

	int64_t measurement_offset = remote_t3_ns - local_t3_ns;

	if (_request_reset_counter > REQUEST_RESET_COUNTER_THRESHOLD) {
		reset();

		if (_debug) { std::cout << "\033[1;33m[ micrortps__timesync ]\tTimesync clock changed, resetting\033[0m" << std::endl; }
	}

	if (_num_samples == 0) {
		updateOffset(measurement_offset);
		_skew_ns_per_sync = 0;
	}

	if (_num_samples >= WINDOW_SIZE) {
		if (std::abs(measurement_offset - _offset_ns.load()) > TRIGGER_RESET_THRESHOLD_NS) {
			_request_reset_counter++;

			if (_debug) { std::cout << "\033[1;33m[ micrortps__timesync ]\tTimesync offset outlier, discarding\033[0m" << std::endl; }

			return false;

		} else {
			_request_reset_counter = 0;
		}
	}

	// ignore if rtti > 50ms
	if (_rtti.load() > 50ll * 1000ll * 1000ll) {
		if (_debug) { std::cout << "\033[1;33m[ micrortps__timesync ]\tRTTI too high for timesync: " << _rtti.load() / (1000ll * 1000ll) << "ms\033[0m" << std::endl; }

		return false;
	}

	double alpha = ALPHA_FINAL;
	double beta = BETA_FINAL;

	if (_num_samples < WINDOW_SIZE) {
		double schedule = (double)_num_samples / WINDOW_SIZE;
		double s = 1. - exp(.5 * (1. - 1. / (1. - schedule)));
		alpha = (1. - s) * ALPHA_INITIAL + s * ALPHA_FINAL;
		beta = (1. - s) * BETA_INITIAL + s * BETA_FINAL;
	}

	_offset_prev = _offset_ns.load();
	updateOffset(static_cast<int64_t>((_skew_ns_per_sync + _offset_ns.load()) * (1. - alpha) +
					  measurement_offset * alpha));
	_skew_ns_per_sync =
		static_cast<int64_t>(beta * (_offset_ns.load() - _offset_prev.load()) + (1. - beta) * _skew_ns_per_sync);

	_num_samples++;

	return true;
}

void TimeSync::processTimesyncMsg(timesync_msg_t *msg, TimesyncPublisher *pub)
{
	if (getMsgSeq(msg) != _last_remote_msg_seq) {
		_last_remote_msg_seq = getMsgSeq(msg);

		if (getMsgTC1(msg) > 0) {
@[if ros2_distro]@
			if (!addMeasurement(getMsgTS1(msg), getMsgTC1(msg), getROSTimeNSec())) {
@[else]@
			if (!addMeasurement(getMsgTS1(msg), getMsgTC1(msg), getSteadyTimeNSec())) {
@[end if]@
				if (_debug) { std::cerr << "\033[1;33m[ micrortps__timesync ]\tOffset not updated\033[0m" << std::endl; }
			}

		} else if (getMsgTC1(msg) == 0) {
@[if ros2_distro]@
			setMsgTimestamp(msg, getROSTimeUSec());
@[else]@
			setMsgTimestamp(msg, getSteadyTimeUSec());
@[end if]@
			setMsgSeq(msg, getMsgSeq(msg) + 1);
@[if ros2_distro]@
			setMsgTC1(msg, getROSTimeNSec());
@[else]@
			setMsgTC1(msg, getSteadyTimeNSec());
@[end if]@

			pub->publish(msg);
		}
	}
}

timesync_msg_t TimeSync::newTimesyncMsg()
{
	timesync_msg_t msg{};

@[if ros2_distro]@
	setMsgTimestamp(&msg, getROSTimeUSec());
@[else]@
	setMsgTimestamp(&msg, getSteadyTimeUSec());
@[end if]@
	setMsgSeq(&msg, _last_msg_seq);
	setMsgTC1(&msg, 0);
@[if ros2_distro]@
	setMsgTS1(&msg, getROSTimeNSec());
@[else]@
	setMsgTS1(&msg, getSteadyTimeNSec());
@[end if]@

	_last_msg_seq++;

	return msg;
}

timesync_status_msg_t TimeSync::newTimesyncStatusMsg()
{
	timesync_status_msg_t msg{};

@[if ros2_distro]@
	setMsgTimestamp(&msg, getROSTimeUSec());
@[else]@
	setMsgTimestamp(&msg, getSteadyTimeUSec());
@[end if]@
	setMsgSourceProtocol(&msg, 1); // SOURCE_PROTOCOL_RTPS
	setMsgRemoteTimeStamp(&msg, _remote_time_stamp.load() / 1000ULL);
	setMsgObservedOffset(&msg, _offset_prev.load());
	setMsgEstimatedOffset(&msg, _offset_ns.load());
	setMsgRoundTripTime(&msg, _rtti.load() / 1000ll);

	return msg;
}
