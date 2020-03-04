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
 * @file microRTPS_timesync.cpp
 * @brief source code for time sync implementation
 */

#include <time.h>
#include <cmath>
#include <iostream>

#include "microRTPS_timesync.h"

#include "Timesync_Publisher.h"
#include "Timesync_Subscriber.h"

TimeSync::TimeSync()
    : _offset_ns(-1),
      _skew_ns_per_sync(0.0),
      _num_samples(0),
      _request_reset_counter(0),
      _last_msg_seq(0)
{}

TimeSync::~TimeSync() { stop(); }

void TimeSync::setNewOffsetCB(std::function<void(int64_t)> callback) { _notifyNewOffset = callback; }

void TimeSync::start(const Timesync_Publisher* pub) {
	stop();

	_Timesync_pub = (*pub);

	auto run = [this]() {
		while (!_request_stop) {
			px4_msgs::msg::Timesync msg = newTimesyncMsg();

			_Timesync_pub.publish(&msg);

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	};
	_request_stop = false;
	_send_timesync_thread.reset(new std::thread(run));
}

void TimeSync::stop() {
	_request_stop = true;
	if (_send_timesync_thread && _send_timesync_thread->joinable()) _send_timesync_thread->join();
	_send_timesync_thread.reset();
}

void TimeSync::reset() {
	_num_samples = 0;
	_request_reset_counter = 0;
}

bool TimeSync::addMeasurement(int64_t local_t1_ns, int64_t remote_t2_ns, int64_t local_t3_ns) {
	int64_t rtti = local_t3_ns - local_t1_ns;

	// assume rtti is evenly split both directions
	int64_t remote_t3_ns = remote_t2_ns + rtti / 2ll;

	int64_t measurement_offset = remote_t3_ns - local_t3_ns;

	if (_request_reset_counter > REQUEST_RESET_COUNTER_THRESHOLD) {
		reset();
		std::cout << std::endl << "Timesync clock changed, resetting" << std::endl;
	}

	if (_num_samples >= WINDOW_SIZE) {
		if (std::abs(measurement_offset - _offset_ns) > TRIGGER_RESET_THRESHOLD_NS) {
			_request_reset_counter++;
			std::cout << std::endl << "Timesync offset outlier, discarding" << std::endl;
			return false;
		} else {
			_request_reset_counter = 0;
		}
	}

	if (_num_samples == 0) {
		_offset_ns = measurement_offset;
		_skew_ns_per_sync = 0;
	}

	{
		int64_t local_t2 = remote_t2_ns - _offset_ns;
		int64_t time_there = local_t2 - local_t1_ns;

		int64_t remote_t3 = local_t3_ns + _offset_ns;
		int64_t time_back = remote_t3 - remote_t2_ns;

		if (std::abs(time_back - time_there) > 3ll * 1000ll * 1000ll) {
			std::cout << "trip there: " << time_there / 1e6f << "ms  trip back: " << time_back / 1e6f
				  << "ms , discarding" << std::endl;
			return false;
		}
	}

	// ignore if rtti > 10ms
	if (rtti > 15ll * 1000ll * 1000ll) {
		std::cout << std::endl
			  << "RTTI too high for timesync: " << rtti / (1000ll * 1000ll) << "ms" << std::endl;
		return false;
	}

	double schedule = std::min((double)_num_samples / WINDOW_SIZE, 1.);
	double alpha = ALPHA_INITIAL * (1. - schedule) + ALPHA_FINAL * schedule;
	double beta = BETA_INTIIAL * (1. - schedule) + BETA_FINAL * schedule;

	int64_t offset_prev = _offset_ns;
	_offset_ns = static_cast<int64_t>((_skew_ns_per_sync + _offset_ns) * (1. - alpha) + measurement_offset * alpha);
	_skew_ns_per_sync = static_cast<int64_t>(beta * (_offset_ns - offset_prev) + (1. - beta) * _skew_ns_per_sync);

	_num_samples++;

	return true;
}

void TimeSync::processTimesyncMsg(const px4_msgs::msg::Timesync* msg) {
	if (msg->seq() == _last_msg_seq) return;
	_last_msg_seq = msg->seq();

	int64_t now_time = getSystemMonoNanos();

	if (msg->tc1() == 0) {
		px4_msgs::msg::Timesync reply = (*msg);
		reply.timestamp() = getSystemTime();
		reply.seq() = _last_msg_seq;
		reply.tc1() = now_time;

		_Timesync_pub.publish(&reply);

	} else if (msg->tc1() > 0) {
		bool added = addMeasurement(msg->ts1(), msg->tc1(), now_time);
		if (added && _notifyNewOffset) _notifyNewOffset(_offset_ns);
	}
}

px4_msgs::msg::Timesync TimeSync::newTimesyncMsg() {
	px4_msgs::msg::Timesync msg{};
	msg.timestamp() = getSystemTime();
	msg.seq() = _last_msg_seq;
	msg.tc1() = 0;
	msg.ts1() = getSystemMonoNanos();

	_last_msg_seq++;

	return msg;
}
