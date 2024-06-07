/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_stream.cpp
 * Mavlink messages stream implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdlib.h>

#include "mavlink_stream.h"
#include "mavlink_main.h"

/**
 * If stream rate is set to unlimited, set the rate to 50 Hz. To get higher
 * rates, it needs to be set explicitly.
 */

const uint32_t MavlinkStreamUnlimitedInterval = 20000;

MavlinkStream::MavlinkStream(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	_last_sent = hrt_absolute_time();
}

/**
 * Update subscriptions and send message if necessary
 */
int
MavlinkStream::update(const hrt_abstime &t)
{
	update_data();

	// If the message has never been sent before we want
	// to send it immediately and can return right away
	if (_last_sent == 0) {
		// this will give different messages on the same run a different
		// initial timestamp which will help spacing them out
		// on the link scheduling
		if (send()) {
			_last_sent = hrt_absolute_time();

			if (!_first_message_sent) {
				_first_message_sent = true;
			}
		}

		return 0;
	}

	// One of the previous iterations sent the update
	// already before the deadline
	if (_last_sent > t) {
		return -1;
	}

	int64_t dt = t - _last_sent;
	int interval = _interval;

	if (!const_rate()) {
		interval /= _mavlink->get_rate_mult();
	}

	// We don't need to send anything if the inverval is 0. send() will be called manually.
	if (interval == 0) {
		return 0;
	}

	const bool unlimited_rate = interval < 0;

	// Send the message if it is due or
	// if it will overrun the next scheduled send interval
	// by 30% of the interval time. This helps to avoid
	// sending a scheduled message on average slower than
	// scheduled. Doing this at 50% would risk sending
	// the message too often as the loop runtime of the app
	// needs to be accounted for as well.
	// This method is not theoretically optimal but a suitable
	// stopgap as it hits its deadlines well (0.5 Hz, 50 Hz and 250 Hz)

	if (unlimited_rate || (dt > (interval - (_mavlink->get_main_loop_delay() / 10) * 3))) {
		// interval expired, send message

		// If the interval is non-zero and dt is smaller than 1.5 times the interval
		// do not use the actual time but increment at a fixed rate, so that processing delays do not
		// distort the average rate. The check of the maximum interval is done to ensure that after a
		// long time not sending anything, sending multiple messages in a short time is avoided.
		if (send()) {
			_last_sent = ((interval > 0) && ((int64_t)(1.5f * interval) > dt)) ? _last_sent + interval : t;

			if (!_first_message_sent) {
				_first_message_sent = true;
			}

			return 0;

		} else {
			return -1;
		}
	}

	return -1;
}

MavlinkStreamPoll::MavlinkStreamPoll()
{
	px4_sem_init(&_poll_sem, 1, 0);
#if defined(__PX4_NUTTX)
	sem_setprotocol(&_poll_sem, SEM_PRIO_NONE);
#endif

	pthread_mutex_init(&_mtx, nullptr);
}

MavlinkStreamPoll::~MavlinkStreamPoll()
{
	// This removes and deletes every object in the list
	_reqs.clear();

	px4_sem_destroy(&_poll_sem);
	pthread_mutex_destroy(&_mtx);
}

void MavlinkStreamPoll::recalculate_roots_and_start(MavStreamPollReq *req)
{
	// Now go through the ordered _reqs list, to see if this timer needs to
	// be started, and if some others can be stopped

	bool is_root = true;
	uint32_t interval_us = req->interval_us();

	for (auto r : _reqs) {
		if (r->interval_us() <= interval_us) {
			if (r->is_root() && interval_us % r->interval_us() == 0) {
				is_root = false;
			}

		} else {
			if (is_root && r->is_root() && r->interval_us() % interval_us == 0) {
				r->stop_interval();
			}
		}
	}

	// If this was a new root interval, start the hrt

	if (is_root) {
		req->start_interval(hrt_callback, &_poll_sem);
	}
}

int
MavlinkStreamPoll::register_poll(uint16_t stream_id, uint32_t interval_us)
{
	// Streans with interval 0 are disabled and don't need to be registered here

	if (interval_us == 0) {
		return OK;
	}

	MavStreamPollReq *req = new MavStreamPollReq(stream_id, interval_us);

	if (req == nullptr) {
		return -ENOMEM;
	}

	pthread_mutex_lock(&_mtx);

	recalculate_roots_and_start(req);
	_reqs.add_sorted(req);


	pthread_mutex_unlock(&_mtx);
	return OK;
}

int
MavlinkStreamPoll::unregister_poll(uint16_t stream_id)
{
	pthread_mutex_lock(&_mtx);

	for (auto req : _reqs) {
		if (req->stream_id() == stream_id) {
			_reqs.remove(req);

			if (req->is_root()) {
				// This interval may be driving other streams. Re-calculate root clocks for all the
				// remaining requests

				for (auto r : _reqs) {
					recalculate_roots_and_start(r);
				}

				req->stop_interval();
			}

			delete (req);
			break;
		}
	}

	pthread_mutex_unlock(&_mtx);

	return OK;
}

int
MavlinkStreamPoll::set_interval(uint16_t stream_id, int interval_us)
{
	unregister_poll(stream_id);

	if (interval_us < 0) {
		interval_us = MavlinkStreamUnlimitedInterval;
	}

	return register_poll(stream_id, interval_us);
}

/**
 * Perform orb polling
 */
int
MavlinkStreamPoll::poll(const hrt_abstime timeout_us)
{
	int ret;

	// Wait event for a maximum timeout time

	struct timespec to;
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	px4_clock_gettime(CLOCK_MONOTONIC, &to);
#else
	px4_clock_gettime(CLOCK_REALTIME, &to);
#endif
	hrt_abstime now = ts_to_abstime(&to);
	abstime_to_ts(&to, now + timeout_us);

	ret = px4_sem_timedwait(&_poll_sem, &to);

	if (ret < 0) {
		ret = -errno;
	}

	return ret;
}

void
MavlinkStreamPoll::hrt_callback(void *arg)
{
	px4_sem_post((px4_sem_t *)arg);
}

