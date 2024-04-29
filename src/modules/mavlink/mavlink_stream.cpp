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

#if defined(CONFIG_MAVLINK_UORB_POLL)
// 20ms default update rate for polled topics, if not set otherwise
const unsigned int MavlinkStreamDefaultTopicInterval = 20;

MavlinkStreamPoll::MavlinkStreamPoll() :
	_fds{nullptr},
	_orbs{nullptr},
	_reqs{nullptr},
	_reqs_capacity{16},
	_reqs_count{0},
	_capacity{16},
	_count{0}
{
	_orbs = (MavStreamPollItem *)malloc(16 * sizeof(MavStreamPollItem));
	_fds = (orb_poll_struct_t *)malloc(16 * sizeof(orb_poll_struct_t));
	_reqs = (MavStreamOrbPollReq *)malloc(16 * sizeof(MavStreamOrbPollReq));
	pthread_mutex_init(&_mtx, nullptr);
}

MavlinkStreamPoll::~MavlinkStreamPoll()
{
	if (_fds) {
		free(_fds);
	}

	if (_orbs) {
		free(_orbs);
	}

	if (_reqs) {
		free(_reqs);
	}

	pthread_mutex_destroy(&_mtx);
}

int
MavlinkStreamPoll::_add_orb(ORB_ID orb_id, int interval_ms)
{
	// Check if the orb is already in the list
	bool exists = false;

	for (int i = 0; i < _count; i++) {
		if (_orbs[i].orb_id == orb_id && _orbs[i].interval == interval_ms) {
			_orbs[i].usage_count++;
			exists = true;
			break;
		}
	}

	// Did not exist, add new one to the list
	if (!exists) {
		while (_count >= _capacity) {
			_capacity = _capacity + 16;
			_orbs = (MavStreamPollItem *)realloc(_orbs, _capacity * sizeof(MavStreamPollItem));
			_fds = (orb_poll_struct_t *)realloc(_fds, _capacity * sizeof(orb_poll_struct_t));

			if (!_fds || !_orbs) {
				PX4_ERR("failed to allocate memory for orb poll items");
				return PX4_ERROR;
			}
		}

		_orbs[_count].orb_id = orb_id;
		_orbs[_count].interval = interval_ms;
		_orbs[_count].usage_count = 1;
		_orbs[_count].fd = orb_subscribe(get_orb_meta(orb_id));
		orb_set_interval(_orbs[_count].fd, interval_ms);
		_count++;
	}

	return PX4_OK;
}

int
MavlinkStreamPoll::_remove_orb(ORB_ID orb_id, int interval_ms)
{
	// Decrement usage/remove the item from the orbs list
	for (int i = 0; i < _count; i++) {
		if (_orbs[i].orb_id == orb_id &&
		    _orbs[i].interval == interval_ms) {
			_orbs[i].usage_count--;

			// If this was the last request for the orb,
			// unsubscribe and remove from orbs list
			if (_orbs[i].usage_count == 0) {
				orb_unsubscribe(_orbs[i].fd);
				// Replace the current item with
				// the last item in the _orbs list.
				// Loop counting is not disturbed because
				// we break out from loop after this.
				_orbs[i] = _orbs[--_count];
			}

			break;
		}
	}

	return PX4_OK;
}

/**
 * Register stream object to poll list of orbs
 */
int
MavlinkStreamPoll::register_orbs(uint16_t stream_id, ORB_ID *orbs, int cnt)
{
	if (!orbs || cnt <= 0) {
		return PX4_OK;
	}

	pthread_mutex_lock(&_mtx);

	for (int i = 0; i < cnt; i++) {

		// Increase reqs capacity if necessary
		while (_reqs_count >= _reqs_capacity) {
			_reqs_capacity = _reqs_capacity + 16;
			_reqs = (MavStreamOrbPollReq *)realloc(_reqs, _reqs_capacity * sizeof(MavStreamOrbPollReq));

			if (!_reqs) {
				PX4_ERR("failed to allocate memory for orb poll reqs");
				pthread_mutex_unlock(&_mtx);
				return PX4_ERROR;
			}
		}

		MavStreamOrbPollReq *req = &_reqs[_reqs_count];
		req->stream_id = stream_id;
		req->orb_id = orbs[i];
		req->interval = MavlinkStreamDefaultTopicInterval;
		_reqs_count++;

		// Update orbs list
		_add_orb(orbs[i], req->interval);
	}

	// Update fds
	for (int i = 0; i < _count; i++) {
		_fds[i].fd = _orbs[i].fd;
		_fds[i].events = POLLIN;
		_fds[i].revents = 0;
	}

	pthread_mutex_unlock(&_mtx);
	return PX4_OK;
}

/**
 * Unregister stream object from orbs polling
 */
int
MavlinkStreamPoll::unregister_orbs(uint16_t stream_id)
{
	int i = 0;

	pthread_mutex_lock(&_mtx);

	while (i < _reqs_count) {
		if (_reqs[i].stream_id == stream_id) {

			// Remove orb from the orbs list
			_remove_orb(_reqs[i].orb_id, _reqs[i].interval);

			// Finally, replace the current item with
			// the last item in the _reqs list
			_reqs[i] = _reqs[--_reqs_count];

		} else {
			// Only increment if current _reqs item is not removed.
			// Otherwise we have moved the last item to the current
			// position, so we need to check the same index again.
			i++;
		}
	}

	// Update fds
	for (int j = 0; j < _count; j++) {
		_fds[j].fd = _orbs[j].fd;
		_fds[j].events = POLLIN;
		_fds[j].revents = 0;
	}

	pthread_mutex_unlock(&_mtx);
	return PX4_OK;
}

/**
 * Set stream update interval to adjust orb polling accordingly
 */
int
MavlinkStreamPoll::set_interval(uint16_t stream_id, int interval_ms)
{
	pthread_mutex_lock(&_mtx);

	// Update interval for all orb subscriptions and find the maximum interval
	for (int i = 0; i < _count; i++) {
		if (_reqs[i].stream_id == stream_id) {

			// Remove orb_id subscription with current old interval
			_remove_orb(_reqs[i].orb_id, _reqs[i].interval);

			// Update interval
			_reqs[i].interval = interval_ms;

			// Add orb_id subscription with new interval
			_add_orb(_reqs[i].orb_id, _reqs[i].interval);

		}
	}

	pthread_mutex_unlock(&_mtx);
	return PX4_OK;
}

/**
 * Perform orb polling
 */
int
MavlinkStreamPoll::poll(const hrt_abstime timeout)
{
	int timeout_ms = timeout / 1000;

	if (timeout_ms <= 0) {
		timeout_ms = 1;
	}

	return px4_poll(_fds, _count, timeout_ms);
}

/**
 * Acknowledge all orb data for next poll
 */
void
MavlinkStreamPoll::ack_all()
{
	for (int i = 0; i < _count; i++) {
		orb_ack(_orbs[i].fd);
	}
}
#endif /* CONFIG_MAVLINK_UORB_POLL */
