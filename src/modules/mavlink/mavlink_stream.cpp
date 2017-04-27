/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 */

#include <stdlib.h>

#include "mavlink_stream.h"
#include "mavlink_main.h"

MavlinkStream::MavlinkStream(Mavlink *mavlink) :
	next(nullptr),
	_mavlink(mavlink),
	_interval(1000000),
	_last_sent(0 /* 0 means unlimited - updates on every iteration */)
{
}

MavlinkStream::~MavlinkStream()
{
}

/**
 * Set messages interval in ms
 */
void
MavlinkStream::set_interval(const unsigned int interval)
{
	_interval = interval;
}

/**
 * Update subscriptions and send message if necessary
 */
int
MavlinkStream::update(const hrt_abstime t)
{
	// If the message has never been sent before we want
	// to send it immediately and can return right away
	if (_last_sent == 0) {
		// this will give different messages on the same run a different
		// initial timestamp which will help spacing them out
		// on the link scheduling
		_last_sent = hrt_absolute_time();
#ifndef __PX4_QURT
		send(t);
#endif
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

	// send the message if it is due or
	// if it will overrun the next scheduled send interval
	// by 40% of the interval time. This helps to avoid
	// sending a scheduled message on average slower than
	// scheduled. Doing this at 50% would risk sending
	// the message too often as the loop runtime of the app
	// needs to be accounted for as well.
	// This method is not theoretically optimal but a suitable
	// stopgap as it hits its deadlines well (0.5 Hz, 50 Hz and 250 Hz)
	int64_t update_dt = (interval - (_mavlink->get_main_loop_delay() / 10) * 4);

	if (dt > update_dt) {
		// interval expired, send message
#ifndef __PX4_QURT
		send(t);
#endif
		// if the interval is non-zero do not use the actual time but
		// increment at a fixed rate, so that processing delays do not
		// distort the average rate
		_last_sent = (interval > 0) ? _last_sent + interval : t;

		return 0;
	}

	return -1;
}
