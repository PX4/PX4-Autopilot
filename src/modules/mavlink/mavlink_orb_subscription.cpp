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
 * @file mavlink_orb_subscription.cpp
 * uORB subscription implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <uORB/uORB.h>
#include <stdio.h>

#include "mavlink_orb_subscription.h"

MavlinkOrbSubscription::MavlinkOrbSubscription(const orb_id_t topic) :
	_fd(orb_subscribe(_topic)),
	_published(false),
	_topic(topic),
	_last_check(0),
	next(nullptr)
{
	_data = malloc(topic->o_size);
	memset(_data, 0, topic->o_size);
}

MavlinkOrbSubscription::~MavlinkOrbSubscription()
{
	close(_fd);
	free(_data);
}

const orb_id_t
MavlinkOrbSubscription::get_topic()
{
	return _topic;
}

void *
MavlinkOrbSubscription::get_data()
{
	return _data;
}

bool
MavlinkOrbSubscription::update(const hrt_abstime t)
{
	if (_last_check == t) {
		/* already checked right now, return result of the check */
		return _updated;

	} else {
		_last_check = t;
		orb_check(_fd, &_updated);

		if (_updated) {
			orb_copy(_topic, _fd, _data);
			_published = true;
			return true;
		}
	}

	return false;
}

bool
MavlinkOrbSubscription::is_published()
{
	if (_published) {
		return true;
	}

	bool updated;
	orb_check(_fd, &updated);

	if (updated) {
		_published = true;
	}

	return _published;
}
