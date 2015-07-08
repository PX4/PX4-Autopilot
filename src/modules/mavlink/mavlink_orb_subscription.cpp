/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <uORB/uORB.h>
#include <stdio.h>

#include "mavlink_orb_subscription.h"

MavlinkOrbSubscription::MavlinkOrbSubscription(const orb_id_t topic, int instance) :
	next(nullptr),
	_topic(topic),
	_instance(instance),
	_fd(orb_subscribe_multi(_topic, instance)),
	_published(false)
{
}

MavlinkOrbSubscription::~MavlinkOrbSubscription()
{
	close(_fd);
}

orb_id_t
MavlinkOrbSubscription::get_topic() const
{
	return _topic;
}

int
MavlinkOrbSubscription::get_instance() const
{
	return _instance;
}

bool
MavlinkOrbSubscription::update(uint64_t *time, void* data)
{
	// TODO this is NOT atomic operation, we can get data newer than time
	// if topic was published between orb_stat and orb_copy calls.

	uint64_t time_topic;
	if (orb_stat(_fd, &time_topic)) {
		/* error getting last topic publication time */
		time_topic = 0;
	}

	if (orb_copy(_topic, _fd, data)) {
		/* error copying topic data */
		memset(data, 0, _topic->o_size);
		return false;

	} else {
		/* data copied successfully */
		_published = true;
		if (time_topic != *time) {
			*time = time_topic;
			return true;

		} else {
			return false;
		}
	}
}

bool
MavlinkOrbSubscription::update(void* data)
{
	return !orb_copy(_topic, _fd, data);
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
