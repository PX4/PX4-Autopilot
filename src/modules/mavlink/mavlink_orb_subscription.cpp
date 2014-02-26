/*
 * mavlink_orb_subscription.cpp
 *
 *  Created on: 23.02.2014
 *      Author: ton
 */


#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <uORB/uORB.h>

#include "mavlink_orb_subscription.h"

MavlinkOrbSubscription::MavlinkOrbSubscription(const struct orb_metadata *topic, size_t size) : _topic(topic), _last_check(0), next(nullptr)
{
	_data = malloc(size);
	memset(_data, 0, size);
	_fd = orb_subscribe(_topic);
}

MavlinkOrbSubscription::~MavlinkOrbSubscription()
{
	close(_fd);
	free(_data);
}

const struct orb_metadata *
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
	if (_last_check != t) {
		_last_check = t;
		bool updated;
		orb_check(_fd, &updated);
		if (updated) {
			orb_copy(_topic, _fd, _data);
			return true;
		}
	}
	return false;
}
