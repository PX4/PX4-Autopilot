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

MavlinkOrbSubscription::MavlinkOrbSubscription(const struct orb_metadata *topic, size_t size)
{
	this->topic = topic;
	this->data = malloc(size);
	memset(this->data, 0, size);
	this->fd = orb_subscribe(topic);
	this->last_update = 0;
}

MavlinkOrbSubscription::~MavlinkOrbSubscription()
{
	close(fd);
	free(data);
}

bool MavlinkOrbSubscription::update(const hrt_abstime t)
{
	if (last_update != t) {
		bool updated;
		orb_check(fd, &updated);
		if (updated) {
			orb_copy(topic, fd, data);
			return true;
		}
	}
	return false;
}
