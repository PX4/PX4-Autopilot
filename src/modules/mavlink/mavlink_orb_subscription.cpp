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

MavlinkOrbSubscription::MavlinkOrbSubscription(const struct orb_metadata *meta, size_t size)
{
	this->meta = meta;
	this->data = malloc(size);
	memset(this->data, 0, size);
	this->fd = orb_subscribe(meta);
	this->last_update = 0;
	this->interval = 0;
}

MavlinkOrbSubscription::~MavlinkOrbSubscription()
{
	close(fd);
	free(data);
}

int MavlinkOrbSubscription::set_interval(const unsigned int interval)
{
	this->interval = interval;
	return orb_set_interval(fd, interval);
}

int MavlinkOrbSubscription::update(const hrt_abstime t)
{
	if (last_update != t) {
		bool updated;
		orb_check(fd, &updated);
		if (updated)
			return orb_copy(meta, fd, data);
	}
	return OK;
}
