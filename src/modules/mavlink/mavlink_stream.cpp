/*
 * mavlink_stream.cpp
 *
 *  Created on: 24.02.2014
 *      Author: ton
 */

#include <stdlib.h>

#include "mavlink_orb_listener.h"

MavlinkStream::MavlinkStream(MavlinkOrbListener *listener, void (*callback)(const MavlinkStream *), const unsigned int subs_n, const struct orb_metadata **metas, const size_t *sizes, const uintptr_t arg, const unsigned int interval)
{
	this->callback = callback;
	this->arg = arg;
	this->interval = interval * 1000;
	this->mavlink = mavlink;
	this->listener = listener;
	this->subscriptions_n = subs_n;
	this->subscriptions = (MavlinkOrbSubscription **) malloc(subs_n * sizeof(MavlinkOrbSubscription *));

	for (int i = 0; i < subs_n; i++) {
		this->subscriptions[i] = listener->add_subscription(metas[i], sizes[i], this, interval);
	}
}

MavlinkStream::~MavlinkStream()
{
	free(subscriptions);
}

/**
 * Update mavlink stream, i.e. update subscriptions and send message if necessary
 */
int MavlinkStream::update(const hrt_abstime t)
{
	uint64_t dt = t - last_sent;
	if (dt > 0 && dt >= interval) {
		/* interval expired, update all subscriptions */
		for (unsigned int i = 0; i < subscriptions_n; i++) {
			subscriptions[i]->update(t);
		}

		/* format and send mavlink message */
		callback(this);
		last_sent = t;
	}
}
