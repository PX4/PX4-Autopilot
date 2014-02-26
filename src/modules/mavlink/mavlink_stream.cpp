/*
 * mavlink_stream.cpp
 *
 *  Created on: 24.02.2014
 *      Author: ton
 */

#include <stdlib.h>

#include "mavlink_stream.h"
#include "mavlink_main.h"

MavlinkStream::MavlinkStream() : _interval(1000), _last_sent(0), _channel(MAVLINK_COMM_0), next(nullptr)
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
	_interval = interval * 1000;
}

/**
 * Set mavlink channel
 */
void
MavlinkStream::set_channel(mavlink_channel_t channel)
{
	_channel = channel;
}

/**
 * Update subscriptions and send message if necessary
 */
int
MavlinkStream::update(const hrt_abstime t)
{
	uint64_t dt = t - _last_sent;
	if (dt > 0 && dt >= _interval) {
		/* interval expired, send message */
		send(t);
		_last_sent = (t / _interval) * _interval;
	}
}
