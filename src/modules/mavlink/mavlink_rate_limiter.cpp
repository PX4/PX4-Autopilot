/*
 * mavlink_rate_limiter.cpp
 *
 *  Created on: 26.02.2014
 *      Author: ton
 */


#include "mavlink_rate_limiter.h"

MavlinkRateLimiter::MavlinkRateLimiter() : _last_sent(0), _interval(1000000)
{
}

MavlinkRateLimiter::MavlinkRateLimiter(unsigned int interval) : _last_sent(0), _interval(interval * 1000)
{
}

MavlinkRateLimiter::~MavlinkRateLimiter()
{
}

void
MavlinkRateLimiter::set_interval(unsigned int interval)
{
	_interval = interval * 1000;
}

bool
MavlinkRateLimiter::check(hrt_abstime t)
{
	uint64_t dt = t - _last_sent;
	if (dt > 0 && dt >= _interval) {
		_last_sent = (t / _interval) * _interval;
		return true;
	}
	return false;
}
