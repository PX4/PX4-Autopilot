/*
 * mavlink_stream.h
 *
 *  Created on: 24.02.2014
 *      Author: ton
 */

#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

#include <drivers/drv_hrt.h>

class Mavlink;
class MavlinkStream;

#include "mavlink_main.h"

class MavlinkOrbSubscription;

class MavlinkStream {
public:
	void	(*callback)(const MavlinkStream *);
	uintptr_t	arg;
	unsigned int subscriptions_n;
	MavlinkOrbSubscription **subscriptions;
	hrt_abstime last_sent;
	unsigned int interval;
	MavlinkStream *next;
	Mavlink		*mavlink;

	MavlinkStream(Mavlink *mavlink, void (*callback)(const MavlinkStream *), const unsigned int subs_n, const struct orb_metadata **topics, const size_t *sizes, const uintptr_t arg, const unsigned int interval);
	~MavlinkStream();
	int update(const hrt_abstime t);
};


#endif /* MAVLINK_STREAM_H_ */
