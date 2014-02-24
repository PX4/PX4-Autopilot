/*
 * mavlink_stream.h
 *
 *  Created on: 24.02.2014
 *      Author: ton
 */

#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

class Mavlink;
class MavlinkOrbListener;
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
	MavlinkOrbListener* listener;

	MavlinkStream(MavlinkOrbListener *listener, void (*callback)(const MavlinkStream *), const unsigned int subs_n, const struct orb_metadata **metas, const size_t *sizes, const uintptr_t arg, const unsigned int interval);
	~MavlinkStream();
	int update(const hrt_abstime t);
};


#endif /* MAVLINK_STREAM_H_ */
