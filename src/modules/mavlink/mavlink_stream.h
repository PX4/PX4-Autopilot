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

class MavlinkStream {
private:
	hrt_abstime _last_sent;

protected:
	mavlink_channel_t _channel;
	unsigned int _interval;

	virtual void send(const hrt_abstime t) = 0;

public:
	MavlinkStream *next;

	MavlinkStream();
	~MavlinkStream();
	void set_interval(const unsigned int interval);
	void set_channel(mavlink_channel_t channel);
	int update(const hrt_abstime t);
	virtual MavlinkStream *new_instance() = 0;
	virtual void subscribe(Mavlink *mavlink) = 0;
	virtual const char *get_name() = 0;
};


#endif /* MAVLINK_STREAM_H_ */
