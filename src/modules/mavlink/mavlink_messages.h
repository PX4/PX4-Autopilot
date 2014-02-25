/*
 * mavlink_messages.h
 *
 *  Created on: 25.02.2014
 *      Author: ton
 */

#ifndef MAVLINK_MESSAGES_H_
#define MAVLINK_MESSAGES_H_

#include "mavlink_stream.h"

#define MAX_TOPICS_PER_MAVLINK_STREAM 4

struct msgs_list_s {
	char *name;
	void (*callback)(const MavlinkStream *);
	const struct orb_metadata *topics[MAX_TOPICS_PER_MAVLINK_STREAM+1];
	size_t sizes[MAX_TOPICS_PER_MAVLINK_STREAM+1];
};

extern struct msgs_list_s msgs_list[];

static void msg_heartbeat(const MavlinkStream *stream);
static void msg_sys_status(const MavlinkStream *stream);


#endif /* MAVLINK_MESSAGES_H_ */
