/*
 * mavlink_orb_subscription.h
 *
 *  Created on: 23.02.2014
 *      Author: ton
 */

#ifndef MAVLINK_ORB_SUBSCRIPTION_H_
#define MAVLINK_ORB_SUBSCRIPTION_H_

#include <systemlib/uthash/utlist.h>
#include <drivers/drv_hrt.h>


class MavlinkOrbSubscription {
public:
	MavlinkOrbSubscription *next;

	MavlinkOrbSubscription(const orb_id_t topic, size_t size);
	~MavlinkOrbSubscription();

	bool update(const hrt_abstime t);
	void *get_data();
	const struct orb_metadata *get_topic();

private:
	const orb_id_t _topic;
	int _fd;
	void *_data;
	hrt_abstime _last_check;
};


#endif /* MAVLINK_ORB_SUBSCRIPTION_H_ */
