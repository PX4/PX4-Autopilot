/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_orb_subscription.h
 * uORB subscription definition.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef MAVLINK_ORB_SUBSCRIPTION_H_
#define MAVLINK_ORB_SUBSCRIPTION_H_

#include <systemlib/uthash/utlist.h>
#include <drivers/drv_hrt.h>


class MavlinkOrbSubscription
{
public:
	MavlinkOrbSubscription *next;	///< pointer to next subscription in list

	MavlinkOrbSubscription(const orb_id_t topic);
	~MavlinkOrbSubscription();

	/**
	 * Check if subscription updated and get data.
	 *
	 * @return true only if topic was updated and data copied to buffer successfully.
	 * If topic was not updated since last check it will return false but still copy the data.
	 * If no data available data buffer will be filled with zeroes.
	 */
	bool update(uint64_t *time, void* data);

	/**
	 * Copy topic data to given buffer.
	 *
	 * @return true only if topic data copied successfully.
	 */
	bool update(void* data);

	/**
	 * Check if the topic has been published.
	 *
	 * This call will return true if the topic was ever published.
	 * @return true if the topic has been published at least once.
	 */
	bool is_published();
	orb_id_t get_topic() const;

private:
	const orb_id_t _topic;		///< topic metadata
	int _fd;			///< subscription handle
	bool _published;		///< topic was ever published
};


#endif /* MAVLINK_ORB_SUBSCRIPTION_H_ */
