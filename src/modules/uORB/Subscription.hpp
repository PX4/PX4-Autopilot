/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file Subscription.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <px4_defines.h>

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

namespace uORB
{

class SubscriptionCallback;

// Base subscription wrapper class
class Subscription
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	Subscription(const orb_metadata *meta, uint8_t instance = 0) : _meta(meta), _instance(instance)
	{
		subscribe();
	}

	~Subscription()
	{
		unsubscribe();
	}

	bool subscribe();
	void unsubscribe();

	bool valid() const { return _node != nullptr; }
	bool published()
	{
		if (valid()) {
			return _node->is_published();
		}

		// try to initialize
		if (init()) {
			// check again if valid
			if (valid()) {
				return _node->is_published();
			}
		}

		return false;
	}

	/**
	 * Check if there is a new update.
	 * */
	bool updated() { return published() ? (_node->published_message_count() != _last_generation) : false; }

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool update(void *dst) { return updated() ? copy(dst) : false; }

	/**
	 * Check if subscription updated based on timestamp.
	 *
	 * @return true only if topic was updated based on a timestamp and
	 * copied to buffer successfully.
	 * If topic was not updated since last check it will return false but
	 * still copy the data.
	 * If no data available data buffer will be filled with zeros.
	 */
	bool update(uint64_t *time, void *dst);

	/**
	 * Copy the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool copy(void *dst) { return published() ? _node->copy(dst, _last_generation) : false; }

	uint8_t		get_instance() const { return _instance; }
	orb_id_t	get_topic() const { return _meta; }

protected:

	friend class SubscriptionCallback;

	DeviceNode		*get_node() { return _node; }

	bool			init();

	DeviceNode		*_node{nullptr};
	const orb_metadata	*_meta{nullptr};

	/**
	 * Subscription's latest data generation.
	 * Also used to track (and rate limit) subscription
	 * attempts if the topic has not yet been published.
	 */
	unsigned		_last_generation{0};
	uint8_t			_instance{0};
};

// Subscription wrapper class with data
template<class T>
class SubscriptionData : public Subscription
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionData(const orb_metadata *meta, uint8_t instance = 0) :
		Subscription(meta, instance)
	{
		copy(&_data);
	}

	~SubscriptionData() = default;

	// no copy, assignment, move, move assignment
	SubscriptionData(const SubscriptionData &) = delete;
	SubscriptionData &operator=(const SubscriptionData &) = delete;
	SubscriptionData(SubscriptionData &&) = delete;
	SubscriptionData &operator=(SubscriptionData &&) = delete;

	// update the embedded struct.
	bool update() { return Subscription::update((void *)(&_data)); }

	const T &get() const { return _data; }

private:

	T _data{};
};

} // namespace uORB
