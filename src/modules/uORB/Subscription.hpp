/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
#include <containers/List.hpp>
#include <systemlib/err.h>
#include <px4_defines.h>

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

namespace uORB
{

/**
 * Base subscription wrapper class, used in list traversal
 * of various subscriptions.
 */
class SubscriptionBase
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionBase() = default;
	SubscriptionBase(const orb_metadata *meta, uint8_t instance = 0);
	virtual ~SubscriptionBase();

	bool valid() const { return _node != nullptr; }

	bool init();
	bool forceInit();

	virtual bool updated() { return published() ? (_node->published_message_count() != _last_generation) : false; }

	virtual bool update(void *dst) { return updated() ? copy(dst) : false; }

	/**
	 * Check if subscription updated based on timestamp.
	 *
	 * @return true only if topic was updated based on a timestamp and
	 * copied to buffer successfully.
	 * If topic was not updated since last check it will return false but
	 * still copy the data.
	 * If no data available data buffer will be filled with zeros.
	 */
	virtual bool update(uint64_t *time, void *dst);

	bool copy(void *dst) { return published() ? _node->copy(dst, _last_generation) : false; }


	hrt_abstime	last_update() { return published() ? _node->last_update() : 0; }
	bool		published() { return (_node != nullptr) ? _node->is_published() : init(); }

	bool		set_instance(uint8_t instance);
	uint8_t		get_instance() const { return _instance; }

	bool		set_topic(orb_metadata *meta);
	orb_id_t	get_topic() const { return _meta; }

protected:

	bool subscribe();
	void unsubscribe();

	DeviceNode		*_node{nullptr};
	unsigned		_last_generation{0};
	const orb_metadata		*_meta{nullptr};
	uint8_t			_instance{0};
};

// TODO: finish
class SubscriptionInterval : public SubscriptionBase
{
public:

	SubscriptionInterval() = default;

	SubscriptionInterval(const orb_metadata *meta, unsigned interval = 0, uint8_t instance = 0) :
		SubscriptionBase(meta, instance),
		_interval(interval)
	{}

	virtual ~SubscriptionInterval() = default;

	bool updated() override
	{
		if (hrt_absolute_time() >= (_last_update + (_interval * 1000))) {
			return SubscriptionBase::updated();
		}

		return false;
	}

	bool update(void *dst) override
	{
		if (updated()) {
			if (copy(dst)) {
				_last_update = hrt_absolute_time();
				return true;
			}
		}

		return false;
	}

	int get_interval() const { return _interval; }
	void set_interval(unsigned interval) { _interval = interval; }

protected:

	unsigned _interval{0}; // interval in milliseconds

	uint64_t	_last_update{0};

};

/**
 * alias class name so it is clear that the base class
 */
typedef SubscriptionBase SubscriptionTiny;

/**
 * The subscription base class as a list node.
 */
class  SubscriptionNode : public SubscriptionInterval, public ListNode<SubscriptionNode *>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param instance The instance for multi sub.
	 * @param list 	A pointer to a list of subscriptions
	 * 	that this should be appended to.
	 */
	SubscriptionNode(const orb_metadata *meta, unsigned interval = 0, uint8_t instance = 0,
			 List<SubscriptionNode *> *list = nullptr);

	virtual ~SubscriptionNode() override = default;

	/**
	 * This function is the callback for list traversal
	 * updates, a child class must implement it.
	 */
	virtual bool update() = 0;

	/**
	 * Like update(), but does not check first if there is data available
	 */
	virtual bool forcedUpdate() = 0;

};

/**
 * Subscription wrapper class
 */
template<class T>
class Subscription final : public SubscriptionNode
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param list A list interface for adding to
	 * 	list during construction
	 */
	Subscription(const orb_metadata *meta, unsigned interval = 0, uint8_t instance = 0,
		     List<SubscriptionNode *> *list = nullptr):
		SubscriptionNode(meta, interval, instance, list)
	{
		forcedUpdate();
	}

	~Subscription() override final = default;

	// no copy, assignment, move, move assignment
	Subscription(const Subscription &) = delete;
	Subscription &operator=(const Subscription &) = delete;
	Subscription(Subscription &&) = delete;
	Subscription &operator=(Subscription &&) = delete;

	/**
	 * Create an update function that uses the embedded struct.
	 */
	bool update() override final { return SubscriptionBase::update((void *)(&_data)); }

	bool forcedUpdate() override final { return copy(&_data); }

	/*
	 * This function gets the T struct data
	 * */
	const T &get() const { return _data; }

private:

	T _data{};
};




// LEGACY helper
// TODO: remove entirely
template<class T>
class SubscriptionPolled
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param instance The instance for multi sub.
	 */
	SubscriptionPolled(const orb_metadata *meta, unsigned interval = 0, uint8_t instance = 0) :
		_meta(meta),
		_instance(instance)
	{
		if (instance > 0) {
			_handle = orb_subscribe_multi(_meta, instance);

		} else {
			_handle = orb_subscribe(_meta);
		}

		if (_handle < 0) {
			PX4_ERR("%s sub failed", _meta->o_name);
		}

		if (interval > 0) {
			orb_set_interval(_handle, interval);
		}

		forcedUpdate();
	}

	virtual ~SubscriptionPolled()
	{
		if (orb_unsubscribe(_handle) != PX4_OK) {
			PX4_ERR("%s unsubscribe failed", _meta->o_name);
		}
	}

	// no copy, assignment, move, move assignment
	SubscriptionPolled(const SubscriptionBase &) = delete;
	SubscriptionPolled &operator=(const SubscriptionBase &) = delete;
	SubscriptionPolled(SubscriptionBase &&) = delete;
	SubscriptionPolled &operator=(SubscriptionBase &&) = delete;

	/**
	 * Check if there is a new update.
	 * */
	bool updated()
	{
		bool isUpdated = false;

		if (orb_check(_handle, &isUpdated) != PX4_OK) {
			PX4_ERR("%s check failed", _meta->o_name);
		}

		return isUpdated;
	}

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool update(void *data)
	{
		bool orb_updated = false;

		if (updated()) {
			if (orb_copy(_meta, _handle, data) != PX4_OK) {
				PX4_ERR("%s copy failed", _meta->o_name);

			} else {
				orb_updated = true;
			}
		}

		return orb_updated;
	}

	//Create an update function that uses the embedded struct.
	bool update() { return update((void *)(&_data)); }
	bool forcedUpdate() { return orb_copy(_meta, _handle, &_data) == PX4_OK; }

	/*
	 * This function gets the T struct data
	 * */
	const T &get() const { return _data; }

	int getHandle() const { return _handle; }

private:

	const orb_metadata *_meta;
	unsigned _instance;
	int _handle{-1};
	T _data{};

};

} // namespace uORB
