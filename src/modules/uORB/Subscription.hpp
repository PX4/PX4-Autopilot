/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file Subscription.h
 *
 */

#pragma once

#include <assert.h>

#include <uORB/uORB.h>
#include <containers/List.hpp>
#include <systemlib/err.h>

namespace uORB
{

/**
 * Base subscription warapper class, used in list traversal
 * of various subscriptions.
 */
class __EXPORT SubscriptionBase
{
public:
// methods

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param instance The instance for multi sub.
	 */
	SubscriptionBase(const struct orb_metadata *meta,
			 unsigned interval = 0, unsigned instance = 0);

	/**
	 * Check if there is a new update.
	 * */
	bool updated();

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	void update(void *data);

	/**
	 * Deconstructor
	 */
	virtual ~SubscriptionBase();

// accessors
	const struct orb_metadata *getMeta() { return _meta; }
	int getHandle() const { return _handle; }

	unsigned getInterval() const
	{
		unsigned int interval;
		orb_get_interval(getHandle(), &interval);
		return interval;
	}
protected:
// accessors
	void setHandle(int handle) { _handle = handle; }
// attributes
	const struct orb_metadata *_meta;
	int _instance;
	int _handle;
private:
	// disallow copy
	SubscriptionBase(const SubscriptionBase &other);
	// disallow assignment
	SubscriptionBase &operator=(const SubscriptionBase &other);
};

/**
 * alias class name so it is clear that the base class
 */
typedef SubscriptionBase SubscriptionTiny;

/**
 * The subscription base class as a list node.
 */
class __EXPORT SubscriptionNode :

	public SubscriptionBase,
	public ListNode<SubscriptionNode *>
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
	SubscriptionNode(const struct orb_metadata *meta,
			 unsigned interval = 0,
			 int instance = 0,
			 List<SubscriptionNode *> *list = nullptr) :
		SubscriptionBase(meta, interval, instance)
	{
		if (list != nullptr) { list->add(this); }
	}

	/**
	 * This function is the callback for list traversal
	 * updates, a child class must implement it.
	 */
	virtual void update() = 0;

};

/**
 * Subscription wrapper class
 */
template<class T>
class __EXPORT Subscription :
	public SubscriptionNode
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
	Subscription(const struct orb_metadata *meta,
		     unsigned interval = 0,
		     int instance = 0,
		     List<SubscriptionNode *> *list = nullptr);

	Subscription(const Subscription &);

	/**
	 * Deconstructor
	 */
	virtual ~Subscription();


	/**
	 * Create an update function that uses the embedded struct.
	 */
	void update();

	/**
	 * Create an update function that uses the embedded struct.
	 */
	bool check_updated();
	/*
	 * This function gets the T struct data
	 * */
	const T &get() const { return _data; }
private:
	T _data;
};

} // namespace uORB
