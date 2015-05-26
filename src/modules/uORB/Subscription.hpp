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
	 *
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 */
	SubscriptionBase(const struct orb_metadata *meta,
		unsigned interval=0) :
		_meta(meta),
		_handle() {
		setHandle(orb_subscribe(getMeta()));
		orb_set_interval(getHandle(), interval);
	}

	/**
	 * Check if there is a new update.
	 * */
	bool updated() {
		bool isUpdated = false;
		orb_check(_handle, &isUpdated);
		return isUpdated;
	}

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	void update(void * data) {
		if (updated()) {
			orb_copy(_meta, _handle, data);
		}
	}

	/**
	 * Deconstructor
	 */
	virtual ~SubscriptionBase() {
		orb_unsubscribe(_handle);
	}
// accessors
	const struct orb_metadata *getMeta() { return _meta; }
	int getHandle() { return _handle; }
protected:
// accessors
	void setHandle(int handle) { _handle = handle; }
// attributes
	const struct orb_metadata *_meta;
	int _handle;
private:
	// forbid copy
	SubscriptionBase(const SubscriptionBase& other);
	// forbid assignment
	SubscriptionBase& operator = (const SubscriptionBase &);
};

/**
 * alias class name so it is clear that the base class
 */
typedef SubscriptionBase SubscriptionTiny;

/**
 * The publication base class as a list node.
 */
class __EXPORT SubscriptionNode :

	public SubscriptionBase,
	public ListNode<SubscriptionNode *>
{
public:
	/**
	 * Constructor
	 *
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param list 	A pointer to a list of subscriptions
	 * 	that this should be appended to.
	 */
	SubscriptionNode(const struct orb_metadata *meta,
		unsigned interval=0,
		List<SubscriptionNode *> * list=nullptr) :
		SubscriptionBase(meta, interval),
		_interval(interval) {
		if (list != nullptr) list->add(this);
	}

	/**
	 * This function is the callback for list traversal
	 * updates, a child class must implement it.
	 */
	virtual void update() = 0;
// accessors
	unsigned getInterval() { return _interval; }
protected:
// attributes
	unsigned _interval;

};

/**
 * Subscription wrapper class
 */
template<class T>
class __EXPORT Subscription :
	public T, // this must be first!
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
		unsigned interval=0,
		List<SubscriptionNode *> * list=nullptr);
	/**
	 * Deconstructor
	 */
	virtual ~Subscription();


	/**
	 * Create an update function that uses the embedded struct.
	 */
	void update() {
		SubscriptionBase::update(getDataVoidPtr());
	}

	/*
	 * XXX
	 * This function gets the T struct, assuming
	 * the struct is the first base class, this
	 * should use dynamic cast, but doesn't
	 * seem to be available
	 */
	void *getDataVoidPtr();
	T getData();
};

} // namespace uORB
