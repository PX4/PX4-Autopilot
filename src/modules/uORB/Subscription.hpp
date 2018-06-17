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

namespace uORB
{

/**
 * Base subscription wrapper class, used in list traversal
 * of various subscriptions.
 */
class __EXPORT SubscriptionBase
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
	SubscriptionBase(const struct orb_metadata *meta, unsigned interval = 0, unsigned instance = 0);
	virtual ~SubscriptionBase();

	// no copy, assignment, move, move assignment
	SubscriptionBase(const SubscriptionBase &) = delete;
	SubscriptionBase &operator=(const SubscriptionBase &) = delete;
	SubscriptionBase(SubscriptionBase &&) = delete;
	SubscriptionBase &operator=(SubscriptionBase &&) = delete;

	/**
	 * Check if there is a new update.
	 * */
	bool updated();

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool update(void *data);

	int getHandle() const { return _handle; }

	const orb_metadata *getMeta() const { return _meta; }

	unsigned getInstance() const { return _instance; }

protected:
	const struct orb_metadata *_meta;
	unsigned _instance;
	int _handle;
};

/**
 * alias class name so it is clear that the base class
 */
typedef SubscriptionBase SubscriptionTiny;

/**
 * The subscription base class as a list node.
 */
class __EXPORT SubscriptionNode : public SubscriptionBase, public ListNode<SubscriptionNode *>
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
	SubscriptionNode(const struct orb_metadata *meta, unsigned interval = 0, unsigned instance = 0,
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
class __EXPORT Subscription : public SubscriptionNode
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
	Subscription(const struct orb_metadata *meta, unsigned interval = 0, unsigned instance = 0,
		     List<SubscriptionNode *> *list = nullptr):
		SubscriptionNode(meta, interval, instance, list),
		_data() // initialize data structure to zero
	{}

	~Subscription() override = default;

	// no copy, assignment, move, move assignment
	Subscription(const Subscription &) = delete;
	Subscription &operator=(const Subscription &) = delete;
	Subscription(Subscription &&) = delete;
	Subscription &operator=(Subscription &&) = delete;

	/**
	 * Create an update function that uses the embedded struct.
	 */
	bool update() override
	{
		return SubscriptionBase::update((void *)(&_data));
	}

	bool forcedUpdate() override
	{
		return orb_copy(_meta, _handle, &_data) == PX4_OK;
	}

	/*
	 * This function gets the T struct data
	 * */
	const T &get() const
	{
		return _data;
	}

private:
	T _data;
};

} // namespace uORB
