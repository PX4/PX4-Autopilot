/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file UOrbSubscription.h
 *
 */

#pragma once

#include <uORB/uORB.h>
#include "../block/Block.hpp"
#include "../block/List.hpp"


namespace control
{

class Block;

/**
 * Base subscription warapper class, used in list traversal
 * of various subscriptions.
 */
class __EXPORT UOrbSubscriptionBase :
	public ListNode<control::UOrbSubscriptionBase *>
{
public:
// methods

	/**
	 * Constructor
	 *
	 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
	 *			for the topic.
	 */
	UOrbSubscriptionBase(
		List<UOrbSubscriptionBase *> * list,
		const struct orb_metadata *meta) :
		_meta(meta),
		_handle() {
		if (list != NULL) list->add(this);
	}
	bool updated();
	void update() {
		if (updated()) {
			orb_copy(_meta, _handle, getDataVoidPtr());
		}
	}
	virtual void *getDataVoidPtr() = 0;
	virtual ~UOrbSubscriptionBase() {
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
};

/**
 * UOrb Subscription wrapper class
 */
template<class T>
class __EXPORT UOrbSubscription :
	public T, // this must be first!
	public UOrbSubscriptionBase
{
public:
	/**
	 * Constructor
	 *
	 * @param list      A list interface for adding to list during construction
	 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
	 *			for the topic.
	 * @param interval  The minimum interval in milliseconds between updates
	 */
	UOrbSubscription(
		List<UOrbSubscriptionBase *> * list,
		const struct orb_metadata *meta, unsigned interval) :
		T(), // initialize data structure to zero
		UOrbSubscriptionBase(list, meta) {
		setHandle(orb_subscribe(getMeta()));
		orb_set_interval(getHandle(), interval);
	}

	/**
	 * Deconstructor
	 */
	virtual ~UOrbSubscription() {}

	/*
	 * XXX
	 * This function gets the T struct, assuming
	 * the struct is the first base class, this
	 * should use dynamic cast, but doesn't
	 * seem to be available
	 */
	void *getDataVoidPtr() { return (void *)(T *)(this); }
	T getData() { return T(*this); }
};

} // namespace control
