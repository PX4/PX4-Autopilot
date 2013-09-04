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
 * @file UOrbPublication.h
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
 * Base publication warapper class, used in list traversal
 * of various publications.
 */
class __EXPORT UOrbPublicationBase : public ListNode<control::UOrbPublicationBase *>
{
public:

	UOrbPublicationBase(
		List<UOrbPublicationBase *> * list,
		const struct orb_metadata *meta) :
		_meta(meta),
		_handle(-1) {
		if (list != NULL) list->add(this);
	}
	void update() {
		if (_handle > 0) {
			orb_publish(getMeta(), getHandle(), getDataVoidPtr());
		} else {
			setHandle(orb_advertise(getMeta(), getDataVoidPtr()));
		}
	}
	virtual void *getDataVoidPtr() = 0;
	virtual ~UOrbPublicationBase() {
		orb_unsubscribe(getHandle());
	}
	const struct orb_metadata *getMeta() { return _meta; }
	int getHandle() { return _handle; }
protected:
	void setHandle(orb_advert_t handle) { _handle = handle; }
	const struct orb_metadata *_meta;
	orb_advert_t _handle;
};

/**
 * UOrb Publication wrapper class
 */
template<class T>
class UOrbPublication :
	public T, // this must be first!
	public UOrbPublicationBase
{
public:
	/**
	 * Constructor
	 *
	 * @param list      A list interface for adding to list during construction
	 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
	 *			for the topic.
	 */
	UOrbPublication(
		List<UOrbPublicationBase *> * list,
		const struct orb_metadata *meta) :
		T(), // initialize data structure to zero
		UOrbPublicationBase(list, meta) {
	}
	virtual ~UOrbPublication() {}
	/*
	 * XXX
	 * This function gets the T struct, assuming
	 * the struct is the first base class, this
	 * should use dynamic cast, but doesn't
	 * seem to be available
	 */
	void *getDataVoidPtr() { return (void *)(T *)(this); }
};

} // namespace control
