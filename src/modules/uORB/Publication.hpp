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
 * @file Publication.h
 *
 */

#pragma once

#include <assert.h>

#include <uORB/uORB.h>
#include <containers/List.hpp>


namespace uORB
{

/**
 * Base publication warapper class, used in list traversal
 * of various publications.
 */
class __EXPORT PublicationBase
{
public:

	/**
	 * Constructor
	 *
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 */
	PublicationBase(const struct orb_metadata *meta) :
		_meta(meta),
		_handle(nullptr)
	{
	}

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	void update(void *data)
	{
		if (_handle != nullptr) {
			orb_publish(getMeta(), getHandle(), data);

		} else {
			setHandle(orb_advertise(getMeta(), data));
		}
	}

	/**
	 * Deconstructor
	 */
	virtual ~PublicationBase()
	{
	}
// accessors
	const struct orb_metadata *getMeta() { return _meta; }
	orb_advert_t getHandle() { return _handle; }
protected:
// accessors
	void setHandle(orb_advert_t handle) { _handle = handle; }
// attributes
	const struct orb_metadata *_meta;
	orb_advert_t _handle;
private:
	// forbid copy
	PublicationBase(const PublicationBase &) : _meta(), _handle() {};
	// forbid assignment
	PublicationBase &operator = (const PublicationBase &);
};

/**
 * alias class name so it is clear that the base class
 * can be used by itself if desired
 */
typedef PublicationBase PublicationTiny;

/**
 * The publication base class as a list node.
 */
class __EXPORT PublicationNode :
	public PublicationBase,
	public ListNode<PublicationNode *>
{
public:
	/**
	 * Constructor
	 *
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param list 	A pointer to a list of subscriptions
	 * 	that this should be appended to.
	 */
	PublicationNode(const struct orb_metadata *meta,
			List<PublicationNode *> *list = nullptr);

	/**
	 * This function is the callback for list traversal
	 * updates, a child class must implement it.
	 */
	virtual void update() = 0;
};

/**
 * Publication wrapper class
 */
template<class T>
class __EXPORT Publication :
	public T, // this must be first!
	public PublicationNode
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param list A list interface for adding to
	 * 	list during construction
	 */
	Publication(const struct orb_metadata *meta,
		    List<PublicationNode *> *list = nullptr);

	/**
	 * Deconstructor
	 **/
	virtual ~Publication();

	/*
	 * XXX
	 * This function gets the T struct, assuming
	 * the struct is the first base class, this
	 * should use dynamic cast, but doesn't
	 * seem to be available
	 */
	void *getDataVoidPtr();

	/**
	 * Create an update function that uses the embedded struct.
	 */
	void update()
	{
		PublicationBase::update(getDataVoidPtr());
	}
};

} // namespace uORB
