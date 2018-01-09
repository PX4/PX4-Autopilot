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
 * @file Publication.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <containers/List.hpp>
#include <systemlib/err.h>

namespace uORB
{

/**
 * Base publication wrapper class, used in list traversal
 * of various publications.
 */
class __EXPORT PublicationBase
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param priority The priority for multi pub/sub, 0-based, -1 means
	 * 	don't publish as multi
	 */
	PublicationBase(const struct orb_metadata *meta, int priority = -1);

	virtual ~PublicationBase();

	// no copy, assignment, move, move assignment
	PublicationBase(const PublicationBase &) = delete;
	PublicationBase &operator=(const PublicationBase &) = delete;
	PublicationBase(PublicationBase &&) = delete;
	PublicationBase &operator=(PublicationBase &&) = delete;

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool update(void *data);

protected:
	const struct orb_metadata *_meta;
	const int _priority;

	orb_advert_t _handle{nullptr};
};

/**
 * alias class name so it is clear that the base class
 * can be used by itself if desired
 */
typedef PublicationBase PublicationTiny;

/**
 * The publication base class as a list node.
 */
class __EXPORT PublicationNode : public PublicationBase, public ListNode<PublicationNode *>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param priority The priority for multi pub, 0-based.
	 * @param list A list interface for adding to
	 * 	list during construction
	 */
	PublicationNode(const struct orb_metadata *meta, int priority = -1, List<PublicationNode *> *list = nullptr);
	virtual ~PublicationNode() override = default;

	/**
	 * This function is the callback for list traversal
	 * updates, a child class must implement it.
	 */
	virtual bool update() = 0;
};

/**
 * Publication wrapper class
 */
template<class T>
class __EXPORT Publication final : public PublicationNode
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param priority The priority for multi pub, 0-based.
	 * @param list A list interface for adding to
	 * 	list during construction
	 */
	Publication(const struct orb_metadata *meta, int priority = -1, List<PublicationNode *> *list = nullptr)  :
		PublicationNode(meta, priority, list),
		_data()
	{
	}

	~Publication() override = default;

	// no copy, assignment, move, move assignment
	Publication(const Publication &) = delete;
	Publication &operator=(const Publication &) = delete;
	Publication(Publication &&) = delete;
	Publication &operator=(Publication &&) = delete;

	/*
	 * This function gets the T struct
	 * */
	T &get() { return _data; }

	/**
	 * Create an update function that uses the embedded struct.
	 */
	bool update() override
	{
		return PublicationBase::update((void *)(&_data));
	}

private:
	T _data;
};

} // namespace uORB
