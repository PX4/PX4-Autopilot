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
 * @file Publication.h
 *
 */

#pragma once

#include <uORB/uORB.h>


namespace uORB
{

/**
 * Base publication warapper class, used in list traversal
 * of various publications.
 */
class __EXPORT PublicationBase
{
public:
	PublicationBase(const struct orb_metadata *meta) : _meta(meta), _handle(-1) {}

	void publish() {
		if (_handle > 0) {
			orb_publish(get_meta(), get_handle(), get_data_ptr());

		} else {
			set_handle(orb_advertise(get_meta(), get_data_ptr()));
		}
	}

	virtual void *get_data_ptr() {
		return (void *)this;
	}

	virtual ~PublicationBase() {
		if (_handle > 0) {
			orb_unsubscribe(get_handle());
		}
	}

	const struct orb_metadata *get_meta() const {
		return _meta;
	}

	const int get_handle() const {
		return _handle;
	}

protected:
	const struct orb_metadata *_meta;
	orb_advert_t _handle;

	void set_handle(const orb_advert_t handle) {
		_handle = handle;
	}
};

/**
 * Publication wrapper class
 */
template<class T>
class __EXPORT Publication :
	public T, // this must be first!
	public PublicationBase
{
public:
	/**
	 * Constructor
	 *
	 * @param list      A list interface for adding to list during construction
	 * @param meta		The uORB metadata (usually from the ORB_ID() macro)
	 *			for the topic.
	 */
	Publication(const struct orb_metadata *meta) : T(), PublicationBase(meta) {}

	~Publication() {}
};

} // namespace uORB
