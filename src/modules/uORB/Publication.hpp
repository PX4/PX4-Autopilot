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
#include <systemlib/err.h>

namespace uORB
{

/**
 * Base publication wrapper class, used in list traversal
 * of various publications.
 */
template<typename T>
class Publication
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
	Publication(const orb_metadata *meta, int priority = -1) : _meta(meta), _priority(priority) {}

	~Publication() { orb_unadvertise(_handle); }

	/**
	 * Publish the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool publish(const T &data)
	{
		bool updated = false;

		if (_handle != nullptr) {
			if (orb_publish(_meta, _handle, &data) != PX4_OK) {
				PX4_ERR("%s publish fail", _meta->o_name);

			} else {
				updated = true;
			}

		} else {
			orb_advert_t handle = nullptr;

			if (_priority > 0) {
				int instance;
				handle = orb_advertise_multi(_meta, &data, &instance, _priority);

			} else {
				handle = orb_advertise(_meta, &data);
			}

			if (handle != nullptr) {
				_handle = handle;
				updated = true;

			} else {
				PX4_ERR("%s advert fail", _meta->o_name);
			}
		}

		return updated;
	}

protected:
	const orb_metadata *_meta;
	const int _priority;

	orb_advert_t _handle{nullptr};
};

/**
 * The publication base class as a list node.
 */
template<typename T>
class PublicationData : public Publication<T>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param priority The priority for multi pub, 0-based.
	 */
	PublicationData(const orb_metadata *meta, int priority = -1) : Publication<T>(meta, priority) {}
	~PublicationData() = default;

	T	&get() { return _data; }
	void	set(const T &data) { _data = data; }

	// Publishes the embedded struct.
	bool	update() { return Publication<T>::publish(_data); }
	bool	update(const T &data)
	{
		_data = data;
		return Publication<T>::publish(_data);
	}

private:
	T _data{};
};

} // namespace uORB
