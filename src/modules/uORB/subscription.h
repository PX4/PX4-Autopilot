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
 * @file subscription.h
 *
 */

#pragma once

#include <uORB/uORB.h>


namespace uORB
{

/**
 * Base uORB subscription wrapper class, to use with external data buffer.
 */
class __EXPORT Subscription
{
private:
	const struct orb_metadata *_meta;
	int _handle;

public:
	Subscription(const struct orb_metadata *meta) : _meta(meta) {
		_handle = orb_subscribe(_meta);
	}

	/**
	 * Check for updates on the subscription.
	 */
	bool updated() {
		bool updated_flag;
		return !orb_check(_handle, &updated_flag) && updated_flag;
	}

	/**
	 * Update subscription. If update available put it to 'data' and return 'true'.
	 * If update is not available or error return 'false'.
	 */
	bool update(void *data) {
		bool updated_flag;
		if (!orb_check(_handle, &updated_flag) && updated_flag) {
			return !orb_copy(_meta, _handle, data);
		}

		return false;
	}

	/**
	 * Copy topic to 'data'. Return 'true' if topic was published at least once.
	 * If topic was never published or error return 'false'.
	 */
	bool copy(void *data) {
		return !orb_copy(_meta, _handle, data);
	}

	virtual ~Subscription() {
		orb_unsubscribe(_handle);
	}

	const struct orb_metadata *get_meta() const {
		return _meta;
	}

	int get_handle() const {
		return _handle;
	}
};

/**
 * uORB subscription wrapper class with data buffer.
 */
template <class T>
class __EXPORT BufferedSubscription
{
private:
	const struct orb_metadata *_meta;
	int _handle;
	T _data;

public:
	BufferedSubscription(const struct orb_metadata *meta) : _meta(meta), _data({0}) {
		_handle = orb_subscribe(_meta);
	}

	~BufferedSubscription() {
		orb_unsubscribe(_handle);
	}

	/**
	 * Update subscription. Return 'true' only if subscription was updated successfully.
	 */
	bool update() {
		bool updated_flag;
		if (!orb_check(_handle, &updated_flag) && updated_flag) {
			return !orb_copy(_meta, _handle, &_data);
		}

		return false;
	}

	const struct orb_metadata *get_meta() const {
		return _meta;
	}

	int get_handle() const {
		return _handle;
	}

	T& get_data() {
		return _data;
	}
};

} // namespace uORB
