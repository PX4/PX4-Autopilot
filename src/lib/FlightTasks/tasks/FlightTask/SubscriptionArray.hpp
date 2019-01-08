/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file SubscriptionArray.hpp
 *
 * Simple array that contains a dynamic amount of Subscription<T> instances
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#pragma once

#include <uORB/Subscription.hpp>

class SubscriptionArray
{
public:
	SubscriptionArray() = default;
	~SubscriptionArray();

	/**
	 * Get a subscription
	 * @param meta ORB_ID(topic)
	 * @param subscription returned subscription (output parameter)
	 * @param instance topic instance
	 * @return true on success, false otherwise (subscription set to nullptr)
	 */
	template<class T>
	bool get(const struct orb_metadata *meta, uORB::Subscription<T> *&subscription, unsigned instance = 0);

	/**
	 * update all subscriptions (if new data is available)
	 */
	void update();

	/**
	 * update all subscriptions
	 */
	void forcedUpdate();

private:
	void cleanup();

	bool resizeSubscriptions();

	uORB::SubscriptionNode **_subscriptions{nullptr};
	int _subscriptions_count{0}; ///< number of valid subscriptions
	int _subscriptions_size{0}; ///< actual size of the _subscriptions array
};


template<class T>
bool SubscriptionArray::get(const struct orb_metadata *meta, uORB::Subscription<T> *&subscription, unsigned instance)
{
	// does it already exist?
	for (int i = 0; i < _subscriptions_count; ++i) {
		if (_subscriptions[i]->get_topic() == meta && _subscriptions[i]->get_instance() == instance) {
			// we know the type must be correct, so we can use reinterpret_cast (dynamic_cast is not available)
			subscription = reinterpret_cast<uORB::Subscription<T>*>(_subscriptions[i]);
			return true;
		}
	}

	// resize if needed
	if (_subscriptions_count >= _subscriptions_size) {
		if (!resizeSubscriptions()) {
			subscription = nullptr;
			return false;
		}
	}

	subscription = new uORB::Subscription<T>(meta, 0, instance);

	if (!subscription) {
		return false;
	}

	_subscriptions[_subscriptions_count++] = subscription;
	return true;
}
