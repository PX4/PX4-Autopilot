/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include "SubscriptionCallback.hpp"

#include <pthread.h>

namespace uORB
{

// Type-independent part of SubscriptionBlocking, kept out-of-line so it is
// emitted once instead of once per topic type.
class SubscriptionBlockingBase : public SubscriptionCallback
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval_us The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionBlockingBase(const orb_metadata *meta, uint32_t interval_us, uint8_t instance);

	virtual ~SubscriptionBlockingBase();

	void call(unsigned generation) override;

	/**
	 * Block until updated.
	 * @param timeout_us The requested timeout in microseconds, or 0 to wait indefinitely.
	 *
	 * @return true only if topic was updated
	 */
	bool updatedBlocking(uint32_t timeout_us = 0);

private:

	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t	_cv = PTHREAD_COND_INITIALIZER;

};

// Subscription with callback that schedules a WorkItem
template<typename T>
class SubscriptionBlocking : public SubscriptionBlockingBase
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval_us The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionBlocking(const orb_metadata *meta, uint32_t interval_us = 0, uint8_t instance = 0) :
		SubscriptionBlockingBase(meta, interval_us, instance)
	{
	}

	/**
	 * Copy the struct if updated.
	 * @param data The data reference where the struct will be copied.
	 * @param timeout_us The requested timeout in microseconds, or 0 to wait indefinitely.
	 *
	 * @return true only if topic was updated and copied successfully.
	 */
	bool updateBlocking(T &data, uint32_t timeout_us = 0)
	{
		if (updatedBlocking(timeout_us)) {
			return copy(&data);
		}

		return false;
	}
};

} // namespace uORB
