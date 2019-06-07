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

/**
 * @file SubscriptionCallback.hpp
 *
 */

#pragma once

#include <uORB/SubscriptionInterval.hpp>
#include <containers/List.hpp>
#include <px4_work_queue/WorkItem.hpp>

namespace uORB
{

// Subscription wrapper class with callbacks on new publications
class SubscriptionCallback : public SubscriptionInterval, public ListNode<SubscriptionCallback *>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionCallback() = default;
	SubscriptionCallback(const orb_metadata *meta, uint8_t interval_ms, uint8_t instance = 0) :
		SubscriptionInterval(meta, interval_ms, instance)
	{
		register_callback();
	}

	virtual ~SubscriptionCallback()
	{
		unregister_callback();
	};

	bool init() override
	{
		SubscriptionInterval::init();
		return register_callback();
	}

	bool register_callback() { return _subscription.get_node() ? _subscription.get_node()->register_callback(this) : false; }
	void unregister_callback() { if (_subscription.get_node()) _subscription.get_node()->unregister_callback(this); }

	bool change_topic(const orb_metadata *meta, uint8_t instance = 0)
	{
		unregister_callback();
		_subscription.change_topic(meta, instance);
		return register_callback();
	}

	virtual void call() = 0;

};

// Subscription with callback that schedules a WorkItem
class SubscriptionCallbackWorkItem : public SubscriptionCallback
{
public:
	/**
	 * Constructor
	 *
	 * @param work_item The WorkItem that will be scheduled immediately on new publications.
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionCallbackWorkItem() = default;
	SubscriptionCallbackWorkItem(px4::WorkItem *work_item, const orb_metadata *meta, uint8_t instance = 0) :
		SubscriptionCallback(meta, instance),
		_work_item(work_item)
	{
	}

	virtual ~SubscriptionCallbackWorkItem() = default;

	void call() override
	{
		if (hrt_absolute_time() >= next_update_time()) {
			_work_item->ScheduleNow();
		}
	}

private:
	px4::WorkItem *_work_item;
};

} // namespace uORB
