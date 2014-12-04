/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file px4_subscriber.h
 *
 * PX4 Middleware Wrapper Subscriber
 */
#pragma once
#if defined(__linux) || (defined(__APPLE__) && defined(__MACH__))
/* includes when building for ros */
#include "ros/ros.h"
#else
/* includes when building for NuttX */
#include <uORB/Subscription.hpp>
#include <containers/List.hpp>
#include <functional>
#endif

namespace px4
{

#if defined(__linux) || (defined(__APPLE__) && defined(__MACH__))
class Subscriber
{
public:
	/**
	 * Construct Subscriber by providing a ros::Subscriber
	 * @param ros_sub the ros subscriber which will be used to perform the publications
	 */
	Subscriber(ros::Subscriber ros_sub) :
		_ros_sub(ros_sub)
	{}

	~Subscriber() {};
private:
	ros::Subscriber _ros_sub;	/**< Handle to ros subscriber */
};

#else

/**
 * Subscriber class which is used by nodehandle when building for NuttX
 */
class Subscriber
{
public:
	Subscriber() {};
	~Subscriber() {};
private:
};

/**
 * Subscriber class that is templated with the uorb subscription message type
 */
template<typename M>
class SubscriberPX4 :
	public Subscriber,
	public uORB::Subscription<M>
{
public:
	/**
	 * Construct SubscriberPX4 by providing orb meta data
	 * @param meta	    orb metadata for the topic which is used
	 * @param callback	Callback, executed on receiving a new message
	 * @param interval	Minimal interval between calls to callback
	 * @param list	    subscriber is added to this list
	 */
	SubscriberPX4(const struct orb_metadata *meta,
		      unsigned interval,
		      std::function<void(const M &)> callback,
		      List<uORB::SubscriptionNode *> *list) :
		Subscriber(),
		uORB::Subscription<M>(meta, interval, list),
		_callback(callback)
		//XXX store callback
	{}

	~SubscriberPX4() {};

	/**
	 * Update Subscription
	 * Invoked by the list traversal in NodeHandle::spinOnce
	 * If new data is available the callback is called
	 */
	void update()
	{
		if (!uORB::Subscription<M>::updated()) {
			/* Topic not updated, do not call callback */
			return;
		}

		/* get latest data */
		uORB::Subscription<M>::update();

		/* Call callback which performs actions based on this data */
		_callback(uORB::Subscription<M>::getData());

	};
private:
	std::function<void(const M &)> _callback;	/**< Callback handle,
							  called when new data is available */

};
#endif

}
