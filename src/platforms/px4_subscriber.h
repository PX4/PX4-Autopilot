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

#include <functional>

#if defined(__PX4_ROS)
/* includes when building for ros */
#include "ros/ros.h"
#else
/* includes when building for NuttX */
#include <uORB/Subscription.hpp>
#include <containers/List.hpp>
#include "px4_nodehandle.h"
#endif

namespace px4
{

/**
 * Untemplated subscriber base class
 * */
class SubscriberBase
{
public:
	SubscriberBase() {};
	~SubscriberBase() {};

};

/**
 * Subscriber class which is used by nodehandle
 */
template<typename M>
class Subscriber :
	public SubscriberBase
{
public:
	Subscriber() :
		SubscriberBase()
	{};
	~Subscriber() {};

	/* Accessors*/
	/**
	 * Get the last message value
	 */
	virtual M get() = 0;

	/**
	 * Get void pointer to last message value
	 */
	virtual void * get_void_ptr() = 0;
};

#if defined(__PX4_ROS)
/**
 * Subscriber class that is templated with the ros n message type
 */
template<typename M>
class SubscriberROS :
	public Subscriber<M>
{
friend class NodeHandle;

public:
	/**
	 * Construct Subscriber by providing a callback function
	 */
	SubscriberROS(std::function<void(const M &)> cbf) :
		Subscriber<M>(),
		_ros_sub(),
		_cbf(cbf),
		_msg_current()
	{}

	/**
	 * Construct Subscriber without a callback function
	 */
	SubscriberROS() :
		Subscriber<M>(),
		_ros_sub(),
		_cbf(NULL),
		_msg_current()
	{}


	~SubscriberROS() {};

	/* Accessors*/
	/**
	 * Get the last message value
	 */
	M get() { return _msg_current; }
	/**
	 * Get void pointer to last message value
	 */
	void * get_void_ptr() { return (void*)&_msg_current; }

protected:
	/**
	 * Called on topic update, saves the current message and then calls the provided callback function
	 */
	void callback(const M &msg) {
		/* Store data */
		_msg_current = msg;

		/* Call callback */
		if (_cbf != NULL) {
			_cbf(msg);
		}

	}

	/**
	 * Saves the ros subscriber to keep ros subscription alive
	 */
	void set_ros_sub(ros::Subscriber ros_sub) {
		_ros_sub = ros_sub;
	}

	ros::Subscriber _ros_sub;		/**< Handle to ros subscriber */
	std::function<void(const M &)> _cbf;	/**< Callback that the user provided on the subscription */
	M _msg_current;				/**< Current Message value */

};

#else // Building for NuttX

/**
 * Subscriber class that is templated with the uorb subscription message type
 */
template<typename M>
class SubscriberUORB :
	public Subscriber<M>,
	public uORB::Subscription<M>
{
public:

	/**
	 * Construct SubscriberUORB by providing orb meta data without callback
	 * @param meta	    orb metadata for the topic which is used
	 * @param interval	Minimal interval between calls to callback
	 * @param list	    subscriber is added to this list
	 */
	SubscriberUORB(const struct orb_metadata *meta,
		      unsigned interval,
		      List<uORB::SubscriptionNode *> *list) :
		Subscriber<M>(),
		uORB::Subscription<M>(meta, interval, list)
	{}

	~SubscriberUORB() {};

	/**
	 * Update Subscription
	 * Invoked by the list traversal in NodeHandle::spinOnce
	 */
	virtual void update()
	{
		if (!uORB::Subscription<M>::updated()) {
			/* Topic not updated */
			return;
		}

		/* get latest data */
		uORB::Subscription<M>::update();
	};

	/* Accessors*/
	/**
	 * Get the last message value
	 */
	M get() { return uORB::Subscription<M>::getData(); }
	/**
	 * Get void pointer to last message value
	 */
	void * get_void_ptr() { return uORB::Subscription<M>::getDataVoidPtr(); }
};

template<typename M>
class SubscriberUORBCallback :
	public SubscriberUORB<M>
{
public:
	/**
	 * Construct SubscriberUORBCallback by providing orb meta data
	 * @param meta	    orb metadata for the topic which is used
	 * @param callback	Callback, executed on receiving a new message
	 * @param interval	Minimal interval between calls to callback
	 * @param list	    subscriber is added to this list
	 */
	SubscriberUORBCallback(const struct orb_metadata *meta,
		      unsigned interval,
		      std::function<void(const M &)> callback,
		      List<uORB::SubscriptionNode *> *list) :
		SubscriberUORB<M>(meta, interval, list),
		_callback(callback)
	{}

	~SubscriberUORBCallback() {};

	/**
	 * Update Subscription
	 * Invoked by the list traversal in NodeHandle::spinOnce
	 * If new data is available the callback is called
	 */
	virtual void update()
	{
		if (!uORB::Subscription<M>::updated()) {
			/* Topic not updated, do not call callback */
			return;
		}

		/* get latest data */
		uORB::Subscription<M>::update();


		/* Check if there is a callback */
		if (_callback == nullptr) {
			return;
		}

		/* Call callback which performs actions based on this data */
		_callback(uORB::Subscription<M>::getData());

	};

protected:
	std::function<void(const M &)> _callback;	/**< Callback handle,
							  called when new data is available */
};
#endif

}
