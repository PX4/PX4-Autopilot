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
 * PX4 Subscriber API, implements subscribing to messages from a nodehandle
 */
#pragma once

#include <functional>
#include <type_traits>

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
class __EXPORT SubscriberBase
{
public:
	SubscriberBase() {};
	virtual ~SubscriberBase() {};

};

/**
 * Subscriber class which is used by nodehandle
 */
template<typename T>
class __EXPORT Subscriber :
	public SubscriberBase
{
public:
	Subscriber() :
		SubscriberBase(),
		_msg_current()
	{};

	virtual ~Subscriber() {}

	/* Accessors*/
	/**
	 * Get the last message value
	 */
	virtual T& get() {return _msg_current;}

	/**
	 * Get the last native message value
	 */
	virtual decltype(((T*)nullptr)->data()) data()
	{
		return _msg_current.data();
	}

protected:
	T _msg_current;				/**< Current Message value */
};

#if defined(__PX4_ROS)
/**
 * Subscriber class that is templated with the ros n message type
 */
template<typename T>
class SubscriberROS :
	public Subscriber<T>
{
public:
	/**
	 * Construct Subscriber without a callback function
	 */
	SubscriberROS(ros::NodeHandle *rnh) :
		px4::Subscriber<T>(),
		_cbf(NULL),
		_ros_sub(rnh->subscribe(T::handle(), kQueueSizeDefault, &SubscriberROS<T>::callback, this))
	{}

	/**
	 * Construct Subscriber by providing a callback function
	 */
	SubscriberROS(ros::NodeHandle *rnh, std::function<void(const T &)> cbf) :
		_cbf(cbf),
		_ros_sub(rnh->subscribe(T::handle(), kQueueSizeDefault, &SubscriberROS<T>::callback, this))
	{}

	virtual ~SubscriberROS() {};

protected:
	static const uint32_t kQueueSizeDefault = 1;		/**< Size of queue for ROS */

	/**
	 * Called on topic update, saves the current message and then calls the provided callback function
	 * needs to use the native type as it is called by ROS
	 */
	void callback(const typename std::remove_reference<decltype(((T*)nullptr)->data())>::type &msg)
	{
		/* Store data */
		this->_msg_current.data() = msg;

		/* Call callback */
		if (_cbf != NULL) {
			_cbf(this->get());
		}

	}

	ros::Subscriber _ros_sub;		/**< Handle to ros subscriber */
	std::function<void(const T &)> _cbf;	/**< Callback that the user provided on the subscription */

};

#else // Building for NuttX
/**
 * Because we maintain a list of subscribers we need a node class
 */
class __EXPORT SubscriberNode :
	public ListNode<SubscriberNode *>
{
public:
	SubscriberNode(unsigned interval) :
		ListNode(),
		_interval(interval)
	{}

	virtual ~SubscriberNode() {}

	virtual void update() = 0;

	virtual int getUORBHandle() = 0;

	unsigned get_interval() { return _interval; }

protected:
	unsigned _interval;

};

/**
 * Subscriber class that is templated with the uorb subscription message type
 */
template<typename T>
class __EXPORT SubscriberUORB :
	public Subscriber<T>,
	public SubscriberNode
{
public:

	/**
	 * Construct SubscriberUORB by providing orb meta data without callback
	 * @param interval	Minimal interval between calls to callback
	 */
	SubscriberUORB(unsigned interval) :
		SubscriberNode(interval),
		_uorb_sub(new uORB::SubscriptionBase(T::handle(), interval))
	{}

	virtual ~SubscriberUORB() {
		delete _uorb_sub;
	};

	/**
	 * Update Subscription
	 * Invoked by the list traversal in NodeHandle::spinOnce
	 */
	virtual void update()
	{
		if (!_uorb_sub->updated()) {
			/* Topic not updated, do not call callback */
			return;
		}

		_uorb_sub->update(get_void_ptr());
	};

	/* Accessors*/
	int getUORBHandle() { return _uorb_sub->getHandle(); }

protected:
	uORB::SubscriptionBase * _uorb_sub;	/**< Handle to the subscription */

	typename std::remove_reference<decltype(((T*)nullptr)->data())>::type getUORBData()
	{
		return (typename std::remove_reference<decltype(((T*)nullptr)->data())>::type)*_uorb_sub;
	}

	/**
	 * Get void pointer to last message value
	 */
	void *get_void_ptr() { return (void *)&(this->_msg_current.data()); }

};

//XXX reduce to one class with overloaded constructor?
template<typename T>
class __EXPORT SubscriberUORBCallback :
	public SubscriberUORB<T>
{
public:
	/**
	 * Construct SubscriberUORBCallback by providing orb meta data
	 * @param cbf		Callback, executed on receiving a new message
	 * @param interval	Minimal interval between calls to callback
	 */
	SubscriberUORBCallback(unsigned interval,
			       std::function<void(const T &)> cbf) :
		SubscriberUORB<T>(interval),
		_cbf(cbf)
	{}

	virtual ~SubscriberUORBCallback() {};

	/**
	 * Update Subscription
	 * Invoked by the list traversal in NodeHandle::spinOnce
	 * If new data is available the callback is called
	 */
	virtual void update()
	{
		if (!this->_uorb_sub->updated()) {
			/* Topic not updated, do not call callback */
			return;
		}

		/* get latest data */
		this->_uorb_sub->update(this->get_void_ptr());


		/* Check if there is a callback */
		if (_cbf == nullptr) {
			return;
		}

		/* Call callback which performs actions based on this data */
		_cbf(Subscriber<T>::get());

	};

protected:
	std::function<void(const T &)> _cbf;	/**< Callback that the user provided on the subscription */
};
#endif

}
